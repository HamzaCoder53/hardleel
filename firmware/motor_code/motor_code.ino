// motor_test.ino
// ESP32 + 2x BTS7960 (IBT-2) + 2 servos: serial test sketch
// Requires the ESP32 Arduino core 3.x (Boards Manager: "esp32" by Espressif)
//
// Commands (Serial Monitor at 115200 baud, line ending "Newline"):
//   m <left> <right> [ms]  speeds from -255 to 255; runs for ms milliseconds
//                          (default 1000), use 0 to run until stopped
//   s <angle>              move both servos to 0-180 degrees
//   x                      stop both motors
//   e <hz>                 encoder stream rate (0 = off, default 20); e r = reset counts
//   h                      show help
//
// DIY wheel encoders (one single-channel sensor per wheel: slotted opto LM393 module,
// TCRT5000 reflective + printed stripe disc, or hall sensor + magnets). See docs/WHEEL_ENCODERS.md.
// Stream line:  E <millis> <lf> <lr> <rf> <rr>   cumulative signed counts (edges).
// The sensors cannot see direction: each count takes the sign of that SIDE's last
// commanded direction (same sign convention as 'm', before any Pi-side inversion).

#include <Arduino.h>

// ---- Pins ----
// Module with GPIO 25 + 33 is treated as the left side,
// module with GPIO 26 + 27 as the right side.
const int L_RPWM = 33; 
const int L_LPWM = 25; 
const int R_RPWM = 26; 
const int R_LPWM = 27; 
const int EN_PIN = 32;   // all four R_EN/L_EN pins, 10k pull-down to GND
const int SERVO1 = 13;
const int SERVO2 = 4;
// Encoder inputs (free pins with internal pull-ups, no boot-strap role). Sensor modules
// MUST be powered from 3.3 V (not 5 V) so their output never exceeds 3.3 V.
DRAM_ATTR const int ENC_PIN[4] = {18, 19, 22, 23};   // left-front, left-rear, right-front, right-rear
DRAM_ATTR const int ENC_SIDE[4] = {0, 0, 1, 1};      // 0 = left, 1 = right
const uint32_t ENC_MIN_EDGE_US = 250;      // ignore edges closer than this (PWM noise / bounce)

// ---- Settings ----
const bool LEFT_INVERT  = false;  // set true if the left side runs backwards
const bool RIGHT_INVERT = false;  // set true if the right side runs backwards
const int  MAX_SPEED    = 255;    // lower this to cap speed while testing
const int  RAMP_STEP    = 5;      // duty change per ramp tick
const unsigned long RAMP_INTERVAL_MS = 5;    // 0 to full speed in about 0.25 s
const unsigned long DEFAULT_RUN_MS   = 1000;

const char*    FW_VERSION = "motor_code coast-at-zero + encoders 2026-11-16";  // printed at boot and by 'h'

const uint32_t MOTOR_FREQ = 20000;  // 20 kHz
const uint8_t  MOTOR_RES  = 8;      // duty 0-255
const uint32_t SERVO_FREQ = 50;     // standard 20 ms servo frame
const uint8_t  SERVO_RES  = 16;

int targetL = 0, targetR = 0;
int currentL = 0, currentR = 0;
bool timedRun = false;
unsigned long stopAt = 0;
unsigned long lastRamp = 0;
String inputLine;

// ---- Encoders ----
volatile int32_t  encCount[4] = {0, 0, 0, 0};
volatile uint32_t encLastUs[4] = {0, 0, 0, 0};
volatile uint8_t  encLevel[4] = {0, 0, 0, 0};
volatile int8_t   sideDir[2] = {1, 1};        // last non-zero commanded direction per side
uint32_t encPeriodMs = 50;                    // 20 Hz stream, 0 = off
unsigned long lastEncPrint = 0;

// One ISR per pin: count only a real level change that is not too close to the previous one.
static inline void IRAM_ATTR encEdge(int i) {
  uint8_t level = digitalRead(ENC_PIN[i]);
  if (level == encLevel[i]) return;          // glitch shorter than the ISR latency
  uint32_t now = micros();
  if (now - encLastUs[i] < ENC_MIN_EDGE_US) return;
  encLastUs[i] = now;
  encLevel[i] = level;
  encCount[i] += sideDir[ENC_SIDE[i]];
}
void IRAM_ATTR encIsr0() { encEdge(0); }
void IRAM_ATTR encIsr1() { encEdge(1); }
void IRAM_ATTR encIsr2() { encEdge(2); }
void IRAM_ATTR encIsr3() { encEdge(3); }

void setupEncoders() {
  void (*isr[4])() = {encIsr0, encIsr1, encIsr2, encIsr3};
  for (int i = 0; i < 4; i++) {
    pinMode(ENC_PIN[i], INPUT_PULLUP);
    encLevel[i] = digitalRead(ENC_PIN[i]);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN[i]), isr[i], CHANGE);
  }
}

void printEncoders() {
  int32_t c[4];
  noInterrupts();
  for (int i = 0; i < 4; i++) c[i] = encCount[i];
  interrupts();
  Serial.printf("E %lu %ld %ld %ld %ld\n", millis(), (long)c[0], (long)c[1], (long)c[2], (long)c[3]);
}

void writeSide(int rpwmPin, int lpwmPin, int speed) {
  if (speed > 0) {
    ledcWrite(lpwmPin, 0);
    ledcWrite(rpwmPin, speed);
  } else if (speed < 0) {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, -speed);
  } else {
    ledcWrite(rpwmPin, 0);   // both low = brake while EN is high; coasting is done with EN (see updateEnable)
    ledcWrite(lpwmPin, 0);
  }
}

// COAST at zero (team, 26 Oct): both BTS7960 boards share EN_PIN, so when BOTH sides are at 0 the drivers are
// disabled (outputs float, motors spin freely) instead of shorting the motors to ground (brake). When either side
// is driving, EN is high (a side at 0 then brakes - one EN wire cannot coast one side alone).
void updateEnable() {
  digitalWrite(EN_PIN, (currentL != 0 || currentR != 0) ? HIGH : LOW);
}

void setServos(int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulseUs = map(angle, 0, 180, 500, 2500);
  uint32_t duty = (uint32_t)((uint64_t)pulseUs * ((1UL << SERVO_RES) - 1) / 20000);
  ledcWrite(SERVO1, duty);
  ledcWrite(SERVO2, duty);
}

int stepToward(int current, int target) {
  if (current < target) return min(current + RAMP_STEP, target);
  if (current > target) return max(current - RAMP_STEP, target);
  return current;
}

void printHelp() {
  Serial.println(FW_VERSION);
  Serial.println("Commands:");
  Serial.println("  m <left> <right> [ms]  speeds -255..255, run time in ms (default 1000, 0 = until stopped)");
  Serial.println("  s <angle>              both servos to 0-180 degrees");
  Serial.println("  x                      stop motors");
  Serial.println("  e <hz> | e r           encoder stream rate (0 = off) | reset counts");
  Serial.println("  h                      this help");
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);

  if (c == 'm') {
    int l = 0, r = 0;
    long ms = DEFAULT_RUN_MS;
    if (sscanf(cmd.c_str(), "m %d %d %ld", &l, &r, &ms) < 2) {
      Serial.println("Usage: m <left> <right> [ms]");
      return;
    }
    targetL = constrain(l, -MAX_SPEED, MAX_SPEED);
    targetR = constrain(r, -MAX_SPEED, MAX_SPEED);
    if (ms > 0) {
      timedRun = true;
      stopAt = millis() + ms;
      Serial.printf("Left %d, right %d for %ld ms\n", targetL, targetR, ms);
    } else {
      timedRun = false;
      Serial.printf("Left %d, right %d until stopped\n", targetL, targetR);
    }
  } else if (c == 's') {
    int angle;
    if (sscanf(cmd.c_str(), "s %d", &angle) != 1) {
      Serial.println("Usage: s <angle>");
      return;
    }
    setServos(angle);
    Serial.printf("Servos to %d degrees\n", constrain(angle, 0, 180));
  } else if (c == 'x') {
    targetL = targetR = 0;
    timedRun = false;
    Serial.println("Stop");
  } else if (c == 'e') {
    int hz;
    if (cmd.indexOf('r') > 0) {
      noInterrupts();
      for (int i = 0; i < 4; i++) encCount[i] = 0;
      interrupts();
      Serial.println("Encoders reset");
    } else if (sscanf(cmd.c_str(), "e %d", &hz) == 1) {
      hz = constrain(hz, 0, 100);
      encPeriodMs = hz > 0 ? 1000 / hz : 0;
      Serial.printf("Encoder stream %d Hz\n", hz);
    } else {
      Serial.println("Usage: e <hz> | e r");
    }
  } else if (c == 'h') {
    printHelp();
  } else {
    Serial.println("Unknown command, type h for help");
  }
}

void setup() {
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);   // drivers off during setup

  Serial.begin(115200);
  delay(200);

  // Fixed channels: 0-3 share the motor timers, 4-5 share the servo timer
  bool ok = true;
  ok &= ledcAttachChannel(L_RPWM, MOTOR_FREQ, MOTOR_RES, 0);
  ok &= ledcAttachChannel(L_LPWM, MOTOR_FREQ, MOTOR_RES, 1);
  ok &= ledcAttachChannel(R_RPWM, MOTOR_FREQ, MOTOR_RES, 2);
  ok &= ledcAttachChannel(R_LPWM, MOTOR_FREQ, MOTOR_RES, 3);
  ok &= ledcAttachChannel(SERVO1, SERVO_FREQ, SERVO_RES, 4);
  ok &= ledcAttachChannel(SERVO2, SERVO_FREQ, SERVO_RES, 5);
  if (!ok) {
    Serial.println("PWM setup failed: check that the ESP32 core is version 3.x");
    while (true) delay(1000);   // stay stopped, drivers disabled
  }

  writeSide(L_RPWM, L_LPWM, 0);
  writeSide(R_RPWM, R_LPWM, 0);
  setServos(90);
  setupEncoders();

  delay(500);
  updateEnable();              // both sides at 0: drivers stay disabled (coast) until a motor command arrives
  Serial.println("Motor test ready");
  printHelp();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      handleCommand(inputLine);
      inputLine = "";
    } else if (inputLine.length() < 64) {
      inputLine += ch;
    }
  }

  if (timedRun && (long)(millis() - stopAt) >= 0) {
    targetL = targetR = 0;
    timedRun = false;
    Serial.println("Timed run finished");
  }

  if (millis() - lastRamp >= RAMP_INTERVAL_MS) {
    lastRamp = millis();
    currentL = stepToward(currentL, targetL);
    currentR = stepToward(currentR, targetR);
    writeSide(L_RPWM, L_LPWM, LEFT_INVERT ? -currentL : currentL);
    writeSide(R_RPWM, R_LPWM, RIGHT_INVERT ? -currentR : currentR);
    updateEnable();
    // Direction for the encoder counts: follow the applied speed; at 0 keep the last
    // direction (a coasting robot keeps rolling the way it was going).
    if (currentL != 0) sideDir[0] = currentL > 0 ? 1 : -1;
    if (currentR != 0) sideDir[1] = currentR > 0 ? 1 : -1;
  }

  if (encPeriodMs > 0 && millis() - lastEncPrint >= encPeriodMs) {
    lastEncPrint = millis();
    printEncoders();
  }
}
