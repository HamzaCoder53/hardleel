// motor_test.ino
// ESP32 + 2x BTS7960 (IBT-2) + 2 servos: serial test sketch
// Requires the ESP32 Arduino core 3.x (Boards Manager: "esp32" by Espressif)
//
// Commands (Serial Monitor at 115200 baud, line ending "Newline"):
//   m <left> <right> [ms]  speeds from -255 to 255; runs for ms milliseconds
//                          (default 1000), use 0 to run until stopped
//   s <angle>              move both servos to 0-180 degrees
//   x                      stop both motors
//   h                      show help

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

// ---- Settings ----
const bool LEFT_INVERT  = false;  // set true if the left side runs backwards
const bool RIGHT_INVERT = false;  // set true if the right side runs backwards
const int  MAX_SPEED    = 255;    // lower this to cap speed while testing
const int  RAMP_STEP    = 5;      // duty change per ramp tick
const unsigned long RAMP_INTERVAL_MS = 5;    // 0 to full speed in about 0.25 s
const unsigned long DEFAULT_RUN_MS   = 1000;

const char*    FW_VERSION = "motor_code coast-at-zero 2026-10-26";  // printed at boot and by 'h'

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
  }
}
