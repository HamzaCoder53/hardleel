// esp32_pi_bridge.ino
// ESP32 + 2x BTS7960 (IBT-2) + 2 servos motor bridge for Hardleel Robot.
// Requires ESP32 Arduino Core 3.x (Boards Manager: "esp32" by Espressif).
//
// Protocol (115200 baud, ASCII, newline terminated):
//   M <left> <right>   side speeds (-255..255, positive = forward)
//   S <angle>          both drop servos to angle (0..180)
//   X                  stop motors immediately
//   P                  ping (replies PONG)
//
// Responses:
//   READY              sent after power-up initialization
//   PONG               reply to 'P'
//   ERR <reason>       syntax or parameter error
//   WATCHDOG STOP      sent when 500ms watchdog cuts motor power

#include <Arduino.h>

// ---- Pin Configuration ----
const int L_RPWM = 25; // Left BTS7960 RPWM
const int L_LPWM = 33; // Left BTS7960 LPWM
const int R_RPWM = 26; // Right BTS7960 RPWM
const int R_LPWM = 27; // Right BTS7960 LPWM
const int EN_PIN = 32; // Both modules R_EN + L_EN (10k pull-down to GND)
const int SERVO1 = 13; // Drop servo 1 signal
const int SERVO2 = 4;  // Drop servo 2 signal

// ---- Settings ----
const bool LEFT_INVERT  = false; // Set true if physical left side runs backwards
const bool RIGHT_INVERT = false; // Set true if physical right side runs backwards
const int  MAX_SPEED    = 255;   // Full scale 8-bit PWM limit
const int  RAMP_STEP    = 5;     // Duty change per ramp tick
const unsigned long RAMP_INTERVAL_MS = 5; // ~0.25 s for 0 to 255
const unsigned long WATCHDOG_MS      = 500; // 500 ms command timeout

const uint32_t MOTOR_FREQ = 20000; // 20 kHz ultrasonic motor PWM
const uint8_t  MOTOR_RES  = 8;     // 8-bit duty (0-255)
const uint32_t SERVO_FREQ = 50;    // 50 Hz standard servo frame (20 ms)
const uint8_t  SERVO_RES  = 16;   // 16-bit resolution for precise servo pulses

// ---- Runtime State ----
int targetL = 0, targetR = 0;
int currentL = 0, currentR = 0;
unsigned long lastMTime = 0;
unsigned long lastRamp = 0;
bool watchdogTriggered = false;
String inputLine;

void writeSide(int rpwmPin, int lpwmPin, int speed) {
  if (speed > 0) {
    ledcWrite(lpwmPin, 0);
    ledcWrite(rpwmPin, speed);
  } else if (speed < 0) {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, -speed);
  } else {
    ledcWrite(rpwmPin, 0); // Both low = active dynamic brake on BTS7960
    ledcWrite(lpwmPin, 0);
  }
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

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);

  if (c == 'M' || c == 'm') {
    int l = 0, r = 0;
    if (sscanf(cmd.c_str(), "%*c %d %d", &l, &r) < 2) {
      Serial.println("ERR bad_m_syntax");
      return;
    }
    targetL = constrain(l, -MAX_SPEED, MAX_SPEED);
    targetR = constrain(r, -MAX_SPEED, MAX_SPEED);
    lastMTime = millis();
    watchdogTriggered = false;
  } else if (c == 'S' || c == 's') {
    int angle = 0;
    if (sscanf(cmd.c_str(), "%*c %d", &angle) < 1 || angle < 0 || angle > 180) {
      Serial.println("ERR bad_s_syntax");
      return;
    }
    setServos(angle);
  } else if (c == 'X' || c == 'x') {
    targetL = 0;
    targetR = 0;
    currentL = 0;
    currentR = 0;
    writeSide(L_RPWM, L_LPWM, 0);
    writeSide(R_RPWM, R_LPWM, 0);
    lastMTime = millis();
    watchdogTriggered = false;
  } else if (c == 'P' || c == 'p') {
    Serial.println("PONG");
  } else {
    Serial.println("ERR unknown_command");
  }
}

void setup() {
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Drivers disabled during boot

  Serial.begin(115200);
  delay(100);

  // Attach LEDC channels for motors (0-3) and servos (4-5)
  bool ok = true;
  ok &= ledcAttachChannel(L_RPWM, MOTOR_FREQ, MOTOR_RES, 0);
  ok &= ledcAttachChannel(L_LPWM, MOTOR_FREQ, MOTOR_RES, 1);
  ok &= ledcAttachChannel(R_RPWM, MOTOR_FREQ, MOTOR_RES, 2);
  ok &= ledcAttachChannel(R_LPWM, MOTOR_FREQ, MOTOR_RES, 3);
  ok &= ledcAttachChannel(SERVO1, SERVO_FREQ, SERVO_RES, 4);
  ok &= ledcAttachChannel(SERVO2, SERVO_FREQ, SERVO_RES, 5);

  if (!ok) {
    Serial.println("ERR ledc_init_failed");
    while (true) delay(1000);
  }

  writeSide(L_RPWM, L_LPWM, 0);
  writeSide(R_RPWM, R_LPWM, 0);
  setServos(90); // Default drop bin position at power-up: 90 degrees

  delay(200);
  digitalWrite(EN_PIN, HIGH); // Enable BTS7960 drivers

  lastMTime = millis();
  lastRamp = millis();

  Serial.println("READY");
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (inputLine.length() > 0) {
        handleCommand(inputLine);
        inputLine = "";
      }
    } else if (inputLine.length() < 64) {
      inputLine += ch;
    }
  }

  // 500 ms safety watchdog: if motors were commanded and no M command arrived, cut power
  if (!watchdogTriggered && (targetL != 0 || targetR != 0 || currentL != 0 || currentR != 0)) {
    if (millis() - lastMTime >= WATCHDOG_MS) {
      targetL = 0;
      targetR = 0;
      currentL = 0;
      currentR = 0;
      writeSide(L_RPWM, L_LPWM, 0);
      writeSide(R_RPWM, R_LPWM, 0);
      watchdogTriggered = true;
      Serial.println("WATCHDOG STOP");
    }
  }

  // Slew rate ramping
  if (millis() - lastRamp >= RAMP_INTERVAL_MS) {
    lastRamp = millis();
    currentL = stepToward(currentL, targetL);
    currentR = stepToward(currentR, targetR);
    writeSide(L_RPWM, L_LPWM, LEFT_INVERT ? -currentL : currentL);
    writeSide(R_RPWM, R_LPWM, RIGHT_INVERT ? -currentR : currentR);
  }
}
