/*
 * ESP32 motor controller for a Raspberry Pi 4 (USB serial)
 * ---------------------------------------------------------
 * - 2x BTS7960 (IBT-2) drivers: one for the left side (2 motors in parallel),
 *   one for the right side (2 motors in parallel).
 * - 4x 6 V 210 RPM motors with quadrature encoders.
 * - 2S LiPo (8.4 V full). PWM is CAPPED so the average motor voltage never
 *   exceeds 6 V:   maxDuty = MOTOR_MAX_VOLTS / batteryVolts
 *   (BTS7960 voltage drop is tiny, ~16 mOhm, so it is ignored by default.)
 *
 * Wiring
 *   Left  BTS7960:  RPWM -> 27   LPWM -> 26
 *   Right BTS7960:  RPWM -> 25   LPWM -> 33
 *   Both drivers:   R_EN + L_EN -> 32   (add a 10k resistor 32 -> GND so the
 *                                        drivers stay off while the ESP32 boots)
 *   Driver VCC (logic) -> ESP32 3.3V, driver GND -> ESP32 GND
 *   Driver B+ / B-     -> 2S LiPo  (motor power MUST come from the battery)
 *
 *   Encoders (VCC -> 3.3V, GND -> GND):
 *     FL: A=13 B=14    RL: A=16 B=17    FR: A=18 B=19    RR: A=21 B=22
 *   Optional battery sense: GPIO34 <- BATT+ --[100k]--+--[47k]-- GND
 *
 * ---------------- Serial protocol (115200 baud, lines end with '\n') ----------------
 * Pi -> ESP32
 *   M,<left>,<right>   PWM command, each -255..255. 255 = 6 V (the ceiling), - = reverse.
 *                      Send at least every CMD_TIMEOUT_MS or the motors stop.
 *   S                  stop (brake)
 *   R                  reset encoder counts to 0
 *   I                  print config info (as a '#' line)
 *   V,<volts>          battery voltage measured at the start of the run (6.4..8.6): ceiling = 6 V / volts,
 *                      so 255 really is 6 V on a partly discharged pack. Not set / after a reset: assumes
 *                      8.4 V (safe). The Pi bridge resends it every 2 s (parameter battery_voltage).
 *
 * ESP32 -> Pi   (every TELEMETRY_MS)
 *   E,<ms>,<cFL>,<cRL>,<cFR>,<cRR>,<rpmFL>,<rpmRL>,<rpmFR>,<rpmRR>,<battV>,<flags>
 *     c*    = cumulative encoder counts (signed, + = forward)
 *     rpm*  = output-shaft RPM (signed)
 *     flags = bit0 low battery, bit1 command timeout (stopped), bit2 drivers disabled
 *   Lines starting with '#' are human-readable info: the Pi should ignore them.
 */

#include <Arduino.h>

// ======================= USER SETTINGS =======================
// ---- Voltage ceiling ----
const float MOTOR_MAX_VOLTS   = 6.0;   // never exceed this average motor voltage
const float BATT_FULL_VOLTS   = 8.4;   // 2S LiPo fully charged (worst case)
const float DRIVER_DROP_VOLTS = 0.0;   // BTS7960 drop is negligible

const bool  USE_BATTERY_SENSE = false; // true only if the divider on GPIO34 is wired
const int   BATT_PIN          = 34;
const float DIVIDER_RATIO     = (100.0 + 47.0) / 47.0;  // (R1 + R2) / R2
const float LOW_BATT_CUTOFF   = 6.6;   // 3.3 V/cell (needs USE_BATTERY_SENSE)

// ---- PWM ----
const uint32_t PWM_FREQ = 20000;       // 20 kHz: silent, within BTS7960's 25 kHz limit
const uint8_t  PWM_RES  = 10;          // duty 0..1023
const uint32_t PWM_MAX  = (1UL << PWM_RES) - 1;
const int      CMD_MAX  = 255;         // command scale from the Pi
const int      RAMP_STEP = 13;         // max command change per 10 ms (0->255 in ~0.2 s)
                                       // protects the LiPo from current spikes; 0 = off

// ---- Safety ----
const uint32_t CMD_TIMEOUT_MS = 500;   // stop if the Pi goes quiet this long

// ---- Encoders ----
// Counts per OUTPUT-shaft revolution. Both edges of channel A are counted, so:
// COUNTS_PER_REV = encoder PPR (motor shaft) x gear ratio x 2.
// To measure: send R, turn one wheel exactly 1 turn by hand, read its count.
const float COUNTS_PER_REV = 11.0 * 34.0 * 2.0;

// ---- Telemetry ----
const uint32_t SERIAL_BAUD  = 115200;
const uint32_t TELEMETRY_MS = 50;      // 20 Hz
// =============================================================

const int EN_PIN = 32;

struct Side {
  uint8_t rpwm, lpwm;       // RPWM = forward, LPWM = reverse
  uint8_t chR, chL;         // LEDC channels (only used on core 2.x)
  bool reversed;            // set true if this side drives backward on positive commands
  int target;               // -255..255 from the Pi
  int current;              // ramped value actually applied
};

Side sides[2] = {
  {27, 26, 0, 1, true, 0, 0},  // LEFT
  {25, 33, 2, 3, true, 0, 0},  // RIGHT
};

struct Encoder {
  uint8_t a, b;
  bool reversed;            // set true if counts go negative while driving forward
  volatile int32_t count;
  int32_t lastCount;
  float rpm;
};

Encoder encoders[4] = {
  {13, 14, true, 0, 0, 0},  // FL
  {16, 17, true, 0, 0, 0},  // RL
  {18, 19, false, 0, 0, 0},  // FR
  {21, 22, false, 0, 0, 0},  // RR
};

float    batteryVolts = BATT_FULL_VOLTS;
// Battery voltage told by the Pi with "V,<volts>" (measured at the start of a run). Without it the firmware
// assumes a full pack (BATT_FULL_VOLTS) = the smallest, safe ceiling. Limited to SET_VOLTS_MIN..MAX so a typo
// cannot push the motors far above MOTOR_MAX_VOLTS. Lost on reset (back to the safe assumption).
const float SET_VOLTS_MIN = 6.4;       // 2S pack, 3.2 V/cell
const float SET_VOLTS_MAX = 8.6;
float    setVolts     = 0.0;    // 0 = not set by the Pi
uint32_t maxDuty      = 0;      // THE PWM CEILING
bool     lowBattery   = false;
bool     timedOut     = true;   // stay stopped until the first command arrives
uint32_t lastCmdMs    = 0;

// ---------------- PWM helpers (ESP32 Arduino core 2.x and 3.x) ----------------
void pwmAttach(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcAttach(pin, PWM_FREQ, PWM_RES);
#else
  ledcSetup(channel, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, channel);
#endif
}

void pwmWrite(uint8_t pin, uint8_t channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

// ---------------- Encoders ----------------
void IRAM_ATTR encoderISR(void *arg) {
  Encoder *e = (Encoder *)arg;
  int a = digitalRead(e->a);
  int b = digitalRead(e->b);
  e->count += (a == b) ? 1 : -1;
}

int32_t readCount(uint8_t i) {
  int32_t c = encoders[i].count;          // 32-bit read is atomic on ESP32
  return encoders[i].reversed ? -c : c;
}

// ---------------- Battery + ceiling ----------------
float readBatteryRaw() {
  return analogReadMilliVolts(BATT_PIN) / 1000.0 * DIVIDER_RATIO;
}

void updateBattery() {
  if (USE_BATTERY_SENSE) {
    batteryVolts = batteryVolts * 0.95 + readBatteryRaw() * 0.05;   // ~2 s smoothing
    if (!lowBattery && batteryVolts < LOW_BATT_CUTOFF) {
      lowBattery = true;
      Serial.printf("# LOW BATTERY %.2f V - motors disabled until reset\n", batteryVolts);
    }
  } else {
    batteryVolts = setVolts > 0.0 ? setVolts : BATT_FULL_VOLTS;   // told by the Pi, else assume full
  }

  float effective = batteryVolts - DRIVER_DROP_VOLTS;
  if (effective <= 0.5) { maxDuty = 0; return; }
  float ratio = MOTOR_MAX_VOLTS / effective;
  if (ratio > 1.0) ratio = 1.0;
  maxDuty = (uint32_t)(ratio * PWM_MAX);   // full battery: 6.0/8.4 = 71% -> 730
}

// ---------------- Motor output (the ONLY place PWM is written) ----------------
void applySide(uint8_t i, int cmd) {
  Side &s = sides[i];
  if (s.reversed) cmd = -cmd;
  cmd = constrain(cmd, -CMD_MAX, CMD_MAX);

  uint32_t duty = (uint32_t)abs(cmd) * maxDuty / CMD_MAX;
  if (duty > maxDuty) duty = maxDuty;      // hard 6 V ceiling

  if (cmd > 0) {        // forward
    pwmWrite(s.lpwm, s.chL, 0);
    pwmWrite(s.rpwm, s.chR, duty);
  } else if (cmd < 0) { // reverse
    pwmWrite(s.rpwm, s.chR, 0);
    pwmWrite(s.lpwm, s.chL, duty);
  } else {              // both low = brake
    pwmWrite(s.rpwm, s.chR, 0);
    pwmWrite(s.lpwm, s.chL, 0);
  }
}

void updateMotors() {
  bool allowed = !lowBattery && !timedOut;
  digitalWrite(EN_PIN, lowBattery ? LOW : HIGH);   // low battery: drivers fully off

  for (uint8_t i = 0; i < 2; i++) {
    Side &s = sides[i];
    int goal = allowed ? s.target : 0;
    if (RAMP_STEP <= 0) {
      s.current = goal;
    } else if (s.current < goal) {
      s.current = min(s.current + RAMP_STEP, goal);
    } else if (s.current > goal) {
      s.current = max(s.current - RAMP_STEP, goal);
    }
    applySide(i, s.current);
  }
}

// ---------------- Serial commands from the Pi ----------------
void printInfo() {
  Serial.printf("# ceiling duty %lu/%lu (%.1f%%), battery %.2f V -> max %.2f V at motors, "
                "counts/rev %.0f, timeout %lu ms\n",
                (unsigned long)maxDuty, (unsigned long)PWM_MAX, 100.0 * maxDuty / PWM_MAX,
                batteryVolts, (batteryVolts - DRIVER_DROP_VOLTS) * maxDuty / PWM_MAX,
                COUNTS_PER_REV, (unsigned long)CMD_TIMEOUT_MS);
}

void handleLine(char *line) {
  for (char *p = line; *p; p++) if (*p == ',') *p = ' ';   // accept "M,1,2" or "M 1 2"

  switch (line[0]) {
    case 'M': case 'm': {
      int l, r;
      if (sscanf(line + 1, "%d %d", &l, &r) == 2) {
        sides[0].target = constrain(l, -CMD_MAX, CMD_MAX);
        sides[1].target = constrain(r, -CMD_MAX, CMD_MAX);
        lastCmdMs = millis();
        timedOut = false;
      }
      break;
    }
    case 'S': case 's':
      sides[0].target = sides[1].target = 0;
      lastCmdMs = millis();
      break;
    case 'R': case 'r':
      for (uint8_t i = 0; i < 4; i++) { encoders[i].count = 0; encoders[i].lastCount = 0; }
      break;
    case 'V': case 'v': {                  // V,<volts>: battery voltage measured at the start of the run
      float v;
      if (sscanf(line + 1, "%f", &v) == 1) {
        float clamped = constrain(v, SET_VOLTS_MIN, SET_VOLTS_MAX);
        bool changed = fabs(clamped - setVolts) > 0.005;
        setVolts = clamped;
        updateBattery();                   // new ceiling right away
        if (changed) {
          if (clamped != v) Serial.printf("# V %.2f out of range, using %.2f\n", v, clamped);
          printInfo();
        }
      }
      break;
    }
    case 'I': case 'i':
      printInfo();
      break;
  }
}

void readSerial() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;   // line too long: drop it
    }
  }
}

// ---------------- Telemetry to the Pi ----------------
void sendTelemetry(float dt) {
  int32_t c[4];
  for (uint8_t i = 0; i < 4; i++) {
    c[i] = readCount(i);
    float rpm = (c[i] - encoders[i].lastCount) / COUNTS_PER_REV / dt * 60.0f;
    encoders[i].lastCount = c[i];
    encoders[i].rpm = rpm;
  }
  uint8_t flags = (lowBattery ? 1 : 0) | (timedOut ? 2 : 0) | (lowBattery ? 4 : 0);
  Serial.printf("E,%lu,%ld,%ld,%ld,%ld,%.1f,%.1f,%.1f,%.1f,%.2f,%u\n",
                (unsigned long)millis(),
                (long)c[0], (long)c[1], (long)c[2], (long)c[3],
                encoders[0].rpm, encoders[1].rpm, encoders[2].rpm, encoders[3].rpm,
                batteryVolts, flags);
}

// ---------------- Setup / loop ----------------
void setup() {
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);               // drivers off during setup

  for (uint8_t i = 0; i < 2; i++) {
    pwmAttach(sides[i].rpwm, sides[i].chR);
    pwmAttach(sides[i].lpwm, sides[i].chL);
    pwmWrite(sides[i].rpwm, sides[i].chR, 0);
    pwmWrite(sides[i].lpwm, sides[i].chL, 0);
  }

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(encoders[i].a, INPUT_PULLUP);
    pinMode(encoders[i].b, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(encoders[i].a), encoderISR, &encoders[i], CHANGE);
  }

  Serial.begin(SERIAL_BAUD);

  if (USE_BATTERY_SENSE) {
    analogSetPinAttenuation(BATT_PIN, ADC_11db);
    float sum = 0;
    for (int i = 0; i < 32; i++) { sum += readBatteryRaw(); delay(2); }
    batteryVolts = sum / 32;
  }
  updateBattery();

  Serial.println("# ESP32 motor controller ready");
  printInfo();
}

void loop() {
  static uint32_t lastMotor = 0, lastBatt = 0, lastTele = 0;
  readSerial();
  // Read the clock AFTER readSerial(): a command handled there sets lastCmdMs = millis(), which could be later
  // than a time taken before it -> "now - lastCmdMs" wrapped around (unsigned) to ~4e9 and the motors were cut
  // as "command timeout" every 50-200 ms while driving (found 24 Nov, power-0 test: 40 false timeouts in 10 s).
  uint32_t now = millis();

  if (!timedOut && now - lastCmdMs > CMD_TIMEOUT_MS) {
    timedOut = true;                        // Pi went quiet: stop
    Serial.println("# command timeout - stopping");
  }

  if (now - lastBatt >= 100) { lastBatt = now; updateBattery(); }
  if (now - lastMotor >= 10) { lastMotor = now; updateMotors(); }
  if (now - lastTele >= TELEMETRY_MS) {
    float dt = (now - lastTele) / 1000.0f;
    lastTele = now;
    sendTelemetry(dt);
  }
}
