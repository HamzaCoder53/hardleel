// Nano 33 BLE (Rev1, LSM9DS1) -> USB serial for robot_slam/imu_serial_node  +  one servo (24 Nov).
// Libraries: ReefwingLSM9DS1 + Reefwing-imuTypes + Servo.   Board: Arduino Nano 33 BLE (mbed).
//
// Nano -> PC/Pi
//   I,ax,ay,az,gx,gy,gz   (g, deg/s)   ~100 Hz
//   T,<deg C>                          ~1 Hz   LSM9DS1 chip temperature (gyro temperature compensation)
//   S,<angle> | S,off                  reply to every servo command (the IMU node ignores S lines)
// PC/Pi -> Nano (lines end with '\n')
//   S,<angle>    move the servo to 0..180 degrees (attaches it if it was off)
//   S,off        stop driving the servo (it goes limp - saves power / stops buzzing)
//   S,?          report the current angle
//
// Servo wiring: signal -> D9, servo GND -> Nano GND (common ground), servo + -> a separate 5-6 V supply
// (BEC / buck converter from the battery). NOT the Nano's 3V3 pin: a servo pulls far more current than it can
// give. The Nano's 3.3 V signal drives normal hobby servos fine.
#include <ReefwingLSM9DS1.h>
#include <Servo.h>

const int SERVO_PIN = 9;
const int SERVO_START = 90;                   // angle at power-up

ReefwingLSM9DS1 imu;
Servo servo;
int servoAngle = -1;                          // -1 = off (not driven)
unsigned long lastPrint = 0;
unsigned long lastTemp = 0;
char cmd[24];
uint8_t cmdLen = 0;

void reportServo() {
  if (servoAngle < 0) Serial.println("S,off");
  else { Serial.print("S,"); Serial.println(servoAngle); }
}

void setServo(int angle) {
  angle = constrain(angle, 0, 180);
  if (!servo.attached()) servo.attach(SERVO_PIN);
  servo.write(angle);
  servoAngle = angle;
}

void handleCommand(char *line) {
  if (line[0] != 'S' && line[0] != 's') return;
  char *arg = line + 1;
  while (*arg == ',' || *arg == ' ') arg++;
  if (*arg == '?') {
    // report only
  } else if (strncmp(arg, "off", 3) == 0 || strncmp(arg, "OFF", 3) == 0) {
    servo.detach();
    servoAngle = -1;
  } else if (*arg >= '0' && *arg <= '9') {
    setServo(atoi(arg));
  } else {
    return;                                   // unknown: ignore
  }
  reportServo();
}

void readCommands() {                         // non-blocking: never delays the IMU stream
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) { cmd[cmdLen] = 0; handleCommand(cmd); cmdLen = 0; }
    } else if (cmdLen < sizeof(cmd) - 1) {
      cmd[cmdLen++] = c;
    } else {
      cmdLen = 0;                             // too long: drop it
    }
  }
}

void setup() {
  setServo(SERVO_START);                      // known position before the IMU calibration starts
  imu.begin();
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}      // don't hang when no PC is attached

  if (!imu.connected()) {
    while (true) { Serial.println("ERR,IMU not detected"); readCommands(); delay(1000); }
  }

  imu.start();
  imu.calibrateGyro();                        // keep the board still and flat at power-up
  imu.calibrateAccel();
  delay(20);
  imu.readGyro();                             // flush the first reading
  imu.readAccel();
}

void loop() {
  readCommands();
  imu.updateSensorData();

  if (millis() - lastPrint >= 10) {           // about 100 Hz
    lastPrint = millis();
    Serial.print("I,");
    Serial.print(imu.data.ax, 5); Serial.print(',');
    Serial.print(imu.data.ay, 5); Serial.print(',');
    Serial.print(imu.data.az, 5); Serial.print(',');
    Serial.print(imu.data.gx, 4); Serial.print(',');
    Serial.print(imu.data.gy, 4); Serial.print(',');
    Serial.println(imu.data.gz, 4);
  }

  if (millis() - lastTemp >= 1000) {          // 1 Hz is plenty: temperature changes slowly
    lastTemp = millis();
    float t = imu.readTemp();                 // OUT_TEMP: 16 LSB per deg C, 0 = 25 deg C
    Serial.print("T,");
    Serial.println(t, 2);                     // the ROS node ignores values outside -40..125 C
  }
}
