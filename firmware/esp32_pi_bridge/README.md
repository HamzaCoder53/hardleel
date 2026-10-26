# ESP32 Motor & Servo Bridge Firmware (`esp32_pi_bridge`)

Firmware for the ESP32 (classic ESP32-WROOM-32 on 38-pin dev board) driving:
* 2 × BTS7960 (IBT-2) high-power H-bridge modules for 4 × Johnson 37GB 12V 1000rpm gear motors (skid-steer, left pair in parallel, right pair in parallel).
* 2 × Drop-bin servos (50 Hz, 500-2500 µs pulse width).

---

## Hardware Connections

| ESP32 Pin | Connects to | Function |
|:---|:---|:---|
| **GPIO 25** | Left module RPWM | Forward PWM (20 kHz, 8-bit) |
| **GPIO 33** | Left module LPWM | Reverse PWM (20 kHz, 8-bit) |
| **GPIO 26** | Right module RPWM | Forward PWM (20 kHz, 8-bit) |
| **GPIO 27** | Right module LPWM | Reverse PWM (20 kHz, 8-bit) |
| **GPIO 32** | R_EN + L_EN on both modules | Driver Enable (Active HIGH, 10 kΩ pull-down to GND) |
| **GPIO 13** | Servo 1 Signal | Drop bin servo 1 (50 Hz, 16-bit LEDC channel 4) |
| **GPIO 4** | Servo 2 Signal | Drop bin servo 2 (50 Hz, 16-bit LEDC channel 5) |
| **3V3** | VCC on both BTS7960 modules | Logic supply for driver chips |
| **GND** | GND on both BTS7960 modules | Common ground |
| **USB** | Raspberry Pi 4 USB Port | Serial communication & ESP32 power |

> **Note on Motor Power:** Motor power is supplied by a 3S LiPo (11.1 V nominal) through a 10 A fuse connected to the BTS7960 `B+` and `B-` terminals. Servos are powered from a separate 5 V power rail.

---

## Serial Protocol (115200 Baud, 8N1, `\n` Terminated)

### Host -> ESP32 Commands
* `M <left> <right>`: Set side speeds (`-255` to `255`, positive = forward).
* `S <angle>`: Move both drop servos to angle (`0` to `180` degrees, power-up default = `90`).
* `X`: Emergency stop — cut motor outputs immediately.
* `P`: Ping — tests serial link responsiveness.

### ESP32 -> Host Responses
* `READY`: Sent once when ESP32 finishes booting and outputs are enabled.
* `PONG`: Reply to `P` ping command.
* `ERR <reason>`: Sent when syntax or parameter ranges are invalid.
* `WATCHDOG STOP`: Sent if motors were active and no `M` command was received within 500 ms.

---

## Flashing Instructions
1. Open the sketch in **Arduino IDE** (requires ESP32 board package version 3.x+).
2. Select Board: **ESP32 Dev Module**.
3. Port: Select the corresponding COM / ttyUSB port.
4. Click **Upload**.
