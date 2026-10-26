# Retired: Raspberry Pi Pico (RP2040) Motor Firmware

> **STATUS: RETIRED (26 Oct 2026)**

This firmware was retired after hardware burnout on 26 Oct 2026 (TB6612 driver failure causing 12V backfeed to the Pico).
The robot's motor control has been replaced by an **ESP32** running `firmware/esp32_pi_bridge/esp32_pi_bridge.ino` driving two **BTS7960 (IBT-2)** high-power H-bridges.

Files retained for reference:
* `motor_servo.c`
* `motor_servo_orig.c`
* `CMakeLists.txt`
