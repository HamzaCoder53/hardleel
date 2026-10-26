# motor_code.ino - the ESP32 sketch on the robot (team, 26 Oct 2026)

ESP32 + 2x BTS7960 (IBT-2) + 2 drop servos. Pins: left RPWM 33 / LPWM 25, right RPWM 26 / LPWM 27, EN 32 (all
four R_EN/L_EN, 10k pull-down), servos GPIO 13 + 4. 115200 baud, newline.

Commands: `m <left> <right> [ms]` (-255..255, runs ms, default 1000, 0 = until stopped), `s <angle>`, `x`, `h`.
The sketch ramps 0 -> 255 in ~0.25 s and has NO separate watchdog. The Pi's motor_bridge therefore uses
`esp32_protocol: motor_code` and sends every drive command as a timed run of `esp32_hold_ms` (400 ms),
re-sent every 100 ms while driving: if the Pi or the USB link stops, the run ends and the motors stop.
Never send `m ... 0` from software (no timeout).

`../esp32_pi_bridge/` is an alternative sketch (uppercase M/S/X/P + 500 ms watchdog) - set
`esp32_protocol: bridge` if that one is flashed instead. Note its left pins are swapped (RPWM 25 / LPWM 33).
