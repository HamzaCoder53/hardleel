# DIY wheel encoders (4 wheels) - build, flash, calibrate

Why: odometry today is lidar (rf2o) + IMU only, and the PWM-70 stall floor means the robot sometimes
"drives" without moving. One cheap sensor per wheel gives real forward speed and lets the
software see a stall or a stuck wheel.

## 1. Pick a sensor (all four wheels the same)

| Option | Parts per wheel | Counts / turn | Good | Watch out |
|---|---|---|---|---|
| **A. Reflective (recommended)** | TCRT5000 module (LM393, 4-pin) + printed stripe disc | 2 x stripes (24 stripes = 48) | Works on any wheel, no shaft access, disc from `tools/encoder_disc.py` | Shade it from sunlight / room lights; 2-4 mm gap; tune the module's pot |
| B. Slotted opto | "LM393 speed sensor" (U-slot, FC-03 / HC-020K) + 20-slot disc | 40 | Very clean signal | Disc must sit on the shaft, inside the U slot |
| C. Hall | A3144 / KY-003 hall + 4-8 small magnets on the hub | 2 x magnets | Dust/light proof | Low resolution, A3144 needs 4.5 V+ supply and a 10 k pull-up to **3.3 V** |

Single channel = no direction from the sensor. The sketch signs each count with that side's
commanded direction, which is fine for a skid-steer robot that only drives where it is told.

## 2. Wiring (ESP32 running `firmware/motor_code/motor_code.ino`)

| Wheel | ESP32 GPIO | Sensor VCC | Sensor GND |
|---|---|---|---|
| Left-front | **18** | **3V3** | GND |
| Left-rear | **19** | 3V3 | GND |
| Right-front | **22** | 3V3 | GND |
| Right-rear | **23** | 3V3 | GND |

Use the module's **D0** (digital) pin, not A0.

- **Power the modules from 3.3 V, never 5 V.** Their output pull-up goes to VCC, and 5 V into an
  ESP32 pin will damage it. (Hall option C: sensor on 5 V, but pull-up resistor to 3.3 V only.)
- The BTS7960 boards switch 20 kHz PWM at several amps. Run the sensor wires **away from the motor
  wires**, twist each signal wire with its GND, keep the ESP32 GND common with the driver logic GND.
  If counts still appear while the wheels are held still, add a 100 nF capacitor from each signal to GND
  at the ESP32 end.
- Pins already in use (don't touch): 33/25/26/27 motors, 32 driver EN, 13/4 servos.

## 3. Mechanics (option A)

1. Measure the flat face inside each wheel hub (or a spot on the shaft collar): biggest disc that fits = `--outer`.
2. Print a 4-up sheet at **100 % / actual size**:
   `python3 tools/encoder_disc.py --outer 50 --inner 20 --hole 6 --stripes 24 --sheet -o discs.svg`
   Check with a ruler that the disc measures `--outer` mm. Laser print (matte black works best); glue on
   card, then on the hub, **centred** (an off-centre disc gives an uneven pulse train).
3. Mount the TCRT5000 on the chassis facing the stripe ring, 2-4 mm away, squarely. Turn the wheel by
   hand: the module's LED must blink on every stripe. Adjust the blue pot until it does so cleanly,
   then cover the sensor from ambient light (tape / a small shroud).

## 4. Flash and check the ESP32

```
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/motor_code
arduino-cli upload  --fqbn esp32:esp32:esp32 -p COMx firmware/motor_code
```
(Or the Arduino IDE, ESP32 core 3.x.) Serial monitor at 115200, Newline:
- You'll see `E <ms> <lf> <lr> <rf> <rr>` 20x per second. `e 0` stops it, `e 50` = 50 Hz, `e r` resets counts.
- Turn each wheel by hand one full turn: only its column should change, by `ticks_per_rev`
  (2 x stripes). The count goes UP or DOWN depending on the last drive direction (no direction sensing).
- `m 80 80 1000`: all four columns rise; `m -80 -80 1000`: all fall.
- Wheels still, motors running in the air on the other side? Columns must not creep (if they do: noise, see section 2).

Same commands as before (`m`, `x`, `s`, `h`) - the motor behaviour is unchanged.

## 5. Turn it on in ROS (Pi)

Rebuild `motor_teleop` (`MAKEFLAGS=-j1 colcon build --executor sequential --packages-select motor_teleop`), then
```
ros2 run motor_teleop esp32_teleop_bridge --ros-args -p encoders:=true \
    -p ticks_per_rev:=48.0 -p wheel_diameter:=0.065 -p track_width:=0.30
```
Topics: `/wheel/odom` (nav_msgs/Odometry, frame odom -> base_link, **no TF**) and
`/wheel/speeds` ([lf, lr, rf, rr] m/s). All encoder params are live-tunable (tuning GUI / `ros2 param set`).
A dead sensor (one wheel 0 while its partner moves) is logged and the other wheel of that side is used.

## 6. Calibrate

1. **Wheel diameter**: tape mark, drive 2.00 m straight slowly. `wheel_diameter_new = wheel_diameter * 2.00 / odom_x`.
2. **Track width** (skid steer slips, so it is bigger than the real 0.305 m gap): spin on the spot 5 full turns
   (count them / check the IMU). `track_width_new = track_width * encoder_yaw / true_yaw`. The value depends on the
   floor - do it on the competition surface.
3. Save in the tuning GUI.

## 7. Fuse into the EKF (only after calibration)

Add to `ekf_nav.yaml` (forward speed only; the IMU keeps yaw, rf2o keeps x/y):
```yaml
    odom1: /wheel/odom
    odom1_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, false,
                   false, false, false]
    odom1_queue_size: 10
    odom1_differential: false
```
Uses vx and vy (= 0, non-holonomic). Don't fuse wz from the wheels - skid steer slips in every turn.

## 8. Ideas for later
- Stall detection in `maze_explorer`: a commanded speed with `/wheel/speeds` ~0 for 0.5 s = stuck (better than the
  current 2 s pose check).
- Closed-loop speed (PI on `/wheel/speeds`) to get rid of the fixed PWM 70 floor and drive slower in the maze.
