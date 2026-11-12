# Checklist of [NEEDS HUMAN] Items (Ordered by Impact on Engineering Plan)

The following parameters cannot be confirmed solely through the remote OS, filesystem, or codebase audit and require human physical inspection or measurement.

---

### Priority 1: High Impact (Directly Blocks or Changes Control & SLAM Configuration)

#### 1. LiDAR Motor Connection & Undervoltage Cause (Affects Q6, Q7, Q16)
* **Impact:** Critical. The FTDI USB chip is recognized on USB, but the LiDAR scanner does not reply or spin, and the kernel reports repeated `Undervoltage detected!`. If the Pi browns out or the LiDAR power connector is loose, SLAM cannot function.
* **Procedure:**
  1. Inspect the RPLIDAR A1 motor wire/connector. Verify whether the motor lead is plugged into the FTDI breakout board or an external 5V rail.
  2. Inspect the USB-C power supply powering the Raspberry Pi 4. Measure voltage across 5V and GND GPIO test points (Pins 2/4 and Pin 6) using a multimeter. Ensure it remains $\ge 4.95\text{ V}$ under load.
  3. If powering from battery via a buck converter, adjust the potentiometer output to $5.10\text{--}5.20\text{ V}$ DC.

#### 2. Physical LiDAR Zero Mark vs. Robot Forward Direction (Q7, Q8)
* **Impact:** High. An inverted or offset yaw angle rotates all scan coordinates, breaking scan-matching against IMU yaw.
* **Procedure:**
  1. Look at the top of the RPLIDAR A1. Locate the physical zero-degree reference (the wire egress or notch).
  2. With the robot facing strictly forward ($+X$), verify whether the LiDAR 0° indicator points directly forward, backward, or sideways.
  3. Measure distance from the center of the wheel axle to the LiDAR optical center (confirm if $X=0.0\text{ m}$ or offset).

#### 3. Nano 33 BLE Physical Mounting Orientation & Chipset Revision (Q10, Q11)
* **Impact:** High. Determines whether the IMU axes match ROS REP-103 conventions ($+X$ forward, $+Y$ left, $+Z$ up).
* **Procedure:**
  1. Inspect the Arduino Nano 33 BLE PCB. Check if the components face upward (components up = `roll=0`) or downward (`roll=3.14159`).
  2. Look at the small IC near the center with a magnifying glass: check if the marking reads `LSM9DS1` (older Sense revision) or `LSM6DS3TR-C` (standard BLE revision).
  3. Place the robot level on the floor. Verify `/imu/data_raw` reports $+9.81\text{ m/s}^2$ on $a_z$. Rotate the chassis counter-clockwise (CCW) by hand: verify $g_z$ reads positive values.

#### 4. Motor Hardware, Gear Ratio, and Wheel Dimensions (Q13, Q14)
* **Impact:** High. Required to tune Nav2 velocity limits, acceleration bounds, and diff-drive kinematic models.
* **Procedure:**
  1. Use calipers to measure outer wheel diameter across the rubber tire treads (confirm if exactly $65\text{ mm}$, $70\text{ mm}$, etc.).
  2. Measure track width (center-to-center distance between left and right contact patches).
  3. Read motor gearbox label (e.g., TT yellow motor 1:48, GA25 12V 1:30, or N20).

---

### Priority 2: Medium Impact (Calibrates Safety Buffers & Ramp Physics)

#### 5. Ramp Surface Geometry & Positions (Q32)
* **Impact:** Medium. Sets the exact pitch threshold for tilt gating and grade-assist power.
* **Procedure:**
  1. Using a tape measure, measure the length of the ramp incline and vertical rise to the top platform for both ramps (sand and gravel).
  2. Place a smartphone with an inclinometer app flat on the ramp surface to record the exact angle $\theta_{\text{ramp}}$.
  3. Measure the distance along the arena perimeter wall to the ramp start.

#### 6. Battery Chemistry, Cell Count, and Capacity (Q16)
* **Impact:** Medium. Critical for voltage monitoring and motor duty consistency.
* **Procedure:**
  1. Read the battery pack label: confirm chemistry (e.g., 2S / 3S Li-ion 18650, 3S LiPo), nominal voltage ($7.4\text{ V}$ vs $11.1\text{ V}$), and capacity ($\text{mAh}$).
  2. Check if a voltage divider or telemetry wire connects to an analog pin on the Nano or Pico.

#### 7. Contact Bumpers / Physical E-Stop (Q18)
* **Impact:** Medium. Safety requirement.
* **Procedure:**
  1. Inspect the perimeter of the chassis for microswitches, limit switches, or optical bumpers.
  2. Check if there is an emergency power kill switch / latching button wired between the battery and motor drivers.

---

### Priority 3: Mission & Competition Logistics

#### 8. Dynamic Obstacle Timing & Cylinder Specs (Q33, Q34)
* **Impact:** Operational planning for competition runs.
* **Procedure:**
  1. Measure cylinder height and diameter using a ruler (verify if height $> 150\text{ mm}$ to ensure LiDAR beam plane intersects them).
  2. Use a stopwatch to time 10 cycles of the swinging gate: measure total period $T$ and the open dwell time $T_{\text{open}}$.

#### 9. Arena Surface & Stopping Deceleration (Q36)
* **Impact:** Operational tuning of stopping distance.
* **Procedure:**
  1. Note flat-floor material (smooth plywood, painted MDF, vinyl mat, carpet).
  2. Drive robot at $0.2\text{ m/s}$ on the surface, command instant stop, and measure skid distance with a ruler.
