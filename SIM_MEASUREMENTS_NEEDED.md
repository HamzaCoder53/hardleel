# SIM_MEASUREMENTS_NEEDED

Every number below is a PLACEHOLDER in the simulation. Replace it in the named file when you have the real value.
Positions are in the course frame (start (0,0) facing east); the sim robot starts at world x = +0.06 m.

## Course / world (`ros2_ws/src/rdj2026_sim/config/course_features.yaml`)
| what | placeholder | where measured |
|---|---|---|
| wall height | 0.40 m in the generated world (0.45 in the yaml, not yet applied to the generator) | any wall |
| dirt ramp: height, length, slope, bump size/spacing | 0.07 m, 1.2 m, 5 bars of 12 mm, centre y -1.10 | the real ramp |
| gravel ramp: height, length, bumps | 0.08 m, 1.14 m, 4 bars, centre x -3.00 | the real ramp |
| gate position, width, opening time | x -3.02, y 0.07, 0.6 m wide, opens 2 s after the mission asks | the real gate and the rulebook's "standard waiting time" |
| gate decision threshold (mission) | open if median range ahead > 1.5 m (`competition_mission.yaml`) | real gate vs lane |
| dock E2 / E4 y position | -0.95 / -2.31 (E3 -1.50 is measured); panel 0.35 x 0.03 x 0.20 m | the real docks |
| screen: position, size, height, what images it shows | x 1.05, y 0, 0.40 x 0.30 m, centre z 0.22 m, photos from `test_photos/` | the real screen |
| bins (blue/white/black) positions and size | (-1.45,-1.75) (-1.20,-1.75) (-0.95,-1.75), 0.18 m cube | the real end room |
| finish position | (-1.2, -0.35) | the real end room |
| workers: size, paths, timings | disabled by default; 0.125 m radius cylinder, 2 positions, 20 s | the real workers |
| cylinders placed by judges | disabled by default; 0.055 m radius | the real cylinders |
| grass strip | off by default; 0.6 m wide, friction 1.8 | the real grass |
| back corridors / divider / end room geometry | from `course_full_approx` (+- 0.3 m) | measure the arena |

## Robot (`ros2_ws/src/rdj2026_sim/urdf/hardleel_sim.urdf`)
| what | placeholder | real value needed |
|---|---|---|
| mass | 1.8 kg total | weigh the robot |
| wheels | 4 x radius 0.035 m, track 0.245 m, wheelbase 0.20 m | measure |
| tyre friction | 1.2 along the roll direction, 0.7 sideways | test on the real floor |
| body 0.26 x 0.22 x 0.08 m | estimate | measure (must stay inside 30 x 30 x 30 cm) |
| camera mount (0.10, 0, 0.20) m, 66 deg horizontal FOV, 10 Hz | `robot_slam/stack_common.py` CAM_* is also nominal | measure; check standard vs wide lens |
| lidar height 0.15 m above base_link, 6.4 Hz, 1 cm noise | spec values | check on the robot |
| IMU gyro: noise 0.002 rad/s, residual bias 3.8e-4 rad/s (about 1.3 deg/min after calibration), accel noise 0.02 | placeholders; real raw bias is about -12.4 deg/s before calibration | record a still IMU log |
| IMU rate | asks for 93 Hz, Gazebo delivers about 99 Hz (physics step quantisation) | - |
| motor speed variation per side | none by default; `hardware:=true pico_side_noise:= pico_side_bias:=` exists | drive straight on the real robot and measure the drift |

## Mission / stack config that the sim cannot decide
- Time budget for the gate wait, deposit reserve, class-to-dock mapping, loading protocol, judges' data format.
- Whether the robot is allowed to map autonomously and whether it is moved back to the start between phases.
