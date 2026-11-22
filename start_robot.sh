#!/bin/bash
# Start the robot in order:  1) IMU  ->  wait until /imu/data_raw publishes
#                            2) lidar (lidar_supervisor.sh) + Cartographer mapping (carto_mapping.launch.py)
#                            3) ESP32 motor bridge + tuning helper (for the laptop tuning GUI, htune)
# Everything runs in tmux session "robot" (windows imu, lidar, carto, motors, tuning); logs in ~/imu.log,
# ~/lidar_supervisor.log, ~/carto.log, ~/motors.log, ~/tuning.log. ROS settings (domain 0, CycloneDDS) come
# from ~/.bashrc, so every command runs in an interactive shell (bash -ic). Keep the robot still while the
# IMU calibrates.
# Tuning: every run starts from the code defaults + values SAVED in the tuning GUI (~/.hardleel); live GUI
# changes and Cartographer "Apply" values from the previous run are dropped here.
# Modes:  bash ~/start_robot.sh        manual mapping (teleop), IMU EKF gives odom->base_link
#         bash ~/start_robot.sh auto   auto mapping: IMU without EKF + window "odom" (rf2o + nav EKF on the Pi);
#                                      then on the laptop: hauto (Nav2 + frontier explorer, Start in htune)
# Stop:  tmux kill-session -t robot
MODE=${1:-manual}
case $MODE in manual) IMU_ARGS= ;; auto) IMU_ARGS="use_ekf:=false" ;; *) echo "usage: start_robot.sh [auto]"; exit 1 ;; esac
IMU_WAIT_S=${IMU_WAIT_S:-60}
RUN='bash -ic'
SAVED=~/.hardleel/saved_params.yaml

if tmux has-session -t robot 2>/dev/null; then
  echo "tmux session 'robot' already running (stop it: tmux kill-session -t robot)"; exit 1
fi
ls /dev/serial/by-id/usb-Arduino* >/dev/null 2>&1 || ls /dev/ttyACM* >/dev/null 2>&1 || {
  echo "IMU (Arduino Nano) not found on USB - plug it in and try again"; exit 1; }

rm -f ~/.hardleel/session_carto.yaml          # last run's unsaved Cartographer values
[ -s "$SAVED" ] && echo "using saved tuning values: $SAVED" || SAVED=
# which ttyUSB is the lidar / the ESP32 (identical CP2102 adapters; plugs can be swapped): ask them once
eval "$($RUN 'python3 -m robot_slam.serial_ports --shell' 2>/dev/null | grep "^LIDAR_PORT=")"
echo "serial ports: lidar ${LIDAR_PORT:-?}, ESP32 ${ESP32_PORT:-?}"
PFILE="--ros-args ${ESP32_PORT:+-p port:=$ESP32_PORT} ${SAVED:+--params-file $SAVED}"

echo "1/3 starting IMU ..."
tmux new -d -s robot -n imu "$RUN 'ros2 launch robot_slam imu.launch.py $IMU_ARGS ${IMU_PORT:+imu_port:=$IMU_PORT} 2>&1 | tee ~/imu.log'"
# imu_serial_node logs "Publishing." after its gyro calibration; then confirm the topic
# (a ros2 CLI call alone takes >3 s to start on the Pi, so give it 10 s)
for ((t = 0; t < IMU_WAIT_S; t += 2)); do
  grep -q "Publishing." ~/imu.log 2>/dev/null && break; sleep 2
done
$RUN 'timeout 10 ros2 topic echo --once /imu/data_raw' >/dev/null 2>&1 && ok=1
if [ -z "$ok" ]; then
  echo "IMU did not publish /imu/data_raw within ${IMU_WAIT_S}s - see ~/imu.log; SLAM not started"; exit 1
fi
echo "    /imu/data_raw is publishing"

echo "2/3 starting lidar + Cartographer ..."
tmux new-window -t robot -n lidar "$RUN 'LIDAR_PORT=$LIDAR_PORT bash ~/lidar_supervisor.sh 2>&1 | tee ~/lidar_supervisor.log'"
tmux new-window -t robot -n carto "$RUN 'ros2 launch robot_slam carto_mapping.launch.py use_lidar:=false 2>&1 | tee ~/carto.log'"

if [ "$MODE" = auto ]; then
  echo "    auto mode: rf2o + navigation EKF on the Pi (odom->base_link, /odom)"
  tmux new-window -t robot -n odom "$RUN 'ros2 launch robot_slam pi_odom.launch.py 2>&1 | tee ~/odom.log'"
fi
echo "3/3 starting motor bridge + tuning helper ..."
tmux new-window -t robot -n motors "$RUN 'ros2 run motor_teleop esp32_encoder_bridge $PFILE 2>&1 | tee ~/motors.log'"
tmux new-window -t robot -n tuning "$RUN 'ros2 run robot_slam tuning_helper 2>&1 | tee ~/tuning.log'"
echo "started ($MODE): tmux attach -t robot   (windows: $(tmux list-windows -t robot -F '#W' | paste -sd' '))"
