#!/bin/bash
# Start the robot in order:  1) IMU  ->  wait until /imu/data_raw publishes
#                            2) lidar (lidar_supervisor.sh) + Cartographer mapping (carto_mapping.launch.py)
# Everything runs in tmux session "robot" (windows imu, lidar, carto); logs in ~/imu.log,
# ~/lidar_supervisor.log, ~/carto.log. ROS settings (domain 0, CycloneDDS) come from ~/.bashrc,
# so every command runs in an interactive shell (bash -ic). Keep the robot still while the IMU calibrates.
# Stop:  tmux kill-session -t robot
IMU_WAIT_S=${IMU_WAIT_S:-60}
RUN='bash -ic'

if tmux has-session -t robot 2>/dev/null; then
  echo "tmux session 'robot' already running (stop it: tmux kill-session -t robot)"; exit 1
fi
ls /dev/serial/by-id/usb-Arduino* >/dev/null 2>&1 || ls /dev/ttyACM* >/dev/null 2>&1 || {
  echo "IMU (Arduino Nano) not found on USB - plug it in and try again"; exit 1; }

echo "1/2 starting IMU ..."
tmux new -d -s robot -n imu "$RUN 'ros2 launch robot_slam imu.launch.py ${IMU_PORT:+imu_port:=$IMU_PORT} 2>&1 | tee ~/imu.log'"
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

echo "2/2 starting lidar + Cartographer ..."
tmux new-window -t robot -n lidar "$RUN 'bash ~/lidar_supervisor.sh 2>&1 | tee ~/lidar_supervisor.log'"
tmux new-window -t robot -n carto "$RUN 'ros2 launch robot_slam carto_mapping.launch.py use_lidar:=false 2>&1 | tee ~/carto.log'"
echo "started: tmux attach -t robot   (windows: imu, lidar, carto)"
