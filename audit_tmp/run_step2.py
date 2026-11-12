import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.239', username='hardleel', password='hardleel', timeout=5)

script = """
echo "=== ROS BAGS ON PI ==="
find /home/hardleel -type d -name "*.db3" -o -name "*rosbag2*" -o -name "*.mcap" 2>/dev/null
find /home/hardleel -type f -name "*.db3" -o -name "*.mcap" 2>/dev/null
ls -la /home/hardleel/maps 2>/dev/null

echo "=== SLLIDAR LAUNCH CONFIG ==="
find /home/hardleel/ros2_ws/src/sllidar_ros2 -type f -name "*launch*"
cat /home/hardleel/ros2_ws/src/sllidar_ros2/launch/sllidar_a1_launch.py 2>/dev/null || cat /home/hardleel/ros2_ws/src/sllidar_ros2/launch/sllidar_launch.py 2>/dev/null

echo "=== IMU SERIAL NODE SOURCE ==="
cat /home/hardleel/ros2_ws/src/robot_slam/robot_slam/imu_serial_node.py 2>/dev/null

echo "=== ARDUINO NANO FIRMWARE SOURCE (IF PRESENT) ==="
find /home/hardleel -name "*.ino" 2>/dev/null
"""

stdin, stdout, stderr = ssh.exec_command(f"bash -c '{script}'")
print(stdout.read().decode())
err = stderr.read().decode()
if err:
    print("ERR:", err)

ssh.close()
