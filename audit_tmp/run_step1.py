import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.239', username='hardleel', password='hardleel', timeout=5)

script = """
echo "=== OS AND ROS ==="
lsb_release -a
echo "ROS_DISTRO in env: $ROS_DISTRO"
ls -d /opt/ros/* 2>/dev/null

echo "=== HARDWARE AND MEMORY ==="
cat /proc/device-tree/model
echo ""
free -h
vcgencmd measure_temp 2>/dev/null || cat /sys/class/thermal/thermal_zone0/temp

echo "=== NAV2 AND CARTOGRAPHER DPKG ==="
dpkg -l | grep -E "nav2|cartographer|navigation"

echo "=== COLCON LIST ==="
source /opt/ros/jazzy/setup.bash 2>/dev/null
cd /home/hardleel/ros2_ws && colcon list

echo "=== ROS NODES AND TOPICS (IF RUNNING) ==="
ros2 node list 2>/dev/null || echo "No ROS 2 daemon running"
ros2 topic list -t 2>/dev/null || echo "No topics active"
"""

stdin, stdout, stderr = ssh.exec_command(f"bash -c '{script}'")
print(stdout.read().decode())
err = stderr.read().decode()
if err:
    print("ERR:", err)

ssh.close()
