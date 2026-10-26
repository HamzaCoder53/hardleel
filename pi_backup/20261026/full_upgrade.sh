#!/bin/bash
flags() { echo "[power] $(vcgencmd get_throttled) events=$(journalctl -k -b --no-pager | grep -c "Undervoltage detected") $(vcgencmd measure_temp)"; }
export NEEDRESTART_MODE=l
echo "== start $(date +%T)  kernel $(uname -r)"; flags
echo "== apt-get update"; echo hardleel | sudo -S -p "" apt-get update -qq 2>&1 | tail -2
echo "== full-upgrade ($(apt list --upgradable 2>/dev/null | grep -vc Listing) packages)"
echo hardleel | sudo -S -p "" env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=l apt-get -y full-upgrade -o Dpkg::Options::=--force-confold -o Dpkg::Options::=--force-confdef 2>&1 | grep -E "^(Setting up|Unpacking|Processing triggers|Err|E:|W:|The following|[0-9]+ upgraded)" | tail -25
echo "== apt result: exit ${PIPESTATUS[0]}"; flags
echo hardleel | sudo -S -p "" apt-get -y autoremove 2>&1 | tail -2
echo "== dpkg audit"; echo hardleel | sudo -S -p "" dpkg --audit 2>&1 | head -5; echo "(audit done)"
echo "== clean rebuild robot_slam"
source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws
rm -rf build/robot_slam install/robot_slam
colcon build --symlink-install --packages-select robot_slam motor_teleop --parallel-workers 2 2>&1 | tail -8; flags
source ~/ros2_ws/install/setup.bash
echo "== launch files installed"; ls install/robot_slam/share/robot_slam/launch | tr "\n" " "; echo
echo "== executables"; ros2 pkg executables robot_slam | tr "\n" ";"; echo
echo "== reboot needed?"; ls /var/run/reboot-required* 2>/dev/null && cat /var/run/reboot-required.pkgs 2>/dev/null | head -5 || echo "no"
echo "installed kernel(s):"; ls /boot/firmware/vmlinuz* /boot/vmlinuz* 2>/dev/null | head -4; uname -r
echo "== done $(date +%T)"; flags
