#!/bin/bash
# RViz2 (Jazzy) for the hardleel robot over CycloneDDS, ROS_DOMAIN_ID=0.
# Needs: WSL networkingMode=mirrored, laptop and Pi on the same Wi-Fi (not the laptop hotspot).
# usage: hardleel_rviz_d0.sh [pi_ip]          -> open RViz
#        hardleel_rviz_d0.sh check [pi_ip]    -> list the Pi's topics and rates, no RViz
if [ "$1" = check ]; then CHECK=1; shift; fi
PI=${1:-192.168.137.209}
DIR=$(cd "$(dirname "$0")" && pwd)
MYIP=$(ip -4 route get "$PI" 2>/dev/null | grep -oP 'src \K[0-9.]+')
if [ -z "$MYIP" ]; then echo "no route to the Pi ($PI) from WSL - same Wi-Fi? mirrored mode?"; exit 1; fi
ping -c1 -W2 "$PI" >/dev/null || echo "warning: Pi $PI does not answer ping"

# Pin Cyclone to the interface that routes to the Pi (mirrored WSL has several interfaces;
# without the pin topics are discovered but no data arrives).
cat > "$DIR/cyclonedds_d0.xml" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface address="$MYIP"/></Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
      <Peers><Peer Address="$PI"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$DIR/cyclonedds_d0.xml
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE
echo "WSL $MYIP -> Pi $PI, domain 0, CycloneDDS"

# --no-daemon: the ros2 CLI daemon hung here (13 Nov)
if [ -n "$CHECK" ]; then
  timeout 25 ros2 topic list --no-daemon --spin-time 10
  for t in /imu/data_raw /scan /map; do
    if timeout 20 ros2 topic echo "$t" --once --no-daemon --field header >/dev/null 2>&1; then
      echo "$t: data OK"; else echo "$t: no data"; fi
  done
  exit 0
fi
exec rviz2 -d "$DIR/hardleel_d0.rviz"
