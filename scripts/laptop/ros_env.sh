# Source me (WSL, laptop side):  source scripts/laptop/ros_env.sh
# Talks to the real robot on ROS_DOMAIN_ID=7.
#
# Default middleware: rmw_zenoh. WSL (mirrored or NAT) cannot reach the Windows mobile-hotspot subnet with
# UDP, so DDS discovery never reaches the Pi. Instead:
#   laptop nodes -> local rmw_zenohd router (started here if missing) -> TCP 7447 -> Windows portproxy
#   (0.0.0.0:7447 -> 192.168.137.8:7447, scripts/laptop/windows_zenoh_proxy.ps1) -> Pi rmw_zenohd -> Pi nodes
# HARDLEEL_RMW=cyclone gives the old CycloneDDS unicast setup (Pi and laptop on the same router Wi-Fi).
HARDLEEL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_IP="${PI_IP:-192.168.137.8}"   # laptop hotspot (competition network)
# newest installed distro (Jazzy distro, or the Humble fallback distro)
for d in jazzy humble; do [ -f /opt/ros/$d/setup.bash ] && { source /opt/ros/$d/setup.bash; break; }; done
[ -f ~/hardleel_ws/install/setup.bash ] && source ~/hardleel_ws/install/setup.bash
export ROS_DOMAIN_ID=7 PI_IP
mkdir -p ~/.hardleel
if [ "${HARDLEEL_RMW:-zenoh}" = cyclone ]; then
  LOCAL_IP="$(ip -4 route get "$PI_IP" | grep -oP 'src \K[0-9.]+')"
  cat > ~/.hardleel/cyclonedds.xml <<X
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any">
  <General><Interfaces><NetworkInterface address="$LOCAL_IP"/></Interfaces><AllowMulticast>false</AllowMulticast></General>
  <Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>80</MaxAutoParticipantIndex>
    <Peers><Peer Address="$PI_IP"/></Peers></Discovery></Domain></CycloneDDS>
X
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$HOME/.hardleel/cyclonedds.xml"
  echo "hardleel: Pi=$PI_IP laptop=$LOCAL_IP ROS_DOMAIN_ID=7 cyclonedds"
else
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  unset CYCLONEDDS_URI
  # the Windows host as seen from WSL (NAT gateway); its portproxy forwards 7447 to the Pi
  WIN_HOST="$(ip -4 route show default | awk '{print $3; exit}')"
  if ! pgrep -x rmw_zenohd >/dev/null; then
    ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/$WIN_HOST:7447\"]" \
      nohup ros2 run rmw_zenoh_cpp rmw_zenohd > ~/.hardleel/zenohd.log 2>&1 &
    sleep 2
  fi
  echo "hardleel: Pi=$PI_IP via Windows $WIN_HOST:7447 ROS_DOMAIN_ID=7 zenoh (router log ~/.hardleel/zenohd.log)"
fi
# clock: sensor stamps come from the Pi, Nav2/Cartographer compare them with this clock
off=$(chronyc -n tracking 2>/dev/null | awk '/System time/ {v=$4*1000; if (v<0) v=-v; printf "%.0f", v}')
[ -n "$off" ] && [ "$off" -gt 30 ] && echo "WARNING: WSL clock is ${off} ms off its time source - run: sudo chronyc makestep"
true
