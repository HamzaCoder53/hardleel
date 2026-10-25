# Source me (WSL, laptop side):  source scripts/laptop/ros_env.sh [pi_ip]
# Talks to the real robot: ROS_DOMAIN_ID=7 + CycloneDDS pinned to the interface that reaches the Pi
# (WSL mirrored networking has several; without the pin topics are discovered but no data arrives).
# Unicast only: the Pi is a peer, multicast off. The sims stay on domain 0.
HARDLEEL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_IP="${1:-$PI_IP}"
[ -z "$PI_IP" ] && PI_IP="$(timeout 8 ping.exe -n 1 -4 hardleel.local 2>/dev/null | grep -oE '\[[0-9.]+\]' | head -1 | tr -d '[]')"
PI_IP="${PI_IP:-192.168.137.8}"   # laptop hotspot (competition network)
LOCAL_IP="$(ip -4 route get "$PI_IP" | grep -oP 'src \K[0-9.]+')"
mkdir -p ~/.hardleel
cat > ~/.hardleel/cyclonedds.xml <<X
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <Interfaces><NetworkInterface address="$LOCAL_IP"/></Interfaces>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>80</MaxAutoParticipantIndex>
            <Peers><Peer Address="$PI_IP"/></Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
X
# newest installed distro (Jazzy distro, or the Humble fallback distro)
for d in jazzy humble; do [ -f /opt/ros/$d/setup.bash ] && { source /opt/ros/$d/setup.bash; break; }; done
[ -f ~/hardleel_ws/install/setup.bash ] && source ~/hardleel_ws/install/setup.bash
export ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$HOME/.hardleel/cyclonedds.xml" PI_IP
echo "hardleel: Pi=$PI_IP laptop=$LOCAL_IP ROS_DOMAIN_ID=7 cyclonedds"
# clock: sensor stamps come from the Pi, Nav2/Cartographer compare them with this clock
off=$(chronyc -n tracking 2>/dev/null | awk '/System time/ {printf "%.0f", $4*1000}')
[ -n "$off" ] && [ "$off" -gt 30 ] && echo "WARNING: WSL clock is ${off} ms off its time source - run: sudo chronyc makestep"
true
