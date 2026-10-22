#!/bin/bash
# Run once in WSL as root:  wsl -u root bash scripts/laptop/setup_wsl_chrony.sh
# WSL syncs to the Pi first (same clock as the sensor stamps), internet second.
set -e
apt-get install -y chrony >/dev/null
mkdir -p /etc/chrony/sources.d /etc/chrony/conf.d
cat > /etc/chrony/sources.d/hardleel.sources <<'X'
# Pi on Student Wifi, on the PC hotspot
server 172.16.0.175 iburst prefer minpoll 2 maxpoll 4
server 192.168.137.5 iburst prefer minpoll 2 maxpoll 4
# fallback while the Pi does not serve time yet; WSL drifts ~0.8 ms/s so poll often
server time.google.com iburst minpoll 4 maxpoll 5
X
# Ubuntu ships "makestep 1 3"; replace it so ours applies
sed -i "s/^makestep .*/makestep 0.05 -1/" /etc/chrony/chrony.conf
cat > /etc/chrony/conf.d/hardleel.conf <<'X'
# WSL's clock jumps after the laptop sleeps: step whenever we are more than 50 ms off
makestep 0.05 -1
X
systemctl restart chrony
sleep 4; chronyc -n sources
