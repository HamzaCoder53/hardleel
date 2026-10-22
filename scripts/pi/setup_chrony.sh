#!/bin/bash
# Run once on the Pi:  sudo bash ~/setup_chrony.sh
# Makes the Pi the time reference for the laptop (works with no internet at the venue).
# The laptop's WSL chrony syncs to the Pi (scripts/laptop/setup_wsl_chrony.sh).
set -e
DEBIAN_FRONTEND=noninteractive apt-get install -y chrony
cat > /etc/chrony/conf.d/hardleel.conf <<'X'
# hardleel: serve time on the LAN even when no internet source is reachable
local stratum 8
allow 0.0.0.0/0
X
systemctl restart chrony
sleep 3; chronyc -n tracking | head -5
