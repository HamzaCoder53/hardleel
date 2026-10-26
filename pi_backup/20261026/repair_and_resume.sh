#!/bin/bash
flags() { echo "[power] $(vcgencmd get_throttled) events=$(journalctl -k -b --no-pager | grep -c "Undervoltage detected") $(vcgencmd measure_temp)"; }
export NEEDRESTART_MODE=l
echo "== settle $(date +%T)"; sleep 30; flags
echo "== dpkg --configure -a"
echo hardleel | sudo -S -p "" env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=l dpkg --configure -a --force-confold 2>&1 | grep -E "^(Setting up|Processing|dpkg:|E:|Err)" | tail -15
echo "== dpkg exit ${PIPESTATUS[1]}"; flags
echo "== apt-get -f install"
echo hardleel | sudo -S -p "" env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=l apt-get -y -f install -o Dpkg::Options::=--force-confold 2>&1 | tail -4
echo "== audit"; echo hardleel | sudo -S -p "" dpkg --audit 2>&1 | head -5; echo "(audit end)"; flags
echo "== resuming full upgrade"
exec ~/full_upgrade.sh
