#!/bin/bash
# Kill ONLY processes whose environment has ROS_DOMAIN_ID=<domain> (default 42 = the sim tests).
# Never pkill by name in sim scripts: the real robot stack (domain 7) runs in the same WSL distro.
#   kill_domain.sh [domain=42] [signal=TERM]
dom=${1:-42}; sig=${2:-TERM}
for pid in $(pgrep -u "$(id -u)" -f .); do
  [ "$pid" = "$$" ] && continue
  if tr '\0' '\n' < /proc/$pid/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$dom"; then kill -$sig "$pid" 2>/dev/null; fi
done
