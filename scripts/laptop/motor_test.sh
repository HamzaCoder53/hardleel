#!/bin/bash
# MOTOR TEST window: same keys as drive.sh, but straight to /cmd_vel - the collision monitor CANNOT stop it.
# Wheels off the ground or a clear space. The motor bridge still stops 0.3 s after the last command.
source "$(dirname "$0")/ros_env.sh"
exec python3 "$(dirname "$0")/drive.py" --direct
