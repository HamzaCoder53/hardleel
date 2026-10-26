#!/bin/bash
# Car-style keyboard driving (W/S speed that stays, A/D turn, SPACE stop) - see drive.py. teleop.sh = old layout.
source "$(dirname "$0")/ros_env.sh"
exec python3 "$(dirname "$0")/drive.py"
