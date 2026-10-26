#!/bin/bash
# Stop every sim process of THIS repo (domain-42 sim stack). Safe to run anytime; never run from a command line that
# itself contains these names (pkill -f would match its own shell) - that is why it is a script file.
for pat in ".pixi/envs/default/lib/" ".pixi/envs/default/bin/python" "gz sim" parameter_bridge "ros2 launch" ros2_ws/install mac_ws/install "ros2 run" "ros2 bag" cartographer_node probe_rates nav_goals; do
  for pid in $(pgrep -f "$pat"); do [ "$pid" != "$$" ] && [ "$pid" != "$PPID" ] && kill -9 "$pid" 2>/dev/null; done
done
exit 0
