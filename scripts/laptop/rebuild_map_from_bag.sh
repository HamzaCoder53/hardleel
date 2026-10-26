#!/bin/bash
# Rebuild a map OFFLINE from a recorded bag (final stack item 7): tune Cartographer on real course data without
# the robot. Runs cartographer_offline_node over the bag as fast as possible, then writes .pgm/.yaml, tidies it and
# aligns it to course_reference.
#   bash scripts/laptop/rebuild_map_from_bag.sh <bag_dir> [out_name] [config.lua basename] [extra lua line ...]
#   e.g. ... ~/hardleel_bags/mapping_20261025_213848 office_rebuild cartographer_2d.lua \
#            "TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0"
# Scans: /scan_filtered (or /scan if the bag has no filtered scans); IMU: /imu/data. TF from the bag.
HERE="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID=43                      # private: never talks to the robot (7) or the sim tests (42)
source /opt/ros/jazzy/setup.bash; source ~/hardleel_ws/install/setup.bash
BAG=$(cd "$1" && pwd); NAME=${2:-$(basename $BAG)_rebuild}; BASE=${3:-cartographer_2d.lua}; shift 3 2>/dev/null
OUT=~/hardleel_maps/rebuild; mkdir -p $OUT
CFG=$(mktemp -d)
cp ~/hardleel_ws/install/robot_slam/share/robot_slam/config/cartographer_2d*.lua $CFG/
for line in "$@"; do echo "$line" | sed 's/^/-- override: /' >> $CFG/notes.txt; sed -i "s|^return options|$line\nreturn options|" $CFG/$BASE; done
SCAN=/scan_filtered
ros2 bag info $BAG 2>/dev/null | grep -q "Topic: /scan_filtered " || SCAN=/scan
echo "bag $BAG -> $OUT/$NAME (scans $SCAN, config $BASE $*)"
timeout 1800 ros2 run cartographer_ros cartographer_offline_node -configuration_directory $CFG \
  -configuration_basenames $BASE -bag_filenames $BAG -save_state_filename $OUT/$NAME.pbstream \
  --ros-args -r scan:=$SCAN -r imu:=/imu/data -p use_sim_time:=true > $OUT/$NAME.log 2>&1
echo "offline exit $? ($(grep -ciE 'error' $OUT/$NAME.log) error lines)"
[ -f $OUT/$NAME.pbstream ] || { tail -5 $OUT/$NAME.log; exit 1; }
ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename $OUT/$NAME.pbstream \
  -map_filestem $OUT/$NAME -resolution 0.05 > /dev/null 2>&1
ls -la $OUT/$NAME.* | awk '{print "  ", $9, $5}'
ros2 run robot_slam map_tidy $OUT/$NAME.yaml 2>&1 | tail -20
