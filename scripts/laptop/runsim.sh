#!/bin/bash
# Run one sim e2e case. Domain 42 only, lowest priority; the repo script does its own domain-42 cleanup.
source /opt/ros/jazzy/setup.bash
out=~/simrun_$1.txt; shift
nice -n 19 bash ~/hardleel/scripts/laptop/sim_e2e_test.sh "$@" > $out 2>&1
cp -r ~/simtest_m1 ${out%.txt}_logs 2>/dev/null
echo "EXIT $?" >> $out
