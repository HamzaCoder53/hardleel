#!/bin/bash
# Full regression after a model/stack change: map -> accuracy -> Nav2 goals -> mission with ramps/docks/bins.
cd "$(dirname "$0")/../.."
bash scripts/sim/mapping_demo.sh course mapping_course > docs/sim_demo/mapping_course.out 2>&1
python scripts/sim/accuracy_report.py docs/sim_demo/mapping_course.yaml > docs/sim_demo/accuracy_course.out 2>&1
NAV_NAME=nav_course bash scripts/sim/localization_demo.sh course docs/sim_demo/mapping_course.pbstream loading_metal terrain_1 terrain_2 terrain_3 junction path1_1 path1_2 finish > docs/sim_demo/nav_course.out 2>&1
EXTRA_MISSION='gate_align_heading:=true' bash scripts/sim/mission_demo.sh rehearsal_ramps gate=open features=dirt_ramp,gravel_ramp,docks,bins,screen > docs/sim_demo/rehearsal_ramps.out 2>&1
