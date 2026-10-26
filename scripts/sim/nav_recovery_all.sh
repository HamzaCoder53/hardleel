#!/bin/bash
# Final scored pass: Nav2 goals on the saved map, then the teleport/recovery test with the default stack and with loc_v2.
cd "$(dirname "$0")/../.."
PB=docs/sim_demo/mapping_course.pbstream
NAV_NAME=nav_course bash scripts/sim/localization_demo.sh course $PB loading_metal terrain_1 terrain_2 terrain_3 junction path1_1 path1_2 finish > docs/sim_demo/nav_course.out 2>&1
NAV_NAME=recovery_baseline TELEPORT_AFTER=2 TELEPORT_TO=-4.25,-1.0,90 bash scripts/sim/localization_demo.sh course $PB loading_metal terrain_1 terrain_2 junction > docs/sim_demo/recovery_baseline.out 2>&1
NAV_NAME=recovery_v2_apply EXTRA_LOC='loc_v2:=true loc_apply_corrections:=true' TELEPORT_AFTER=2 TELEPORT_TO=-4.25,-1.0,90 bash scripts/sim/localization_demo.sh course $PB loading_metal terrain_1 terrain_2 junction > docs/sim_demo/recovery_v2_apply.out 2>&1
