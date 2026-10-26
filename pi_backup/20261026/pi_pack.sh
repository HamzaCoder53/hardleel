#!/bin/bash
cd ~
rm -f ~/pi_backup.tgz ~/pi_files.txt
{
  find ros2_ws/src -type f ! -path '*/__pycache__/*' ! -name '*.pyc' -size -3M
  ls ~/*.sh ~/*.py ~/*.xml ~/*.launch.py 2>/dev/null | sed "s|$HOME/||"
  ls -d launch_backup_2026-10-15/* backup_pre_terrain_2026-10-22/* 2>/dev/null
  ls hw_logs/* hwtest_logs/* ~/motor_bridge*.log 2>/dev/null | sed "s|$HOME/||"
  find maps -type f -size -2M
  find pi_cam camera -maxdepth 1 -type f \( -name '*.py' -o -name '*.sh' -o -name '*.md' -o -name '*.yaml' \) 2>/dev/null
  ls /boot/firmware/config.txt /boot/firmware/cmdline.txt 2>/dev/null
  ls ~/.config/systemd/user/* /etc/systemd/system/hardleel* 2>/dev/null
} | sort -u > ~/pi_files.txt
tar czf ~/pi_backup.tgz -T ~/pi_files.txt 2>/dev/null
wc -l < ~/pi_files.txt; ls -l ~/pi_backup.tgz | awk '{print $5" bytes"}'
