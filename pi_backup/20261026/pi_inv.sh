#!/bin/bash
cd ~
echo "--- host: $(hostname) $(date)"; df -h / | tail -1
echo "--- top-level sizes"; du -sh --apparent-size * .config 2>/dev/null | sort -h | tail -25
echo "--- ros2_ws/src sizes"; du -sh --apparent-size ros2_ws/src/* 2>/dev/null
echo "--- git repos"; find ~ -maxdepth 3 -name .git -type d 2>/dev/null
echo "--- loose scripts"; ls ~/*.sh ~/*.py 2>/dev/null | tr '\n' ' '
echo; echo "--- maps"; ls ~/maps 2>/dev/null | wc -l; du -sh ~/maps 2>/dev/null
