#!/bin/bash
echo "--- kernel usb/ftdi events (last 15)"
journalctl -k --no-pager 2>&1 | grep -i -E "ftdi|ttyUSB|usb 1-1\.1|disconnect|reset|over-current" | tail -15 | cut -c1-150
echo "--- probe"
timeout 40 python3 ~/lidar_probe.py 2>&1 | tail -8
echo "--- who holds the port"; fuser -v /dev/ttyUSB0 2>&1 | head -4
