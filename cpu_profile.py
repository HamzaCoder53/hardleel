#!/usr/bin/env python3
"""Per-process CPU load on the Pi over a time window (for finding what heats the Pi).

usage: python3 ~/cpu_profile.py [seconds=60]
CPU % is of ONE core (the Pi 4 has 4 cores = 400 % total). ROS nodes are named by their __node:= name.
Also samples temperature, ARM clock and throttling every 5 s.
"""
import os
import re
import subprocess
import sys
import time

HZ = os.sysconf('SC_CLK_TCK')


def proc_times():
    out = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/stat') as f:
                fields = f.read().rsplit(')', 1)[1].split()
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
        except OSError:
            continue
        out[pid] = (int(fields[11]) + int(fields[12]), cmd)   # utime + stime (ticks)
    return out


def name_of(cmd):
    m = re.search(r'__node:=(\S+)', cmd)
    if m:
        return m.group(1)
    if not cmd:
        return None                                          # kernel thread
    parts = cmd.split()
    exe = os.path.basename(parts[0])
    if exe.startswith('python') and len(parts) > 1:
        exe = os.path.basename(parts[1])                     # python3 /path/node -> node
    return exe


def vc(arg):
    try:
        return subprocess.run(['vcgencmd'] + arg.split(), capture_output=True, text=True).stdout.strip()
    except OSError:
        return '?'


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    t0, a = time.monotonic(), proc_times()
    with open('/proc/stat') as f:
        cpu0 = list(map(int, f.readline().split()[1:]))
    temps, clocks = [], []
    while time.monotonic() - t0 < secs:
        temps.append(float(re.findall(r'[\d.]+', vc('measure_temp'))[0]))
        clocks.append(int(re.findall(r'\d+$', vc('measure_clock arm'))[0]) / 1e6)
        time.sleep(5)
    dt, b = time.monotonic() - t0, proc_times()
    with open('/proc/stat') as f:
        cpu1 = list(map(int, f.readline().split()[1:]))
    totals = {}
    for pid, (t1, cmd) in b.items():
        if pid in a and (n := name_of(cmd)):
            totals[n] = totals.get(n, 0.0) + (t1 - a[pid][0]) / HZ / dt * 100
    busy = sum(cpu1) - sum(cpu0) - (cpu1[3] - cpu0[3]) - (cpu1[4] - cpu0[4])   # minus idle + iowait
    total_pct = busy / (sum(cpu1) - sum(cpu0)) * 400
    print(f'window {dt:.0f} s | whole Pi {total_pct:.0f} % of 400 % | load {os.getloadavg()[0]:.2f} | '
          f'temp {min(temps):.1f}-{max(temps):.1f} C | ARM clock {min(clocks):.0f}-{max(clocks):.0f} MHz | '
          f'{vc("get_throttled")}')
    print(f'{"process / node":32s} {"CPU % (1 core = 100)":>20s}')
    for n, v in sorted(totals.items(), key=lambda kv: -kv[1]):
        if v >= 1.0:
            print(f'{n:32s} {v:20.1f}')


if __name__ == '__main__':
    main()
