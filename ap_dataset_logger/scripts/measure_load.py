#!/usr/bin/env python3
"""Measure Jetson load for N seconds: tegrastats (CPU per core, GPU, RAM, power, temps) + top per process.

    measure_load.py [seconds]
"""
import re
import statistics as st
import subprocess
import sys

secs = int(sys.argv[1]) if len(sys.argv) > 1 else 30
teg = subprocess.Popen(['tegrastats', '--interval', '1000'], stdout=subprocess.PIPE, text=True)
top = subprocess.run(['top', '-b', '-d', str(secs), '-n', '2', '-w', '220'], capture_output=True, text=True).stdout
teg.terminate()
lines = teg.stdout.read().splitlines()

blk = top.split('\ntop - ')[-1].splitlines()
print('per process (top, averaged over the window):')
for l in blk:
    if re.search(r'python3|micro_ros|ros2|gst|tegrastats', l) and 'measure_load' not in l:
        f = l.split(None, 11)
        if len(f) >= 12 and float(f[8]) >= 1.0:
            cmd = f[11]
            label = next((k for k in ('vo_ngps_fusion_node', 'rtsp_camera_node', 'micro_ros_agent', 'bag record',
                                      'flight_ui', 'robot_state', 'launch') if k in cmd), cmd[:50])
            print(f'  {float(f[8]):6.1f}% CPU  {float(f[9]):4.1f}% MEM  {label}')
for l in blk:
    if l.startswith('%Cpu') or 'MiB Mem' in l:
        print(' ', l.strip())

cpu = [list(map(int, re.findall(r'(\d+)%@', m.group(1)))) for l in lines if (m := re.search(r'CPU \[(.*?)\]', l))]
num = lambda pat: [float(m.group(1)) for l in lines if (m := re.search(pat, l))]
gpu, ram, pw = num(r'GR3D_FREQ (\d+)%'), num(r'RAM (\d+)/'), num(r'VDD_IN (\d+)mW')
tc, tg = num(r'cpu@([\d.]+)C'), num(r'gpu@([\d.]+)C')
print(f'tegrastats over {len(cpu)} s:')
if cpu:
    per = [round(st.mean(c)) for c in zip(*cpu)]
    print(f'  CPU per core {per}%  overall {st.mean(sum(c) / len(c) for c in cpu):.0f}%  (6 cores, 1.51 GHz)')
if gpu:
    print(f'  GPU {st.mean(gpu):.0f}% mean, {max(gpu):.0f}% max')
if ram:
    print(f'  RAM {st.mean(ram) / 1024:.1f} GB used')
if pw:
    print(f'  power {st.mean(pw) / 1000:.1f} W mean, {max(pw) / 1000:.1f} W max')
if tc and tg:
    print(f'  temperature CPU {max(tc):.0f} C, GPU {max(tg):.0f} C')
