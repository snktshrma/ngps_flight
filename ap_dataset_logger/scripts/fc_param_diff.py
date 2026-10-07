#!/usr/bin/env python3
"""Diff the FC's live parameters against the parameters recorded in a dataflash log (read-only).

    fc_param_diff.py path/to/flight.BIN
"""
import sys
import time
from pymavlink import mavutil

ref_path = sys.argv[1]
ref = {}
log = mavutil.mavlink_connection(ref_path)
while True:
    x = log.recv_match(type='PARM')
    if x is None:
        break
    ref[x.Name] = x.Value            # last value in the log wins

m = mavutil.mavlink_connection('tcp:192.168.144.15:5760', source_system=240, source_component=190)
hb = m.wait_heartbeat(timeout=15)
if not hb:
    raise SystemExit('FC: no heartbeat')
while hb.get_srcComponent() != 1 or hb.type == mavutil.mavlink.MAV_TYPE_GCS:
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
print('FC armed:', bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
live, count = {}, None
m.mav.param_request_list_send(m.target_system, m.target_component)
last = time.time()
while time.time() - last < 6:
    r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if r is None:
        if count and len(live) < count:            # re-request the gaps
            for i in range(count):
                if i not in live.get('_idx', set()):
                    m.mav.param_request_read_send(m.target_system, m.target_component, b'', i)
        continue
    live[r.param_id] = r.param_value
    live.setdefault('_idx', set()).add(r.param_index)
    count = r.param_count
    last = time.time()
idx = live.pop('_idx', set())
print(f'live params: {len(live)} of {count};  log params: {len(ref)}')
changed = sorted(k for k in ref if k in live and abs(live[k] - ref[k]) > 1e-4 * max(1.0, abs(ref[k])))
missing = sorted(k for k in ref if k not in live)
new = sorted(k for k in live if k not in ref)
print(f'\nCHANGED ({len(changed)}):')
for k in changed:
    print(f'  {k:20s} last flight {ref[k]:<12g} now {live[k]:g}')
print(f'\nonly in last flight ({len(missing)}): {" ".join(missing[:40])}')
print(f'only on FC now ({len(new)}): {" ".join(new[:40])}')
