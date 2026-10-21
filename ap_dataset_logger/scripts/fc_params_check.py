#!/usr/bin/env python3
"""Read-only FC readiness check: link, armed state, firmware, the VISO/EKF/DDS params that matter. Changes nothing."""
from pymavlink import mavutil

WANT = {'VISO_TYPE': 1, 'VISO_DELAY_MS': 250, 'VISO_QUAL_MIN': 0, 'EK3_SRC1_POSXY': 3, 'EK3_SRC1_POSZ': 1,
        'EK3_SRC1_YAW': 1, 'EK3_SRC2_POSXY': 6, 'DDS_ENABLE': 1, 'DDS_IP0': 192, 'DDS_IP1': 168, 'DDS_IP2': 144,
        'DDS_IP3': 20, 'DDS_UDP_PORT': 2019, 'DDS_DOMAIN_ID': 0}
m = mavutil.mavlink_connection('tcp:192.168.144.15:5760', source_system=240, source_component=190)
hb = m.wait_heartbeat(timeout=15)
if not hb:
    raise SystemExit('FC: NO heartbeat (powered? Ethernet?)')
while hb.get_srcComponent() != 1 or hb.type == mavutil.mavlink.MAV_TYPE_GCS:
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
print('FC heartbeat ok, armed:', bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
m.mav.command_long_send(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)
av = m.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
print('firmware git', bytes(av.flight_custom_version).decode(errors='replace') if av else '?', '(want 91deccf6)')


def get(n):
    for _ in range(3):
        m.mav.param_request_read_send(1, 1, n.encode(), -1)
        r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        while r and r.param_id != n:
            r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if r:
            return r.param_value


bad = 0
for k, w in WANT.items():
    v = get(k)
    ok = v is not None and abs(v - w) < 1e-6
    bad += not ok
    print(f'  {"ok " if ok else "BAD"} {k:15s} {v}  (want {w})')
sw = [i for i in range(1, 17) if get(f'RC{i}_OPTION') == 90.0]
print('  EKF source-set switch:', sw or 'none (EKF stays on GPS)')
m.mav.command_long_send(1, 1, 512, 0, 24, 0, 0, 0, 0, 0, 0)
g = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
if g:
    print(f'  GPS fix {g.fix_type} sats {g.satellites_visible} hdop {g.eph / 100:.2f}')
print('ALL PARAMS OK' if not bad else f'{bad} PARAM(S) WRONG')
