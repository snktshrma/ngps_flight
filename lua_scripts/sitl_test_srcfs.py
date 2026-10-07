#!/usr/bin/env python3
"""SITL test of srcfs_gps_fallback.lua: every trigger, plus no false switch while vision is good.

    python3 lua_scripts/sitl_test_srcfs.py [--ap ~/ngps_ws/src/ardupilot]

Copter SITL at the nagano1 site, script loaded from ./scripts. Vision is fed as MAVLink
VISION_POSITION_ESTIMATE (VISO_TYPE 1) from the simulator's true position plus a chosen offset, so it is
healthy and right unless a case breaks it. Each case: hover at 30 m in GUIDED on GPS, switch to EKF source
set 2 (vision), check 8 s of silence, inject the fault, expect "SRCFS: <reason> -> src1 GPS".
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
HOME = (36.323136, 138.618168, 921.0, 0)
PARMS = """VISO_TYPE 1
VISO_DELAY_MS 50
VISO_QUAL_MIN 0
VISO_POS_M_NSE 0.5
EK3_SRC1_POSXY 3
EK3_SRC1_VELXY 3
EK3_SRC1_POSZ 1
EK3_SRC1_VELZ 3
EK3_SRC1_YAW 1
EK3_SRC2_POSXY 6
EK3_SRC2_VELXY 0
EK3_SRC2_POSZ 1
EK3_SRC2_VELZ 0
EK3_SRC2_YAW 1
SCR_ENABLE 1
FS_EKF_ACTION 1
BATT_LOW_TIMER 1
"""
R_EARTH = 6378137.0


class Sitl:
    def __init__(self, m):
        self.m = m
        self.lock = threading.Lock()
        self.texts = []
        self.truth = None          # (lat, lon) from SIMSTATE
        self.origin = None
        self.local = None          # LOCAL_POSITION_NED (EKF)
        self.offset_ne = [0.0, 0.0]
        self.feed = True
        self.stop = False
        self.yaw = 0.0

    def ned_truth(self):
        la, lo = self.truth
        la0, lo0 = self.origin
        return (math.radians(la - la0) * R_EARTH, math.radians(lo - lo0) * R_EARTH * math.cos(math.radians(la0)))

    def reader(self):
        while not self.stop:
            x = self.m.recv_match(blocking=True, timeout=0.1)
            if x is None:
                continue
            t = x.get_type()
            with self.lock:
                if t == 'SIMSTATE':
                    self.truth = (x.lat * 1e-7, x.lng * 1e-7)
                elif t == 'LOCAL_POSITION_NED':
                    self.local = (x.x, x.y, x.z)
                elif t == 'ATTITUDE':
                    self.yaw = x.yaw
                elif t == 'GPS_GLOBAL_ORIGIN' and self.origin is None:
                    self.origin = (x.latitude * 1e-7, x.longitude * 1e-7)
                elif t == 'STATUSTEXT':
                    self.texts.append((time.time(), x.text))
                    print(f'    FC: {x.text}', flush=True)

    def feeder(self):
        while not self.stop:
            time.sleep(0.04)
            with self.lock:
                ok = self.feed and self.truth and self.origin and self.local
                if ok:
                    n, e = self.ned_truth()
                    n, e = n + self.offset_ne[0], e + self.offset_ne[1]
                    z, yaw = self.local[2], self.yaw
            if ok:
                self.m.mav.vision_position_estimate_send(int(time.time() * 1e6), n, e, z, 0, 0, yaw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ap', default=os.path.expanduser('~/ngps_ws/src/ardupilot'))
    a = ap.parse_args()
    work = tempfile.mkdtemp(prefix='srcfs_sitl_')
    os.makedirs(f'{work}/scripts')
    shutil.copy(f'{HERE}/srcfs_gps_fallback.lua', f'{work}/scripts/')
    open(f'{work}/test.parm', 'w').write(PARMS)
    sitl = subprocess.Popen([f'{a.ap}/build/sitl/bin/arducopter', '--model', 'quad', '--speedup', '1', '-w',
                             '--defaults', f'{a.ap}/Tools/autotest/default_params/copter.parm,{work}/test.parm',
                             '--home', ','.join(map(str, HOME))],
                            cwd=work, stdout=open(f'{work}/sitl.log', 'w'), stderr=subprocess.STDOUT)
    results = []
    try:
        time.sleep(3)
        m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
        m.wait_heartbeat(timeout=30)
        for mid, hz in ((164, 10), (32, 10), (30, 20)):
            m.mav.command_long_send(1, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
        s = Sitl(m)
        threading.Thread(target=s.reader, daemon=True).start()
        threading.Thread(target=s.feeder, daemon=True).start()

        def cmd(c, *p):
            m.mav.command_long_send(1, 1, c, 0, *(list(p) + [0] * (7 - len(p))))

        def pset(name, v):
            m.param_set_send(name, v)
            time.sleep(0.3)

        def wait_text(pat, timeout, since):
            end = time.time() + timeout
            while time.time() < end:
                with s.lock:
                    hit = [t for ts, t in s.texts if ts >= since and pat in t]
                if hit:
                    return hit[0], time.time() - since
                time.sleep(0.1)
            return None, timeout

        def srcfs_since(since):
            with s.lock:
                return [t for ts, t in s.texts if ts >= since and t.startswith('SRCFS:') and 'loaded' not in t]

        t0 = time.time()
        while s.origin is None and time.time() - t0 < 120:
            cmd(512, 49)
            time.sleep(2)
        assert s.origin, 'no EKF origin'
        assert wait_text('SRCFS: GPS fallback loaded', 1, 0)[0], 'script did not load'

        def hover():
            """GUIDED, armed, ~30 m, on GPS, vision healthy and true."""
            with s.lock:
                s.offset_ne = [0.0, 0.0]
                s.feed = True
            cmd(42007, 1)   # MAV_CMD_SET_EKF_SOURCE_SET 1 (GPS)
            time.sleep(1)
            m.set_mode('GUIDED')
            end = time.time() + 60
            while not m.motors_armed() and time.time() < end:
                m.set_mode('GUIDED')
                cmd(400, 1)
                time.sleep(2)
            assert m.motors_armed(), 'could not arm'
            if -s.local[2] < 20:
                cmd(22, 0, 0, 0, 0, 0, 0, 30)
                end = time.time() + 40
                while -s.local[2] < 28 and time.time() < end:
                    time.sleep(0.5)
            else:
                m.mav.set_position_target_local_ned_send(0, 1, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111111000,
                                                         s.local[0], s.local[1], -30, 0, 0, 0, 0, 0, 0, 0, 0)
            time.sleep(4)

        def to_vision():
            since = time.time()
            cmd(42007, 2)
            time.sleep(8)
            false = srcfs_since(since)
            return false

        def case(name, inject, expect, timeout=30, restore=None):
            print(f'\n=== {name}', flush=True)
            hover()
            false = to_vision()
            since = time.time()
            info = inject()
            text, dt = wait_text(expect, timeout, since)
            extra = ''
            if callable(info):
                extra = info()
            ok = text is not None and not false
            results.append((name, 'PASS' if ok else 'FAIL', f'{dt:.1f} s' if text else 'no switch',
                            text or '', 'FALSE SWITCH before fault: ' + '; '.join(false) if false else '', extra))
            if restore:
                restore()

        # 0. healthy vision for 30 s: must stay on vision
        print('\n=== no false switch (30 s healthy vision)', flush=True)
        hover()
        since = time.time()
        cmd(42007, 2)
        time.sleep(30)
        false = srcfs_since(since)
        results.append(('no false switch, 30 s', 'PASS' if not false else 'FAIL', '', '; '.join(false), '', ''))

        # 1. vision wanders off: ramp the offset east at 0.5 m/s
        ramp = {'run': False}

        def drift():
            ramp['run'] = True

            def go():
                while ramp['run']:
                    with s.lock:
                        s.offset_ne[1] += 0.05
                    time.sleep(0.1)
            threading.Thread(target=go, daemon=True).start()
            return lambda: f'vision offset {s.offset_ne[1]:.1f} m when switched'

        def stop_drift():
            ramp['run'] = False
        case('vision wanders > SRCFS_DIST_M (20 m)', drift, 'off GPS -> src1 GPS', timeout=75, restore=stop_drift)

        # 2. vision stops
        def kill_feed():
            with s.lock:
                s.feed = False
        case('vision stops (VisOdom unhealthy)', kill_feed, 'VisOdom unhealthy -> src1 GPS')

        # 3. RC lost
        case('RC lost', lambda: pset('SIM_RC_FAIL', 1), 'RC lost -> src1 GPS',
             restore=lambda: (pset('SIM_RC_FAIL', 0), time.sleep(3)))

        # 5. GPS unavailable: must refuse to switch, and say so
        print('\n=== GPS lost, then vision stops: must stay and warn', flush=True)
        hover()
        false = to_vision()
        pset('SIM_GPS1_ENABLE', 0)
        time.sleep(3)
        since = time.time()
        kill_feed()
        text, dt = wait_text('no GPS: stay', 10, since)
        switched = [t for t in srcfs_since(since) if '-> src1' in t]
        results.append(('GPS lost: no switch, warns', 'PASS' if text and not switched and not false else 'FAIL',
                        f'{dt:.1f} s' if text else 'no warning', text or '', '; '.join(switched), ''))
        # 4. EKF failsafe: vision gone with no GPS -> EKF failsafe; GPS back -> switch for that reason
        print('\n=== EKF failsafe (from the case above), GPS returns', flush=True)
        since = time.time()
        pset('SIM_GPS1_ENABLE', 1)
        text, dt = wait_text('EKF failsafe -> src1 GPS', 60, since)
        results.append(('EKF failsafe, switches once GPS is back', 'PASS' if text else 'FAIL',
                        f'{dt:.1f} s' if text else 'no switch', text or '', '', ''))
        time.sleep(10)

        # 6. battery failsafe (last: it latches)
        case('battery failsafe', lambda: pset('BATT_LOW_VOLT', 20.0), 'battery failsafe -> src1 GPS')
    finally:
        try:
            s.stop = True
        except NameError:
            pass
        sitl.terminate()
        sitl.wait(10)

    print('\n\nRESULT')
    for r in results:
        print(f'  {r[1]:4}  {r[0]:<38} {r[2]:<10} {r[3]}  {r[4]} {r[5]}')
    print('work dir', work)
    sys.exit(0 if results and all(r[1] == 'PASS' for r in results) else 1)


if __name__ == '__main__':
    main()
