#!/usr/bin/env python3
"""Post-flight check of what ArduPilot received from VO+NGPS, from the dataflash (.BIN) log alone.

    fc_log_check.py --download --name flight5      # fetch the newest log over MAVFTP (waits for disarm)
    fc_log_check.py path/to/00000015.BIN            # check a log you already have

VISP is the vision position exactly as the FC received it (VISO_TYPE 1 applies no rotation or
offset). Each check is PASS / WARN / FAIL / INCONCLUSIVE against the EKF (XKF1, which navigates on
GPS during these flights), and the verdict says whether the data is fit for a first ExtNav test --
a hover with GPS one switch-flip away. It does not, and cannot, prove the EKF will fly well on it.

Read-only towards the FC: --download only reads files over MAVFTP.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
from pymavlink import mavutil

BAGS = os.path.expanduser('~/ngps_ws/src/ngps_flight/bags')
FC = 'tcp:192.168.144.15:5760'
STUCK = 4294967295
VISO_DELAY_MAX_MS = 250          # EKF3 caps visual-odometry delay here (AP_NavEKF3_core.cpp)


def download(name: str, n=None) -> str:
    from pymavlink.mavftp import MAVFTP
    m = mavutil.mavlink_connection(FC, source_system=240, source_component=190)
    if not m.wait_heartbeat(timeout=30):
        sys.exit('no heartbeat from the FC')
    while True:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if hb and hb.get_srcComponent() == 1 and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            if not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                break
            print('FC armed: waiting for disarm (logs cannot be read while armed)...', flush=True)
            time.sleep(5)
    ftp = MAVFTP(m, m.target_system, m.target_component)
    last = os.path.join(BAGS, '.LASTLOG.TXT')
    if n is None:   # newest; with LOG_DISARMED a reboot after landing starts a new, empty one: use --log
        ftp.cmd_get(['/APM/LOGS/LASTLOG.TXT', last])
        ftp.process_ftp_reply('OpenFileRO', timeout=30)
        n = int(open(last).read().strip())
    dst = os.path.join(BAGS, f'{name}_{time.strftime("%Y%m%d")}_dataflash_{n:08d}.BIN')
    print(f'downloading log {n} -> {dst}', flush=True)
    t0 = time.time()
    ftp.cmd_get([f'/APM/LOGS/{n:08d}.BIN', dst])
    r = ftp.process_ftp_reply('OpenFileRO', timeout=1200)
    if r is None or r.error_code != 0 or not os.path.getsize(dst):
        sys.exit(f'download failed: {r}')
    print(f'  {os.path.getsize(dst) / 1e6:.1f} MB in {time.time() - t0:.0f} s')
    return dst


def load(path):
    m = mavutil.mavlink_connection(path)
    V, K, A, B, S, msgs, arm, params = [], [], [], [], set(), [], [], {}
    while True:
        x = m.recv_match(type=['VISP', 'XKF1', 'ATT', 'CTUN', 'XKFS', 'MSG', 'ARM', 'PARM'])
        if x is None:
            break
        t = x.get_type()
        if t == 'VISP':
            V.append((x.TimeUS / 1e6, x.CTimeMS, x.PX, x.PY, x.PZ, x.Y, x.Ign))
        elif t == 'XKF1' and x.C == 0:
            K.append((x.TimeUS / 1e6, x.PN, x.PE, x.PD))
        elif t == 'ATT':
            A.append((x.TimeUS / 1e6, x.Yaw))
        elif t == 'CTUN':
            B.append((x.TimeUS / 1e6, x.Alt))
        elif t == 'XKFS' and x.C == 0:
            S.add(int(x.SS))
        elif t == 'MSG':
            msgs.append((x.TimeUS / 1e6, x.Message))
        elif t == 'ARM':
            arm.append((x.TimeUS / 1e6, int(x.ArmState)))
        elif t == 'PARM':
            params[x.Name] = x.Value
    return [np.array(a, float) for a in (V, K, A, B)] + [S, msgs, arm, params]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bin', nargs='?')
    ap.add_argument('--download', action='store_true')
    ap.add_argument('--name', default='flight')
    ap.add_argument('--log', type=int, help='FC log number to download (default: the newest)')
    a = ap.parse_args()
    path = download(a.name, a.log) if a.download else a.bin
    if not path:
        ap.error('give a .BIN or --download')
    V, K, A, B, S, msgs, arm, prm = load(path)
    res = []

    def add(name, status, detail):
        res.append((name, status, detail))
        print(f'  {status:12s} {name:24s} {detail}')

    print(f'\n{path}')
    print(f'  params: VISO_TYPE {prm.get("VISO_TYPE")}, VISO_DELAY_MS {prm.get("VISO_DELAY_MS")}, '
          f'EK3_SRC1_POSXY {prm.get("EK3_SRC1_POSXY")}, EK3_SRC2_POSXY {prm.get("EK3_SRC2_POSXY")}')
    if not len(V):
        add('vision data', 'FAIL', 'no VISP messages: the FC received nothing on /ap/tf')
        return verdict(res)
    if not len(B) or (B[:, 1] > 5).sum() < 10:
        add('flight', 'FAIL', 'never above 5 m: nothing to judge')
        return verdict(res)
    air = B[B[:, 1] > 5, 0]
    t0, t1 = air[0], air[-1]
    w = (V[:, 0] > t0) & (V[:, 0] < t1)
    v = V[w]
    print(f'  airborne {t1 - t0:.0f} s, max altitude {B[:, 1].max():.0f} m, {len(v)} VISP samples in the air')

    # 1. timestamps (the firmware fix)
    stuck = int((V[:, 1] == STUCK).sum())
    step = np.median(np.diff(V[:, 1]))
    add('timestamps (CTimeMS)', 'FAIL' if stuck else 'PASS',
        f'{stuck} stuck at 4294967295; median step {step:.0f} ms' + ('  -> flash the timestamp-fix firmware' if stuck else ''))

    # 2. continuity
    dt = np.diff(v[:, 0])
    rate = len(v) / (t1 - t0)
    gaps = int((dt > 0.3).sum())
    add('rate / gaps', 'FAIL' if gaps or rate < 8 else 'PASS',
        f'{rate:.1f} Hz in the air, {gaps} gaps over 300 ms (VisOdom timeout), worst {dt.max() * 1e3:.0f} ms')
    ign = int(v[:, 6].sum())
    if ign:
        add('ignored samples', 'WARN', f'{ign} VISP samples flagged Ign')

    # 3. single publisher, no junk
    zeros = int(((np.abs(v[:, 2]) < 1e-3) & (np.abs(v[:, 3]) < 1e-3)).sum())
    jump = np.hypot(*np.diff(v[:, 2:4], axis=0).T)
    big = int((jump > 2.0).sum())
    add('single clean stream', 'FAIL' if zeros else ('WARN' if big else 'PASS'),
        f'{zeros} exact (0,0) samples (a second publisher), {big} steps over 2 m (max {jump.max():.2f} m)')

    # 4. frame vs EKF: rotation, scale, mirror, offset, height, heading
    ek = np.c_[np.interp(v[:, 0], K[:, 0], K[:, 1]), np.interp(v[:, 0], K[:, 0], K[:, 2])]
    P, Q = v[:, 2:4] - v[:, 2:4].mean(0), ek - ek.mean(0)
    U, s, Wt = np.linalg.svd(P.T @ Q)
    R = (U @ Wt).T
    det = np.linalg.det(R)
    rot = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    scale = s.sum() / (P ** 2).sum()
    extent = np.ptp(ek, axis=0)
    small = max(extent) < 15
    status = 'FAIL' if det < 0 or abs(rot) > 6 else ('WARN' if abs(rot) > 3 else 'PASS')
    if small and status == 'PASS':
        status = 'PASS (weak)'
    add('frame: rotation/mirror', status,
        f'rotation {rot:+.1f} deg, {"MIRRORED" if det < 0 else "not mirrored"}; moved only '
        f'{extent[0]:.0f} x {extent[1]:.0f} m' + (' (small: rotation poorly constrained)' if small else ''))
    add('frame: VO scale', 'WARN' if abs(scale - 1) > 0.1 else 'PASS',
        f'EKF/VISP displacement ratio {scale:.3f} (1 = exact)')
    off = (v[:, 2:4] - ek).mean(0)
    add('frame: offset', 'FAIL' if np.hypot(*off) > 5 else ('WARN' if np.hypot(*off) > 2.5 else 'PASS'),
        f'mean VISP-EKF N {off[0]:+.2f} E {off[1]:+.2f} m (origin vs home mix-up would be ~10+ m)')
    zd = v[:, 4] - np.interp(v[:, 0], K[:, 0], K[:, 3])
    add('frame: height', 'FAIL' if abs(np.median(zd)) > 3 else ('WARN' if abs(np.median(zd)) > 1 else 'PASS'),
        f'VISP down - EKF down {np.median(zd):+.2f} m median (sign or datum error would be large)')
    yd = (v[:, 5] - np.interp(v[:, 0], A[:, 0], A[:, 1]) + 180) % 360 - 180
    add('frame: heading', 'FAIL' if abs(np.median(yd)) > 5 else ('WARN' if abs(np.median(yd)) > 2 else 'PASS'),
        f'VISP yaw - ATT yaw {np.median(yd):+.2f} +/- {np.std(yd):.2f} deg')

    # 5. accuracy
    err = np.hypot(*(v[:, 2:4] - ek).T)
    rmse = math.sqrt((err ** 2).mean())
    q = len(err) // 4
    add('accuracy vs EKF', 'FAIL' if rmse > 5 else ('WARN' if rmse > 3 else 'PASS'),
        f'RMSE {rmse:.2f} m, p95 {np.percentile(err, 95):.2f}, max {err.max():.2f}; '
        f'first/last quarter {err[:q].mean():.2f} / {err[-q:].mean():.2f} m')

    # 6. lag: high-pass (4 s) position cross-correlation, insensitive to slow offsets
    tg = np.arange(t0 + 5, t1 - 5, 0.02)

    def hp(t, y):
        yi = np.interp(tg, t, y)
        return yi - np.convolve(yi, np.ones(200) / 200, 'same')
    ours = np.r_[hp(v[:, 0], v[:, 2]), hp(v[:, 0], v[:, 3])]
    r, lag = max((np.corrcoef(ours, np.r_[hp(K[:, 0] + L, K[:, 1]), hp(K[:, 0] + L, K[:, 2])])[0, 1], L)
                 for L in np.arange(-0.2, 1.2, 0.01))
    lag_ms = lag * 1e3
    if r < 0.7:
        st = 'INCONCLUSIVE'
    else:
        st = 'PASS' if lag_ms <= VISO_DELAY_MAX_MS + 30 else ('WARN' if lag_ms <= 350 else 'FAIL')
    add('lag behind EKF', st, f'{lag_ms:.0f} ms (r={r:.2f}); VISO_DELAY_MS can absorb up to {VISO_DELAY_MAX_MS} ms'
        + ('; fly longer straight legs for a firm number' if r < 0.7 else ''))

    # 7. EKF and node health
    src = sorted(S)
    add('EKF source set', 'PASS' if src in ([0], []) else 'WARN', f'source sets used: {src} (0 = GPS set)')
    bad = [s for _, s in msgs if any(k in s for k in ('EKF failsafe', 'lane switch', 'GPS Glitch', 'VisOdom: not healthy',
                                                    'GPU failed', 'NGPS error', 'fusion error', 'ALSO published'))]
    add('FC/node warnings', 'WARN' if bad else 'PASS', '; '.join(sorted(set(bad)))[:300] or 'none')
    ngps = [s for _, s in msgs if 'NGPS' in s]
    print('  NGPS/VO messages: ' + ('; '.join(dict.fromkeys(ngps)) or 'none'))
    return verdict(res)


def verdict(res):
    worst = {s for _, s, _ in res}
    fails = [n for n, s, _ in res if s == 'FAIL']
    weak = [n for n, s, _ in res if s in ('INCONCLUSIVE', 'PASS (weak)')]
    warns = [n for n, s, _ in res if s == 'WARN']
    print()
    if fails:
        print(f'VERDICT: NOT ready for ExtNav. Fix: {", ".join(fails)}.')
        code = 2
    elif weak:
        print(f'VERDICT: no faults, but not conclusive ({", ".join(weak)}): fly longer straight legs and re-check.')
        code = 1
    elif warns:
        print(f'VERDICT: usable for a first ExtNav HOVER test with GPS one switch away; review: {", ".join(warns)}.')
        code = 0
    else:
        print('VERDICT: all checks pass: ready for a first ExtNav HOVER test with GPS one switch away.')
        code = 0
    print('(The log can show the data is right and timely; only a supervised hover shows how the EKF flies on it.)')
    return code


if __name__ == '__main__':
    sys.exit(main())
