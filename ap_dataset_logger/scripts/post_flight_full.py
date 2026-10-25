#!/usr/bin/env python3
"""Everything between the GPS survey flight and the showcase, in parallel, with one summary at the end.

    python3 ap_dataset_logger/scripts/post_flight_full.py --name flight11 --site ottawa1 [--bag DIR] [--bin FILE]

  A. FC dataflash log download (MAVFTP, the slow part: ~7-11 min)  ->  fc_log_check.py verdict
     -> EKF3 Replay twice, in parallel: as flown (replayed EKF must match the flown one) and with vision
        as the only position source (how the EKF would fly on our /ap/tf alone).
  B. meanwhile, on the GPU (one job at a time): map_compare.py on the bag (<site>, <site>_city, <site>_z19)
     -> make_mosaic.py over <site> with its measured offset -> <site>_today
     -> hold-out check: a mosaic from the first part of the flight, matched with the last part.
Do NOT reboot the FC until this finishes (a reboot kills the log download). Output: bags/<name>_post_full.log
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import subprocess
import sys
import threading
import time

R = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
WS = os.path.normpath(os.path.join(R, '..', '..'))
BAGS = os.path.join(R, 'bags')
REPLAY = os.path.join(WS, 'src', 'ardupilot', 'build', 'sitl', 'tool', 'Replay')
SRC = ('source /opt/ros/jazzy/setup.bash; source {ws}/install/setup.bash; source {ws}/install-jazzy/setup.bash; '
       'cd {r} && ').format(ws=WS, r=R)
T0 = time.time()
LOG = None
lock = threading.Lock()


def say(msg):
    line = f'[{time.time() - T0:6.0f} s] {msg}'
    with lock:
        print(line, flush=True)
        LOG.write(line + '\n')
        LOG.flush()


def run(tag, cmd, cwd=None, container=False):
    """Run a command, stream its output into the log with a tag; returns (rc, output)."""
    if container:   # as the login user BY NAME (keeps the GPU groups; ramguard can stop it)
        cmd = ['docker', 'exec', '-u', os.environ.get('USER', 'rmackay9'), '-e', f'HOME={os.path.expanduser("~")}',
               'vps-jp7', 'bash', '-c', SRC + cmd]
    say(f'{tag}: start')
    t = time.time()
    p = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = []
    for line in p.stdout:
        if 'TRT] [W]' in line or 'PVA device' in line:
            continue
        out.append(line.rstrip())
        with lock:
            LOG.write(f'    {tag}| {line}')
    p.wait()
    say(f'{tag}: done rc={p.returncode} in {time.time() - t:.0f} s')
    return p.returncode, out


def newest_bag():
    for d in sorted(glob.glob(os.path.join(BAGS, '*/')), key=os.path.getmtime, reverse=True):
        if os.path.isfile(os.path.join(d, 'metadata.yaml')) and glob.glob(os.path.join(d, '*.mcap')):
            return d.rstrip('/')
    sys.exit('no closed bag found')


def gpu_chain(a, bag, res):
    maps = [m for m in (a.site, f'{a.site}_city', f'{a.site}_z19') if os.path.isdir(os.path.join(R, 'ngps_config', 'tiff', m))]
    rc, out = run('map_compare', f'python3 ngps_config/tools/map_compare.py --bag {bag} --maps {" ".join(maps)} '
                  f'--period 1.0 --min-alt 30', container=True)
    res['map_compare'] = [l for l in out if 'matched' in l or 'offset_en_m' in l or 'by height' in l]
    try:
        j = json.load(open(bag + '_map_compare.json'))
        base = j['maps'].get(a.site, {})
        off = base.get('offset_en_m', [0.0, 0.0])
    except Exception as exc:  # noqa: BLE001
        say(f'map_compare json unreadable ({exc}); mosaic base offset 0')
        off = [0.0, 0.0]
    res['base_offset'] = off
    if os.path.exists(bag + '_map_compare.json'):   # the hold-out run below writes the same file name
        os.replace(bag + '_map_compare.json', bag + '_map_compare_main.json')
    rc, out = run('make_mosaic', f'python3 ngps_config/tools/make_mosaic.py --bag {bag} --base {a.site} '
                  f'--name {a.site}_today --base-offset {off[0]} {off[1]} --min-alt 40', container=True)
    res['mosaic'] = [l for l in out if 'frames to paste' in l or 'pasted' in l or 'wrote' in l]
    # the showcase config: the site's fusion config pointed at today's mosaic, then config_check
    src_cfg = os.path.join(R, 'ap_ngps_ros2', 'config', f'fusion_{a.site}.yaml')
    dst_cfg = os.path.join(R, 'ap_ngps_ros2', 'config', f'fusion_{a.site}_today.yaml')
    if rc == 0 and os.path.isfile(src_cfg):
        txt = open(src_cfg).read().replace(f'/tiff/{a.site}/', f'/tiff/{a.site}_today/')
        open(dst_cfg, 'w').write(f"# today's mosaic over {a.site} (post_flight_full.py, {time.strftime('%Y-%m-%d %H:%M')})\n" + txt)
        rc2, out2 = run('config_check', f'PYTHONPATH=ap_ngps_ros2/src python3 -m ap_ngps_ros2.config_check {dst_cfg}', container=True)
        res['mosaic'].append(f'showcase config {dst_cfg}: ' + ('OK' if rc2 == 0 else 'CHECK FAILED'))
    # hold-out: mosaic from the first 60 % of the airborne time, matched with the last 40 %
    rc, out = run('airborne', 'python3 - <<\'EOF\'\nimport rosbag2_py,numpy as np\nfrom rclpy.serialization import '
                  'deserialize_message\nfrom geographic_msgs.msg import GeoPoseStamped\n'
                  f'r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri="{bag}",storage_id="mcap"),'
                  'rosbag2_py.ConverterOptions("",""))\nr.set_filter(rosbag2_py.StorageFilter(topics=["/ap/geopose/filtered"]))\n'
                  'g=[]\nwhile r.has_next():\n    _,d,_=r.read_next(); m=deserialize_message(d,GeoPoseStamped); '
                  'g.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.position.altitude))\n'
                  'g=np.array(g); g=g[np.abs(g[:,1])>1]; t0=g[0,0]; base=np.percentile(g[:200,1],50)\n'
                  'air=g[g[:,1]-base>40,0]; print("AIR",air[0]-t0,air[-1]-t0)\nEOF', container=True)
    air = next((l.split()[1:] for l in out if l.startswith('AIR')), None)
    if air:
        a0, a1 = map(float, air)
        cut = a0 + 0.6 * (a1 - a0)
        run('holdout_mosaic', f'python3 ngps_config/tools/make_mosaic.py --bag {bag} --base {a.site} --name {a.site}_h1 '
            f'--base-offset {off[0]} {off[1]} --min-alt 40 --t-end {cut:.1f}', container=True)
        rc, out = run('holdout_compare', f'python3 ngps_config/tools/map_compare.py --bag {bag} --maps {a.site} {a.site}_h1 '
                      f'--period 1.0 --min-alt 30 --t-start {cut:.1f}', container=True)
        res['holdout'] = [l for l in out if 'matched' in l or 'offset_en_m' in l]
        # map_compare overwrote <bag>_map_compare.json with the hold-out run: keep both
        if os.path.exists(bag + '_map_compare.json'):
            os.replace(bag + '_map_compare.json', bag + '_map_compare_holdout.json')
        subprocess.run(['rm', '-rf', os.path.join(R, 'ngps_config', 'tiff', f'{a.site}_h1')])
    else:
        res['holdout'] = ['could not find the airborne window']


def fc_chain(a, res):
    if a.bin:
        path = os.path.abspath(a.bin)   # Replay runs in its own folder
    else:
        rc, out = run('fc_log_download+check', [sys.executable, os.path.join(R, 'ap_dataset_logger', 'scripts', 'fc_log_check.py'),
                                                '--download', '--name', a.name])
        res['fc_log'] = [l for l in out if any(k in l for k in ('PASS', 'FAIL', 'WARN', 'VERDICT', 'airborne', 'MB in'))]
        path = next((os.path.abspath(l.strip()) for l in out if l.strip().endswith('.BIN') and os.path.isfile(l.strip())), None)
        if not path:
            res['replay'] = ['no BIN downloaded: replay skipped']
            return
    if a.bin:
        rc, out = run('fc_log_check', [sys.executable, os.path.join(R, 'ap_dataset_logger', 'scripts', 'fc_log_check.py'), path])
        res['fc_log'] = [l for l in out if any(k in l for k in ('PASS', 'FAIL', 'WARN', 'VERDICT', 'airborne'))]
    work = os.path.join(BAGS, f'{a.name}_ekf_replay')
    jobs = {'asflown': [], 'vision': ['--parm', 'EK3_SRC1_POSXY=6', '--parm', 'EK3_SRC1_VELXY=0', '--parm', 'EK3_SRC1_VELZ=0']}
    th = []
    for k, extra in jobs.items():
        os.makedirs(os.path.join(work, k), exist_ok=True)
        th.append(threading.Thread(target=run, args=(f'replay_{k}', [REPLAY] + extra + [path]), kwargs={'cwd': os.path.join(work, k)}))
        th[-1].start()
    for t in th:
        t.join()
    res['replay'] = compare_replays(work)


def compare_replays(work):
    import numpy as np
    from pymavlink import mavutil
    out = []
    for k in ('asflown', 'vision'):
        f = os.path.join(work, k, 'logs', '00000001.BIN')
        if not os.path.isfile(f):
            out.append(f'{k}: no replay output')
            continue
        m = mavutil.mavlink_connection(f)
        K, C = {}, []
        while True:
            x = m.recv_match(type=['XKF1', 'CTUN'])
            if x is None:
                break
            if x.get_type() == 'XKF1':
                K.setdefault(x.C, []).append((x.TimeUS / 1e6, x.PN, x.PE))
            else:
                C.append((x.TimeUS / 1e6, x.Alt))
        C = np.array(C)
        air = C[C[:, 1] > 5, 0]
        o, r = np.array(K.get(0, [])), np.array(K.get(100, []))
        if not len(air) or not len(o) or not len(r):
            out.append(f'{k}: missing cores/airborne data')
            continue
        s = (o[:, 0] >= air[0]) & (o[:, 0] <= air[-1])
        o = o[s]
        d = np.hypot(np.interp(o[:, 0], r[:, 0], r[:, 1]) - o[:, 1], np.interp(o[:, 0], r[:, 0], r[:, 2]) - o[:, 2])
        out.append(f'{k}: replayed EKF vs flown EKF over {air[-1] - air[0]:.0f} s airborne: median {np.median(d):.2f} m, '
                   f'p95 {np.percentile(d, 95):.2f} m, max {d.max():.2f} m')
    return out


def main():
    global LOG
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name', required=True)
    ap.add_argument('--site', default='ottawa1')
    ap.add_argument('--bag', help='default: the newest closed bag')
    ap.add_argument('--bin', help='use this FC log instead of downloading the newest one')
    a = ap.parse_args()
    bag = os.path.abspath(a.bag) if a.bag else newest_bag()
    LOG = open(os.path.join(BAGS, f'{a.name}_post_full.log'), 'w')
    say(f'bag {bag}; site {a.site}; FC log {"given: " + a.bin if a.bin else "download newest"}')
    subprocess.run(['docker', 'start', 'vps-jp7'], stdout=subprocess.DEVNULL)
    res = {}
    t1 = threading.Thread(target=gpu_chain, args=(a, bag, res))
    t2 = threading.Thread(target=fc_chain, args=(a, res))
    t1.start(); t2.start(); t1.join(); t2.join()
    say('==================== SUMMARY ====================')
    for k in ('map_compare', 'base_offset', 'mosaic', 'holdout', 'fc_log', 'replay'):
        v = res.get(k)
        say(f'-- {k}:')
        for line in (v if isinstance(v, list) else [v]):
            say(f'   {line}')
    say(f'total {time.time() - T0:.0f} s. Now safe to reboot the FC.')


if __name__ == '__main__':
    main()
