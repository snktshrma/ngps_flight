#!/usr/bin/env python3
"""RAM guard for the Jetson (no swap), flight-aware: free memory before the machine locks up, without
taking the flight stack down for someone else's leak.

Above a threshold it stops one process, chosen by tier:
  >= --threshold (90 %)        the largest NON-flight process (analysis, map/mosaic builds, editors' helpers...)
  >= --recorder-threshold (95) if no non-flight process is left: the bag recorder (data lost, flight unaffected)
  >= --flight-threshold (97)   last resort: the largest remaining flight process. If that is the fusion node,
                               ArduPilot sees VisOdom stop and srcfs_gps_fallback.lua puts the EKF back on GPS
                               within 0.5 s -- still better than a frozen Jetson (no DDS, no /ap/tf, no recorder).
Never stopped: the micro-ROS agent (the FC's DDS link), and the protected session/system list below.
It runs as the login user, so it can only ever stop that user's processes (root/gdm are out of reach).

    ramguard.py                          # run forever: 90 / 95 / 97 %
    ramguard.py --once --dry-run         # show the tiers and what it would stop right now
Log: ~/ramguard.log. Install (user service): system/ramguard.service.
"""
import argparse
import os
import signal
import time

PROTECT = ('cursor-server', 'vscode-server', '.cursor', 'claude', 'sshd', 'systemd', 'dbus',
           'gnome', 'Xorg', 'Xwayland', 'pipewire', 'wireplumber', 'pulseaudio', 'screen',
           'tmux', 'docker', 'containerd', 'conmon', 'distrobox', 'ramguard', 'ibus', 'gvfs',
           'at-spi', 'xdg-', 'evolution', 'gsd-', 'tracker-miner', 'sd-pam', 'polkit',
           'gpg-agent', 'ssh-agent', 'keyring', 'agetty',
           'micro_ros_agent')                       # the FC's DDS link: never
# Flight stack, by command-line substring.
RECORDER = ('ros2 bag record', 'bag record', 'rosbag2_transport', 'rosbag2_recorder')
FLIGHT = ('vo_ngps_fusion_node', 'ngps_localization_node', 'rtsp_camera_node', 'run_fusion.sh',
          'flight_log.sh', 'flight_log_inner.sh', 'mavlink_rates')
# Processes started with plain `docker exec` run as root and are out of this guard's reach: start analysis
# jobs with `docker exec -u "$(id -un)"` (post_gps_flight.sh does; by name, not uid:gid, or the GPU groups are lost)
# so they stay killable.
# Killing something small cannot rescue a full machine and is more likely to break a
# session helper than to help, so below this the guard only logs.
MIN_RSS_MB = 100
SHELLS = {'bash', 'sh', 'zsh', 'dash', 'fish', 'login', 'sudo', 'su'}
LOG = os.path.expanduser('~/ramguard.log')
ME = os.getuid()


def log(msg):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {msg}"
    print(line, flush=True)
    with open(LOG, 'a') as f:
        f.write(line + '\n')


def mem_used_pct():
    m = {}
    with open('/proc/meminfo') as f:
        for line in f:
            k, v = line.split(':', 1)
            m[k] = int(v.split()[0])
    return 100.0 * (1 - m['MemAvailable'] / m['MemTotal']), m['MemAvailable'] / 1024


def tier_of(comm, cmd):
    """'protected' | 'recorder' | 'flight' | 'other'."""
    if not cmd or cmd.startswith('(') or comm in SHELLS or any(s in cmd or s in comm for s in PROTECT):
        return 'protected'
    if any(s in cmd for s in RECORDER):
        return 'recorder'
    if any(s in cmd for s in FLIGHT):
        return 'flight'
    return 'other'


def processes():
    out = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        p = int(pid)
        if p == os.getpid():
            continue
        try:
            if os.stat(f'/proc/{pid}').st_uid != ME:
                continue
            with open(f'/proc/{pid}/comm') as f:
                comm = f.read().strip()
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
            with open(f'/proc/{pid}/statm') as f:
                rss_mb = int(f.read().split()[1]) * os.sysconf('SC_PAGE_SIZE') / 2 ** 20
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        out.append((rss_mb, p, comm, cmd, tier_of(comm, cmd)))
    return out


def choose(used, procs, threshold, recorder_threshold, flight_threshold):
    """The process to stop at `used` % RAM, or None. procs: (rss_mb, pid, comm, cmd, tier)."""
    allowed = []
    if used >= threshold:
        allowed.append('other')
    if used >= recorder_threshold:
        allowed.append('recorder')
    if used >= flight_threshold:
        allowed.append('flight')
    for tier in allowed:                             # cheapest tier first, largest process within it
        c = sorted((p for p in procs if p[4] == tier and p[0] >= MIN_RSS_MB), reverse=True)
        if c:
            return c[0]
    return None


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--threshold', type=float, default=90.0, help='%% RAM: stop the largest non-flight process')
    ap.add_argument('--recorder-threshold', type=float, default=95.0, help='%% RAM: then the bag recorder')
    ap.add_argument('--flight-threshold', type=float, default=97.0, help='%% RAM: then the largest flight process')
    ap.add_argument('--interval', type=float, default=1.0)
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--once', action='store_true')
    a = ap.parse_args()
    th = (a.threshold, a.recorder_threshold, a.flight_threshold)
    log(f"ramguard started: other {th[0]:.0f}%, recorder {th[1]:.0f}%, flight {th[2]:.0f}%"
        f"{' (dry run)' if a.dry_run else ''}")
    warned = 0.0
    while True:
        used, avail = mem_used_pct()
        if used >= a.threshold - 5 and time.time() - warned > 30:
            log(f"warning: RAM {used:.1f}% used ({avail:.0f} MB available)")
            warned = time.time()
        if used >= a.threshold or a.once:
            procs = processes()
            if a.once:
                for tier in ('other', 'recorder', 'flight'):
                    big = sorted((p for p in procs if p[4] == tier), reverse=True)[:3]
                    log(f"  {tier:8}: " + (', '.join(f"{p[2]} {p[0]:.0f} MB" for p in big) or '-'))
            pick = choose(max(used, a.threshold) if a.once else used, procs, *th)
            if pick is None:
                log(f"RAM {used:.1f}%: nothing to stop at this level (flight processes are kept below "
                    f"{a.recorder_threshold:.0f}/{a.flight_threshold:.0f}%)")
            else:
                rss, pid, comm, cmd, tier = pick
                what = f"{tier} pid {pid} {comm} ({rss:.0f} MB): {cmd[:120]}"
                if a.dry_run or (a.once and used < a.threshold):
                    log(f"[dry run] RAM {used:.1f}% -> would stop {what}")
                else:
                    log(f"RAM {used:.1f}% -> SIGTERM {what}")
                    try:
                        os.kill(pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    for _ in range(30):
                        time.sleep(0.1)
                        if not alive(pid):
                            break
                    if alive(pid) and mem_used_pct()[0] >= a.threshold:
                        log(f"  still alive and RAM still high -> SIGKILL pid {pid}")
                        try:
                            os.kill(pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    time.sleep(2)
                    log(f"  RAM now {mem_used_pct()[0]:.1f}%")
            if a.once:
                return
        time.sleep(a.interval)


if __name__ == '__main__':
    main()
