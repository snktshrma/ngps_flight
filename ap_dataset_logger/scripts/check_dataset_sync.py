#!/usr/bin/env python3
"""Verify a recorded flight dataset before you rely on it.

    ros2 run ap_dataset_logger check_dataset_sync.py <bag_dir>

Answers the questions that decide whether a dataset is usable for an offline NGPS run:

* did the camera actually deliver a steady 30 fps, or are there gaps where frames were
  dropped mid-flight?
* are the camera timestamps smooth (RTCP-synced) or jittery (arrival-stamped)?
* do the camera and the pose/IMU topics overlap in time, and by how much?
* is any topic missing entirely, which usually means it was never being published?

Reads the bag through rosbag2_py, so it works for mcap and sqlite3 alike.
"""

import argparse
import glob
import os
import sys
from collections import defaultdict


def percentile(values, pct):
    if not values:
        return float('nan')
    s = sorted(values)
    if len(s) == 1:
        return float(s[0])
    pos = (len(s) - 1) * pct / 100.0
    lo, hi = int(pos), min(int(pos) + 1, len(s) - 1)
    return float(s[lo] + (s[hi] - s[lo]) * (pos - lo))


def open_reader(path):
    """Open a bag, tolerating a missing metadata.yaml.

    A recording cut short -- power loss at the end of a flight, or a recorder killed
    before it finalised -- leaves the .mcap file complete but with no metadata.yaml,
    and rosbag2 then refuses to guess. mcap is self-describing, so pointing the reader
    straight at the file recovers the data. ``ros2 bag reindex <dir> -s mcap``
    regenerates the metadata permanently if you would rather fix the bag itself.
    """
    import rosbag2_py

    def _try(uri, storage_id):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
                    rosbag2_py.ConverterOptions('', ''))
        return reader

    if os.path.isdir(path) and not os.path.exists(os.path.join(path, 'metadata.yaml')):
        files = sorted(glob.glob(os.path.join(path, '*.mcap')))
        if files:
            if len(files) > 1:
                print(f'note: {path} has no metadata.yaml and {len(files)} split files; '
                      f'reading only {os.path.basename(files[0])}.\n'
                      f'      Run: ros2 bag reindex {path} -s mcap', file=sys.stderr)
            else:
                print(f'note: no metadata.yaml; reading {os.path.basename(files[0])} '
                      f'directly (ros2 bag reindex {path} -s mcap fixes this)',
                      file=sys.stderr)
            return _try(files[0], 'mcap')
    try:
        return _try(path, '')
    except Exception:
        if path.endswith('.mcap'):
            return _try(path, 'mcap')
        raise


def read_bag(path):
    try:
        reader = open_reader(path)
    except Exception as exc:
        print(f'cannot open bag {path!r}: {exc}', file=sys.stderr)
        raise SystemExit(1)

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    # Bag-write time per topic; enough for rate/gap analysis without deserialising
    # every message, which matters for a multi-GB flight bag.
    stamps = defaultdict(list)
    while reader.has_next():
        topic, _data, t_ns = reader.read_next()
        stamps[topic].append(t_ns / 1e9)
    return types, stamps


def header_stamps(path, topic):
    """Deserialise one topic's header stamps -- the sync-relevant clock."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_reader(path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        return []
    msg_type = get_message(types[topic])
    try:
        sf = rosbag2_py.StorageFilter(topics=[topic])
        reader.set_filter(sf)
    except Exception:
        pass
    out = []
    while reader.has_next():
        name, data, _t = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        hdr = getattr(msg, 'header', None)
        if hdr is not None:
            out.append(hdr.stamp.sec + hdr.stamp.nanosec * 1e-9)
    return out


def describe(label, ts, expect_hz=None):
    if len(ts) < 3:
        print(f'  {label:<34} {len(ts)} msgs -- too few to analyse')
        return
    ts = sorted(ts)
    span = ts[-1] - ts[0]
    gaps = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    hz = (len(ts) - 1) / span if span > 0 else float('nan')
    p50 = percentile(gaps, 50) * 1e3
    p95 = percentile(gaps, 95) * 1e3
    worst = max(gaps) * 1e3
    note = ''
    if expect_hz:
        nominal = 1000.0 / expect_hz
        # Deliberately not called "missing frames": these are bag-write times, and a
        # burst of frames arriving together inflates p95 without a single frame being
        # lost. Only the capture clock can distinguish the two, so loss is reported in
        # the timestamp-quality section instead.
        if p95 > nominal * 1.5:
            note = f'  <-- arrival bursty (p95 {p95 / nominal:.1f}x nominal)'
    print(f'  {label:<34} {len(ts):>7} msgs  {hz:>6.2f} Hz  '
          f'gap p50={p50:>6.2f} p95={p95:>7.2f} max={worst:>8.2f} ms{note}')
    return {'span': span, 'first': ts[0], 'last': ts[-1], 'p50': p50, 'p95': p95}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--camera-topic', default='/camera/image_raw/compressed')
    ap.add_argument('--expect-fps', type=float, default=30.0)
    args = ap.parse_args()

    types, stamps = read_bag(args.bag)
    if not stamps:
        print('bag contains no messages', file=sys.stderr)
        return 1

    print(f'== topics ({len(stamps)}) ==')
    for topic in sorted(stamps):
        describe(topic, stamps[topic],
                 expect_hz=args.expect_fps if topic == args.camera_topic else None)

    cam = args.camera_topic
    if cam not in stamps:
        print(f'\nWARNING: camera topic {cam} not in this bag', file=sys.stderr)
        return 1

    print('\n== camera timestamp quality ==')
    print('   (bag-write time is when the recorder saw it; header stamp is when the')
    print('    camera captured it. A smooth header with a jittery arrival is exactly')
    print('    what RTCP sync is supposed to produce.)')
    hdr = header_stamps(args.bag, cam)
    arr = sorted(stamps[cam])
    if len(hdr) >= 3:
        hdr = sorted(hdr)
        hgaps = [hdr[i + 1] - hdr[i] for i in range(len(hdr) - 1)]
        agaps = [arr[i + 1] - arr[i] for i in range(len(arr) - 1)]
        nominal = 1000.0 / args.expect_fps
        print(f'  header stamp gaps  p50={percentile(hgaps,50)*1e3:6.2f}  '
              f'p95={percentile(hgaps,95)*1e3:6.2f}  max={max(hgaps)*1e3:7.2f} ms')
        print(f'  bag arrival gaps   p50={percentile(agaps,50)*1e3:6.2f}  '
              f'p95={percentile(agaps,95)*1e3:6.2f}  max={max(agaps)*1e3:7.2f} ms')
        # A gap in the capture clock is genuine loss: that clock ticks once per frame
        # whether or not the frame reaches us.
        dropped = sum(max(0, int(round(g * 1e3 / nominal)) - 1) for g in hgaps)
        if dropped:
            worst = max(hgaps) * 1e3
            print(f'  frames lost upstream: {dropped} '
                  f'({dropped / (len(hdr) + dropped):.2%} of expected), '
                  f'largest single gap {worst:.0f} ms')
        else:
            print('  frames lost upstream: none')

        hp95 = percentile(hgaps, 95) * 1e3
        if hp95 < nominal * 1.15:
            print(f'  -> header stamps are smooth (p95 {hp95:.2f} ms vs '
                  f'{nominal:.2f} ms nominal): sync looks good')
        else:
            print(f'  -> header p95 {hp95:.2f} ms exceeds nominal {nominal:.2f} ms: '
                  'either frames were dropped or RTCP sync was unavailable')
        lag = [a - h for a, h in zip(arr, hdr)]
        print(f'  arrival minus capture: p50={percentile(lag,50)*1e3:.1f} ms  '
              f'p95={percentile(lag,95)*1e3:.1f} ms   (pipeline + record latency)')

    print('\n== cross-topic coverage ==')
    cam_first, cam_last = arr[0], arr[-1]
    for topic in sorted(stamps):
        if topic == cam or len(stamps[topic]) < 2:
            continue
        ts = sorted(stamps[topic])
        overlap = min(cam_last, ts[-1]) - max(cam_first, ts[0])
        cam_span = cam_last - cam_first
        pct = 100.0 * overlap / cam_span if cam_span > 0 else 0.0
        flag = '' if pct > 95 else '   <-- partial coverage'
        print(f'  {topic:<34} covers {max(0.0, pct):5.1f}% of the camera span{flag}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
