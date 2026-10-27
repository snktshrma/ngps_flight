#!/usr/bin/env python3
"""Queueing rules for the GCS statustext reporter.

The GCS statustext queue is shallow and shared with ArduPilot's own messages, so
these rules are what keep the node from crowding it out. They are exercised
against a stub publisher: no ROS graph, and no dependency on whether this build
of ardupilot_msgs carries StatusText yet.
"""

import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from ap_ngps_ros2 import statustext as st  # noqa: E402


class _StubLogger:
    def __init__(self):
        self.lines = []

    def _add(self, msg):
        self.lines.append(msg)

    info = warning = error = debug = _add


class _StubNode:
    def __init__(self):
        self._logger = _StubLogger()

    def get_logger(self):
        return self._logger

    def create_publisher(self, *_args, **_kwargs):
        # Reached only where this build of ardupilot_msgs has StatusText (the container).
        return _StubPublisher()


class _StubPublisher:
    def __init__(self, subscribers=1):
        self.published = []
        self.subscribers = subscribers

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, msg):
        self.published.append((msg.severity, msg.text))


def _reporter(node=None, **kwargs):
    """A reporter wired to a stub publisher, bypassing ROS entirely."""
    rep = st.StatusTextReporter(node or _StubNode(), enabled=False, **kwargs)
    rep._pub = _StubPublisher()
    rep._msg_cls = lambda: types.SimpleNamespace(severity=0, text='')
    return rep


def _drain(rep, count=10):
    for _ in range(count):
        rep._last_publish_s = 0.0  # bypass the inter-message interval
        rep.spin_once()


def test_missing_message_type_disables_cleanly():
    node = _StubNode()
    rep = st.StatusTextReporter(node, enabled=True)
    rep.send('hello', st.INFO)
    rep.spin_once()
    # Whether or not this build has StatusText, sending must never raise and the
    # text must still reach the node log.
    assert any('hello' in line for line in node._logger.lines)


def test_text_is_tagged_and_severity_preserved():
    rep = _reporter()
    rep.send('first fix', st.WARNING)
    _drain(rep)
    assert rep._pub.published == [(st.WARNING, 'NGPS: first fix')]


def test_empty_text_is_dropped():
    rep = _reporter()
    rep.send('', st.INFO)
    rep.send('   ', st.INFO)
    _drain(rep)
    assert rep._pub.published == []


def test_long_text_is_truncated_within_gcs_buffer():
    rep = _reporter()
    rep.send('x' * 400, st.INFO)
    _drain(rep)
    severity, text = rep._pub.published[0]
    assert len(text) <= 251, 'must fit the GCS buffer once ArduPilot prefixes "DDS: "'
    assert text.endswith('~')


def test_repeats_are_suppressed_then_allowed_again():
    rep = _reporter(repeat_suppress_s=30.0)
    for _ in range(5):
        rep.send('position lost', st.WARNING, key='fix')
    _drain(rep)
    assert len(rep._pub.published) == 1

    rep._last_sent_s['fix'] -= 31.0  # let the suppression window lapse
    rep.send('position lost', st.WARNING, key='fix')
    _drain(rep)
    assert len(rep._pub.published) == 2


def test_key_groups_messages_whose_text_varies():
    rep = _reporter(repeat_suppress_s=30.0)
    rep.send('position lost, 5s', st.WARNING, key='fix')
    rep.send('position lost, 9s', st.WARNING, key='fix')
    _drain(rep)
    assert len(rep._pub.published) == 1


def test_rate_limit_spaces_messages_out():
    rep = _reporter(min_interval_s=2.0)
    rep.send('a', st.INFO)
    rep.send('b', st.INFO)
    rep.spin_once()
    rep.spin_once()  # too soon: the second message waits
    assert len(rep._pub.published) == 1
    rep._last_publish_s -= 3.0
    rep.spin_once()
    assert len(rep._pub.published) == 2


def test_queue_overflow_keeps_the_newest_state():
    rep = _reporter(queue_depth=3)
    for i in range(6):
        rep.send(f'msg {i}', st.INFO)
    _drain(rep)
    assert [t for _, t in rep._pub.published] == ['NGPS: msg 3', 'NGPS: msg 4', 'NGPS: msg 5']


def test_messages_wait_for_ardupilot_to_be_discovered():
    rep = _reporter()
    rep._pub.subscribers = 0
    rep.send('started', st.INFO)
    _drain(rep)
    assert rep._pub.published == [], 'nothing should be sent before discovery'
    rep._pub.subscribers = 1
    _drain(rep)
    assert rep._pub.published == [(st.INFO, 'NGPS: started')]


def test_out_of_range_severity_falls_back_to_info():
    rep = _reporter()
    rep.send('odd severity', 99)
    _drain(rep)
    assert rep._pub.published[0][0] == st.INFO


if __name__ == '__main__':
    import pytest

    raise SystemExit(pytest.main([__file__, '-v']))
