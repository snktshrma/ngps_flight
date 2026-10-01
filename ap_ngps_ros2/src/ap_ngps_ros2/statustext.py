"""GCS status messages over ArduPilot's DDS statustext topic.

ArduPilot forwards ``/ap/statustext`` to the GCS as MAVLink STATUSTEXT, prefixed
with ``DDS: ``. The GCS statustext queue is only 10-30 deep and shared with
ArduPilot's own messages, so this is strictly a low-rate channel: every message
is rate limited and repeats are suppressed. Callers should report *state
changes* (started, first fix, fix lost, fix regained), never per-frame detail.

The message type ships with newer ardupilot_msgs. When it is missing the
reporter disables itself and node logging continues unchanged.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Optional

TOPIC_DEFAULT = '/ap/statustext'

# MAV_SEVERITY. Note 0 is EMERGENCY, so severity is always set explicitly.
EMERGENCY = 0
ALERT = 1
CRITICAL = 2
ERROR = 3
WARNING = 4
NOTICE = 5
INFO = 6
DEBUG = 7

# ArduPilot drops anything over 255 bytes and the GCS buffer truncates at 251
# once "DDS: " is prepended. Our own "NGPS: " tag costs another 6.
_MAX_TEXT = 251 - len('NGPS: ')


class StatusTextReporter:
    """Publishes short status lines to the GCS, rate limited and de-duplicated."""

    def __init__(
        self,
        node,
        *,
        enabled: bool = True,
        topic: str = TOPIC_DEFAULT,
        min_interval_s: float = 2.0,
        repeat_suppress_s: float = 30.0,
        queue_depth: int = 8,
    ) -> None:
        self._node = node
        self._log = node.get_logger()
        self._min_interval_s = max(0.0, float(min_interval_s))
        self._repeat_suppress_s = max(0.0, float(repeat_suppress_s))
        self._queue: deque = deque(maxlen=max(1, int(queue_depth)))
        self._last_sent_s: dict[str, float] = {}
        self._last_publish_s = 0.0
        self._pub = None
        self._msg_cls = None
        self._warned_no_subscriber = False

        if not enabled:
            self._log.info('statustext: disabled by parameter')
            return
        try:
            from ardupilot_msgs.msg import StatusText  # type: ignore
        except ImportError:
            self._log.info(
                'statustext: ardupilot_msgs/StatusText not available in this build; '
                'GCS status messages disabled (node logging unaffected)'
            )
            return
        self._msg_cls = StatusText
        # Default QoS (RELIABLE, VOLATILE, KEEP_LAST): ArduPilot's subscriber is
        # RELIABLE, so a BEST_EFFORT publisher would never match it.
        self._pub = node.create_publisher(StatusText, topic, 10)
        self._log.info(f'statustext: publishing GCS messages on {topic}')

    @property
    def enabled(self) -> bool:
        return self._pub is not None

    def send(
        self,
        text: str,
        severity: int = INFO,
        *,
        key: Optional[str] = None,
        repeat_suppress_s: Optional[float] = None,
    ) -> None:
        """Queue one status line.

        ``key`` groups messages whose text varies but which mean the same thing
        (e.g. "no fix for 8s"), so repeats are suppressed as one. Messages are
        logged through the node logger whether or not DDS is available.
        """
        text = (text or '').strip()
        if not text:
            return  # ArduPilot silently drops empty text
        if len(text) > _MAX_TEXT:
            text = text[:_MAX_TEXT - 1] + '~'
        severity = int(severity)
        if severity < EMERGENCY or severity > DEBUG:
            severity = INFO

        if severity <= ERROR:
            self._log.error(f'statustext: {text}')
        elif severity == WARNING:
            self._log.warning(f'statustext: {text}')
        else:
            self._log.info(f'statustext: {text}')

        if self._pub is None:
            return

        now = time.monotonic()
        dedupe_key = key or text
        suppress = self._repeat_suppress_s if repeat_suppress_s is None else float(repeat_suppress_s)
        last = self._last_sent_s.get(dedupe_key)
        if last is not None and (now - last) < suppress:
            return
        self._last_sent_s[dedupe_key] = now
        # A full queue drops the oldest: during a burst the newest state is the
        # one worth showing.
        self._queue.append((severity, f'NGPS: {text}'))

    def spin_once(self) -> None:
        """Publish at most one queued message. Call from a ~1 Hz timer."""
        if self._pub is None or not self._queue:
            return
        now = time.monotonic()
        if (now - self._last_publish_s) < self._min_interval_s:
            return
        # Hold messages until ArduPilot's subscriber has been discovered,
        # otherwise the first ones are published into the void.
        if self._pub.get_subscription_count() < 1:
            if not self._warned_no_subscriber:
                self._warned_no_subscriber = True
                self._log.info(
                    'statustext: no subscriber yet (is ArduPilot DDS enabled?); '
                    'holding messages until discovery completes'
                )
            return
        severity, text = self._queue.popleft()
        msg = self._msg_cls()
        msg.severity = severity
        msg.text = text
        self._pub.publish(msg)
        self._last_publish_s = now
