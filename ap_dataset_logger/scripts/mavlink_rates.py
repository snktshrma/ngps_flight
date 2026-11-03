#!/usr/bin/env python3
"""Ask the flight controller, over MAVLink, to stream what the dataset needs.

ArduPilot sends each port only what that port's SRx_* parameters ask for, and on a
telemetry port the defaults are a few Hz -- too slow for a useful dataset. Rather than
changing flight-controller parameters the night before a flight, this sends
MAV_CMD_SET_MESSAGE_INTERVAL through MAVROS. ArduPilot applies it to the link the request
arrived on, so nothing about the vehicle's saved configuration changes.

The requests are repeated after every reconnect, because a rebooted flight controller
forgets them.
"""

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# MAVLink message id -> (name, Hz). What each one is for in the dataset.
#
# IMU at 200 Hz: measured on the CubeOrange+ over its PPP/Ethernet link, ArduPilot
# delivers the full 200 Hz (/mavros/imu/data 200.7 Hz, data_raw 200.3 Hz). That is more
# than DDS managed (~120 Hz) and 8x the dataflash log's IMU rate unless LOG_BITMASK has
# IMU_FAST set, so this is the best VIO IMU source available without touching the FC.
# Each message is ~40 bytes, so even a 921600-baud serial link carries it comfortably.
# The `imu_hz` parameter lowers it if a link struggles.
IMU_IDS = (30, 27, 26)
RATES = {
    30:  ('ATTITUDE',                       200.0),  # /mavros/imu/data orientation
    27:  ('RAW_IMU',                        200.0),  # /mavros/imu/data_raw
    26:  ('SCALED_IMU',                     200.0),  # /mavros/imu/data accel + gyro
    32:  ('LOCAL_POSITION_NED',              30.0),  # /mavros/local_position/{pose,velocity_local}
    33:  ('GLOBAL_POSITION_INT',             30.0),  # /mavros/global_position/global (EKF)
    24:  ('GPS_RAW_INT',                     10.0),  # /mavros/global_position/raw/fix (GPS truth)
    2:   ('SYSTEM_TIME',                     10.0),  # /mavros/time_reference: aligns with dataflash
    49:  ('GPS_GLOBAL_ORIGIN',                1.0),  # /mavros/global_position/gp_origin (EKF origin)
    # Where the camera is pointing, per frame. The gimbal is a MAVLink device
    # (MNT1_TYPE=6); ArduPilot relays its attitude on request. The legacy MOUNT_STATUS and
    # MOUNT_ORIENTATION messages are rejected by this firmware, so only the v2 one is used.
    285: ('GIMBAL_DEVICE_ATTITUDE_STATUS',   30.0),  # /mavros/gimbal_control/device/attitude_status
}


MAX_ATTEMPTS = 3


class MavlinkRates(Node):
    def __init__(self):
        super().__init__('mavlink_rates')
        self.declare_parameter('mavros_ns', '/mavros')
        self.declare_parameter('imu_hz', 200.0)
        ns = self.get_parameter('mavros_ns').get_parameter_value().string_value.rstrip('/')
        imu_hz = float(self.get_parameter('imu_hz').get_parameter_value().double_value)
        self.rates = {i: (n, imu_hz if i in IMU_IDS else hz) for i, (n, hz) in RATES.items()}
        self.client = self.create_client(MessageInterval, f'{ns}/set_message_interval')
        self.create_subscription(
            State, f'{ns}/state', self._on_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.connected = False
        self.accepted = set()
        self.attempts = {}
        self.pending = {}
        self.create_timer(1.0, self._tick)

    def _on_state(self, msg: State):
        if msg.connected and not self.connected:
            self.get_logger().info('flight controller connected; requesting stream rates')
        if not msg.connected and self.connected:
            # A reconnect usually means the flight controller rebooted, and it forgets
            # every requested interval when it does.
            self.get_logger().warn('flight controller link lost; will re-request on reconnect')
            self.accepted.clear()
            self.attempts.clear()
        self.connected = msg.connected

    def _tick(self):
        if not self.connected or self.pending:
            return
        todo = [i for i in self.rates
                if i not in self.accepted and self.attempts.get(i, 0) < MAX_ATTEMPTS]
        if not todo:
            return
        if not self.client.service_is_ready():
            self.get_logger().info('waiting for MAVROS set_message_interval service...')
            return
        for msg_id in todo:
            self.attempts[msg_id] = self.attempts.get(msg_id, 0) + 1
            fut = self.client.call_async(
                MessageInterval.Request(message_id=msg_id, message_rate=self.rates[msg_id][1]))
            self.pending[msg_id] = fut
            fut.add_done_callback(lambda f, i=msg_id: self._done(i, f))

    def _done(self, msg_id, fut):
        name, hz = self.rates[msg_id]
        self.pending.pop(msg_id, None)
        try:
            ok = fut.result().success
        except Exception as exc:  # service died mid-call, e.g. MAVROS restarting
            ok = False
            self.get_logger().warn(f'{name}: request failed ({exc})')
        if ok:
            self.accepted.add(msg_id)
            self.get_logger().info(f'{name:<20} {hz:5.1f} Hz  accepted')
        elif self.attempts.get(msg_id, 0) >= MAX_ATTEMPTS:
            self.get_logger().error(
                f'{name:<20} {hz:5.1f} Hz  rejected {MAX_ATTEMPTS} times -- giving up; '
                'that topic will run at the port\'s default rate')
        else:
            self.get_logger().warn(f'{name:<20} {hz:5.1f} Hz  rejected -- retrying')


def main():
    rclpy.init()
    node = MavlinkRates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
