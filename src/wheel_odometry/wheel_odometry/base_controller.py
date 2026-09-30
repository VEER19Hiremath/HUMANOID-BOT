#!/usr/bin/env python3
"""Base controller: /cmd_vel -> Mega wheel speeds, Mega pulses -> /odom + TF.

The Mega (arduino/wheelodom.ino) closes the speed loop on each wheel, so a
commanded wheel speed is the real one whatever the drag. This node:

  * turns /cmd_vel into left/right wheel speeds for an arcs-only base (no
    turning on the spot, wheels never in opposite directions) and eases them
    in (acceleration limit);
  * sends them as "VL:<m/s> VR:<m/s>" at 50 Hz while moving (the Mega stops
    by itself 300 ms after the last command);
  * sends the calibrated metres-per-pulse of each wheel ("K:<l> <r>") at
    every (re)connect, from config/odometry.yaml;
  * integrates the Mega's signed pulse counts into /odom and odom->base_footprint.
    The pose comes from measured wheel motion only: no extrapolation between
    reports, no command-based guessing.
"""

import math
import time
import termios

import rclpy
import serial
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

MAX_WHEEL_MPS = 0.15     # cruise (real m/s since the floor calibration)
WHEEL_DEADBAND = 0.008   # m/s; below = wheel stopped
# Tightest arc: radius ~0.46 m, the inner wheel at ~36 % of the outer one.
# Tighter scrubs the tyres and stalls a hub; wider left no room to turn
# round in a 20 x 20 ft area (the planner uses 0.50 m).
MAX_TURN_RATE_PER_SPEED = 0.47   # w <= v / (wheel_base / 2) * this
MIN_ARC_SPEED = 0.08     # pure rotation becomes a forward arc at this speed
ACCEL = 0.10             # m/s^2 per wheel, speeding up
DECEL = 0.30             # m/s^2 per wheel, slowing down
CMD_TIMEOUT = 0.5        # s without /cmd_vel -> stop
BOOT_TIME = 1.8          # s after opening the port (the Mega may reset)
RETRY_PERIOD = 0.5       # s between reconnect attempts
STATUS_LOG_PERIOD = 1.0  # s between logged Mega status lines
MAX_PULSES_PER_REPORT = 60   # 50 ms reports; more = corrupt line


def wheel_speeds(v, w, wheel_base):
    """(left, right) wheel speeds in m/s for body speed v and turn rate w."""
    half = wheel_base / 2.0
    if abs(w) > 0.015 and abs(v) < MIN_ARC_SPEED:
        v = math.copysign(MIN_ARC_SPEED, v if v != 0.0 else 1.0)
    if abs(v) >= 0.02:
        w_max = abs(v) / half * MAX_TURN_RATE_PER_SPEED
        w = max(-w_max, min(w_max, w))
    left, right = v - half * w, v + half * w
    # Keep the ratio when limiting the faster wheel to cruise.
    peak = max(abs(left), abs(right))
    if peak > MAX_WHEEL_MPS:
        left, right = left * MAX_WHEEL_MPS / peak, right * MAX_WHEEL_MPS / peak
    if abs(left) < WHEEL_DEADBAND:
        left = 0.0
    if abs(right) < WHEEL_DEADBAND:
        right = 0.0
    if left * right < 0.0:            # never opposite directions
        left = right = left if abs(left) >= abs(right) else right
    return left, right


def ease(current, target, dt):
    """Move a wheel speed toward target within the accel/decel limits."""
    limit = (DECEL if abs(target) < abs(current) or target * current < 0 else ACCEL) * dt
    return current + max(-limit, min(limit, target - current))


def integrate(x, y, theta, dl, dr, wheel_base):
    """Pose after the wheels moved dl, dr metres (exact for an arc)."""
    ds = 0.5 * (dl + dr)
    dth = (dr - dl) / wheel_base
    if abs(dth) < 1e-6:
        return x + ds * math.cos(theta), y + ds * math.sin(theta), theta
    r = ds / dth
    return (x + r * (math.sin(theta + dth) - math.sin(theta)),
            y - r * (math.cos(theta + dth) - math.cos(theta)),
            theta + dth)


class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        self.declare_parameter('arduino_port', '/dev/arduino')
        self.declare_parameter('wheel_base', 0.4318)
        # Metres per speed pulse of the left wheel, and right/left pulse
        # length. From config/odometry.yaml (scripts/floor_calibrate.py).
        self.declare_parameter('distance_per_pulse', 0.0025)
        self.declare_parameter('right_encoder_multiplier', 1.0)
        self.wheel_base = self.get_parameter('wheel_base').value
        self.k_left = self.get_parameter('distance_per_pulse').value
        self.k_right = self.k_left * self.get_parameter('right_encoder_multiplier').value

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self.ser = None
        self.booted = False
        self.ready_at = 0.0
        self.next_attempt = 0.0
        self.rx_buf = b''

        self.target = [0.0, 0.0]      # commanded wheel speeds (m/s)
        self.wheel = [0.0, 0.0]       # eased wheel speeds sent to the Mega
        self.last_cmd = 0.0
        self.last_tick = time.monotonic()

        self.x = self.y = self.theta = 0.0
        self.counts = None            # last (left, right) signed counts
        self.v_meas = self.w_meas = 0.0
        self.last_report = None
        self.wheel_angle = [0.0, 0.0]
        self.wheel_radius = 0.0762
        self._last_status_log = 0.0

        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            f'Base controller: {self.k_left * 1000:.3f} / {self.k_right * 1000:.3f} '
            'mm per pulse (left / right)')

    # ---------------- commands ----------------
    def _on_cmd_vel(self, msg):
        self.target = list(wheel_speeds(msg.linear.x, msg.angular.z, self.wheel_base))
        self.last_cmd = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        dt = min(max(now - self.last_tick, 0.0), 0.1)
        self.last_tick = now
        if now - self.last_cmd > CMD_TIMEOUT:
            self.target = [0.0, 0.0]
        for i in (0, 1):
            self.wheel[i] = ease(self.wheel[i], self.target[i], dt)
            if abs(self.wheel[i]) < 0.002 and self.target[i] == 0.0:
                self.wheel[i] = 0.0

        self._service_link()
        if self.booted and (any(self.wheel) or now - self.last_cmd < CMD_TIMEOUT):
            self._send(f'VL:{self.wheel[0]:.3f} VR:{self.wheel[1]:.3f}')
        self._publish_joints(dt)

    # ---------------- serial link ----------------
    def _open(self, port):
        """Open the Mega without the DTR pulse that resets it."""
        link = serial.Serial()
        link.port = port
        link.baudrate = 115200
        link.timeout = 0
        link.write_timeout = 0.1
        link.dtr = False
        link.rts = False
        link.open()
        return link

    def _close(self):
        if self.ser is None:
            return
        try:
            if self.ser.is_open:
                self.ser.write(b'VL:0.000 VR:0.000\n')
                self.ser.flush()
                attr = termios.tcgetattr(self.ser.fd)
                attr[2] &= ~termios.HUPCL      # no reset pulse on close
                termios.tcsetattr(self.ser.fd, termios.TCSANOW, attr)
            self.ser.close()
        except (serial.SerialException, OSError, termios.error):
            pass
        self.ser = None
        self.booted = False

    def _lost(self, reason):
        self.get_logger().warn(f'Mega link lost ({reason}); reconnecting')
        self._close()
        self.counts = None
        self.next_attempt = time.monotonic() + RETRY_PERIOD

    def _send(self, line):
        try:
            self.ser.write((line + '\n').encode())
        except (serial.SerialException, OSError, termios.error) as e:
            self._lost(e)

    def _service_link(self):
        now = time.monotonic()
        if self.ser is None:
            if now < self.next_attempt:
                return
            self.next_attempt = now + RETRY_PERIOD
            try:
                self.ser = self._open(self.get_parameter('arduino_port').value)
            except (serial.SerialException, OSError, termios.error):
                self.ser = None
                return
            self.ready_at = now + BOOT_TIME
            self.rx_buf = b''
            self.booted = False
            self.counts = None
            self.get_logger().info('Mega port opened, waiting for it')
            return
        if now < self.ready_at:
            return
        try:
            self.rx_buf += self.ser.read(self.ser.in_waiting or 0)
        except (serial.SerialException, OSError, termios.error) as e:
            self._lost(e)
            return
        if len(self.rx_buf) > 8192:
            self.rx_buf = self.rx_buf[-1024:]
        *lines, self.rx_buf = self.rx_buf.split(b'\n')
        for raw in lines:
            self._handle(raw.decode('utf-8', errors='ignore').strip())

    def _handle(self, line):
        if not self.booted:
            if line.startswith('SAFE START READY') or line.startswith('L:'):
                self.booted = True
                self.counts = None
                self._send(f'K:{self.k_left:.6f} {self.k_right:.6f}')
                self.get_logger().info('Mega ready')
            if not line.startswith('L:'):
                return
        if line.startswith('SAFE START READY'):
            # The Mega rebooted (counts restart at 0): re-send the pulse sizes.
            self.counts = None
            self._send(f'K:{self.k_left:.6f} {self.k_right:.6f}')
            self.get_logger().warn('Mega restarted')
        elif line.startswith('L:'):
            self._on_counts(line)
        elif 'STALL HARD STOP' in line:
            self.wheel = [0.0, 0.0]
            self.get_logger().warn(
                'Mega STALL HARD STOP: a driven wheel stopped turning; wheels off '
                'for 10 s (jam, obstacle, driver fault or motor power off)')
        elif line.startswith('K '):
            self.get_logger().info(f'Mega pulse size: {line}')
        elif line.startswith('S:'):
            now = time.monotonic()
            if now - self._last_status_log >= STATUS_LOG_PERIOD:
                self._last_status_log = now
                self.get_logger().info(f'Mega drive {line}')

    # ---------------- odometry ----------------
    def _on_counts(self, line):
        try:
            left_s, right_s = line[2:].split('R:')
            counts = (int(left_s), int(right_s))
        except ValueError:
            return
        now = time.monotonic()
        if self.counts is None:
            self.counts, self.last_report = counts, now
            self._publish_odom()
            return
        dl_p, dr_p = counts[0] - self.counts[0], counts[1] - self.counts[1]
        self.counts = counts
        if max(abs(dl_p), abs(dr_p)) > MAX_PULSES_PER_REPORT:
            self.get_logger().warn(f'Ignoring a jump of {dl_p}/{dr_p} pulses')
            return
        dl, dr = dl_p * self.k_left, dr_p * self.k_right
        self.x, self.y, self.theta = integrate(
            self.x, self.y, self.theta, dl, dr, self.wheel_base)
        span = now - self.last_report
        self.last_report = now
        if span > 0.0:
            v = 0.5 * (dl + dr) / span
            w = (dr - dl) / self.wheel_base / span
            self.v_meas = 0.5 * v + 0.5 * self.v_meas
            self.w_meas = 0.5 * w + 0.5 * self.w_meas
        self.wheel_angle[0] += dl / self.wheel_radius
        self.wheel_angle[1] += dr / self.wheel_radius
        self._publish_odom()

    def _publish_odom(self):
        stamp = self.get_clock().now().to_msg()
        q = Quaternion(z=math.sin(self.theta / 2), w=math.cos(self.theta / 2))
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.v_meas
        odom.twist.twist.angular.z = self.w_meas
        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

    def _publish_joints(self, dt):
        if self.counts is None:
            # No Mega: keep TF alive for Nav2 and RViz (pose unchanged).
            self._publish_odom()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = list(self.wheel_angle)
        self.joint_pub.publish(js)

    def destroy_node(self):
        self._close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
