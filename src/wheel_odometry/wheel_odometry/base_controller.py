#!/usr/bin/env python3
"""Base controller: Arduino wheel drive + odometry (encoder or command)."""

import math
import time
import termios

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from rclpy.executors import ExternalShutdownException

from wheel_odometry.motion_limits import MIN_TRAVEL_M, inside_map, wheel_travel


class BaseController(Node):
    BOOT_TIME = 1.8
    RETRY_PERIOD = 0.3

    def disconnect_serial(self, reason):
        self.get_logger().warn(f'Arduino link lost ({reason}); reconnecting')
        self.close_arduino(self.ser)
        self.ser = None
        self.ready_at = 0.0
        self.last_write_ok_at = 0.0
        self.next_attempt = time.monotonic() + self.RETRY_PERIOD

    def open_arduino(self):
        """Open the Mega without the DTR pulse that holds it in reset."""
        port = self.get_parameter('arduino_port').value
        link = serial.Serial()
        link.port = port
        link.baudrate = 115200
        link.timeout = 0
        link.write_timeout = 0.1
        link.dtr = False
        link.rts = False
        try:
            link.open()
            return link
        except (serial.SerialException, OSError, termios.error):
            try:
                link.close()
            except (serial.SerialException, OSError, termios.error):
                pass
            return serial.Serial(port, 115200, timeout=0, write_timeout=0.1)

    def close_arduino(self, link):
        """Stop the wheels, then close without the USB reset (HUPCL) pulse."""
        if link is None:
            return
        try:
            if link.is_open:
                link.write(b'VL:0.00 VR:0.00\n')
                link.flush()
                time.sleep(0.05)
                attr = termios.tcgetattr(link.fd)
                attr[2] = attr[2] & ~termios.HUPCL
                termios.tcsetattr(link.fd, termios.TCSANOW, attr)
                link.dtr = False
                link.close()
        except (serial.SerialException, OSError, termios.error):
            try:
                link.close()
            except (serial.SerialException, OSError, termios.error):
                pass

    def try_connect(self):
        now = time.monotonic()
        if now < self.next_attempt:
            return
        self.next_attempt = now + self.RETRY_PERIOD
        try:
            self.ser = self.open_arduino()
            self.ready_at = now + self.BOOT_TIME
            self.last_left = None
            self.last_right = None
            self.rx_buf = b''
            self.booted = False
            self.get_logger().info('Arduino port opened, waiting for boot')
        except (serial.SerialException, OSError, termios.error):
            self.ser = None

    def __init__(self):
        super().__init__('base_controller')

        self.ser = None
        self.ready_at = 0.0
        self.next_attempt = 0.0
        self.rx_buf = b''
        self.booted = False
        self.last_cmd_time = time.monotonic()
        self.last_motion_time = time.monotonic()

        self.declare_parameter('arduino_port', '/dev/arduino')
        self.declare_parameter('max_pulses_per_sample', 250)
        self.declare_parameter('odom_source', 'encoder')
        self.declare_parameter('open_loop_odom', False)
        self.declare_parameter('idle_close_s', 0.0)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = None
        self.last_right = None
        self.target_vl = 0.0
        self.target_vr = 0.0
        self.slewed_vl = 0.0
        self.slewed_vr = 0.0
        self.v_filt = 0.0
        self.w_filt = 0.0
        self.last_enc_mono = 0.0
        self._pose_from_enc_at = 0.0
        self.last_write_ok_at = 0.0
        self._enc_dead_since = None
        self._enc_dead_logged = False

        self.declare_parameter('wheel_base', 0.4318)
        self.declare_parameter('right_encoder_multiplier', 1.0)
        self.declare_parameter('distance_per_pulse', 0.00531)
        self.declare_parameter('motor_speed_multiplier', 1.0)

        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.wheel_radius = 0.0762
        self.last_time = time.time()

        # 50 Hz keeps RViz / TF live with the wheels.
        self.timer = self.create_timer(0.02, self.update_sensors_and_odom)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info('Base Controller started')

    def scale_for_motors(self, speed):
        if abs(speed) < 0.008:
            return 0.0
        sign = 1.0 if speed > 0.0 else -1.0
        return sign * min(0.12, abs(speed))

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        v = linear
        w = angular
        wheel_base = self.get_parameter('wheel_base').value
        speed_mult = self.get_parameter('motor_speed_multiplier').value
        half = wheel_base / 2.0

        # Avoid opposite-sign wheel cmds (in-place pivot). Mega stall logic
        # then zeros the slow side and the robot spins on one spot.
        if abs(w) > 0.02 and abs(v) < 0.09:
            v = 0.10 if v >= 0.0 else -0.10
        if abs(v) >= 0.02 and half > 1e-6:
            w_max = abs(v) / half * 0.85
            if abs(w) > w_max:
                w = w_max if w >= 0.0 else -w_max

        vl = (v - half * w) * speed_mult
        vr = (v + half * w) * speed_mult
        # Left side is weaker / stalls more often — slight bias.
        if abs(vl) > 0.008:
            vl = (1.0 if vl > 0.0 else -1.0) * min(0.12, abs(vl) * 1.08)
        vl = self.scale_for_motors(vl)
        vr = self.scale_for_motors(vr)

        self.target_vl = vl
        self.target_vr = vr
        now = time.monotonic()
        self.last_cmd_time = now
        if (abs(v) > 0.008 or abs(w) > 0.02 or abs(vl) > 0.0 or abs(vr) > 0.0):
            self.last_motion_time = now

    def slew_wheels(self, dt):
        """Ease wheel speed so a new command does not step the pose."""
        accel = 0.06
        for current_name, target in (
            ('slewed_vl', self.target_vl),
            ('slewed_vr', self.target_vr),
        ):
            current = getattr(self, current_name)
            delta = target - current
            limit = (0.2 if abs(target) < abs(current) else accel) * dt
            if delta > limit:
                delta = limit
            elif delta < -limit:
                delta = -limit
            updated = current + delta
            if abs(target - updated) < 0.001:
                updated = target
            setattr(self, current_name, updated)

    def link_ready(self):
        if self.ser is None or not self.ser.is_open:
            return False
        if time.monotonic() < self.ready_at:
            return False
        return self.booted

    def send_target(self):
        if not self.link_ready():
            return
        command = f'VL:{self.slewed_vl:.2f} VR:{self.slewed_vr:.2f}\n'
        try:
            self.ser.write(command.encode())
            self.last_write_ok_at = time.monotonic()
        except (serial.SerialException, OSError, termios.error) as e:
            self.disconnect_serial(e)

    def drive_confirmed(self):
        """True only if Mega link is up and a write succeeded recently."""
        return self.link_ready() and (
            time.monotonic() - self.last_write_ok_at < 0.4)

    def destroy_node(self):
        try:
            self.close_arduino(self.ser)
        except (serial.SerialException, OSError, termios.error) as e:
            self.get_logger().error(f'Failed to send shutdown stop: {e}')
        super().destroy_node()

    def wants_link(self):
        idle_close = self.get_parameter('idle_close_s').value
        return idle_close <= 0 or (
            time.monotonic() - self.last_motion_time < idle_close)

    def read_lines(self):
        if not self.wants_link():
            if self.ser is not None:
                self.close_arduino(self.ser)
                self.ser = None
                self.booted = False
                self.last_write_ok_at = 0.0
                self.get_logger().info('Idle: Arduino port closed')
            return []

        if self.ser is None:
            self.try_connect()
            return []

        if not self.booted:
            if time.monotonic() < self.ready_at:
                return []
            try:
                data = self.ser.read(self.ser.in_waiting or 0)
            except (serial.SerialException, OSError, termios.error):
                return []
            if b'SAFE START READY' in data or b'L:' in data:
                if b'\n' in data:
                    data = data.split(b'\n', 1)[1]
                self.booted = True
                self.get_logger().info('Arduino ready')
                if time.monotonic() - self.last_cmd_time < 0.5:
                    self.send_target()
                self.rx_buf += data
            else:
                self.rx_buf += data
                if len(self.rx_buf) > 4096:
                    self.rx_buf = b''
                return []

        try:
            chunk = self.ser.read(self.ser.in_waiting or 0)
            self.rx_buf += chunk
            if len(self.rx_buf) > 4096:
                self.rx_buf = b''
                return []
            parts = self.rx_buf.split(b'\n')
            self.rx_buf = parts[-1]
            return [
                p.decode('utf-8', errors='ignore').strip()
                for p in parts[:-1]
            ]
        except (serial.SerialException, OSError, termios.error) as e:
            self.disconnect_serial(e)
            return []

    def update_sensors_and_odom(self):
        now = time.monotonic()
        dt = now - getattr(self, 'last_slew_mono', now)
        self.last_slew_mono = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        if now - self.last_cmd_time > 0.5:
            self.target_vl = 0.0
            self.target_vr = 0.0

        self.slew_wheels(dt)
        self._spin_wheel_joints(dt)
        self._publish_wheel_joints()

        if (now - self.last_cmd_time < 0.5
                or abs(self.slewed_vl) > 0.001
                or abs(self.slewed_vr) > 0.001):
            self.send_target()

        odom_source = self.get_parameter('odom_source').value
        lines = self.read_lines()
        self._log_arduino_status(lines)

        if odom_source == 'command':
            self.command_odom(lines)
            return

        self._absorb_encoders(lines)
        self._glide_odom()

    def _log_arduino_status(self, lines):
        """Surface Mega pin/PWM proof while diagnosing dead drivers."""
        now = time.monotonic()
        if now - getattr(self, '_last_status_log', 0.0) < 1.0:
            return
        for line in lines:
            if line.startswith('S:'):
                self._last_status_log = now
                self.get_logger().info(f'Mega drive {line}')
                break

    def _spin_wheel_joints(self, dt):
        """Advance continuous wheel joints for a smooth RViz roll."""
        step = min(dt, 0.05)
        self.left_wheel_angle += (self.slewed_vl / self.wheel_radius) * step
        self.right_wheel_angle += (self.slewed_vr / self.wheel_radius) * step

    def _publish_wheel_joints(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.joint_pub.publish(js)

    def _absorb_encoders(self, lines):
        for line in lines:
            if not line.startswith('L:'):
                continue
            try:
                parts = line.replace('L:', '').split('R:')
                left = int(parts[0].strip())
                right = int(parts[1].strip())
            except (IndexError, ValueError) as e:
                self.get_logger().error(f'Parse error: {e}')
                continue

            now = time.monotonic()
            if self.last_left is None:
                self.last_left = left
                self.last_right = right
                self.last_enc_mono = now
                continue

            dt = now - self.last_enc_mono
            dl = left - self.last_left
            dr = right - self.last_right
            self.last_left = left
            self.last_right = right
            self.last_enc_mono = now

            max_p = self.get_parameter('max_pulses_per_sample').value
            if max(abs(dl), abs(dr)) > max_p:
                continue

            dist = self.get_parameter('distance_per_pulse').value
            r_mult = self.get_parameter('right_encoder_multiplier').value
            wheel_base = self.get_parameter('wheel_base').value
            dl_m = dl * dist
            dr_m = dr * dist * r_mult
            if dt <= 0.0:
                continue

            # Integrate this sample into the pose immediately so RViz moves live.
            travel = 0.5 * (dl_m + dr_m)
            dtheta = (dr_m - dl_m) / wheel_base
            nx = self.x + travel * math.cos(self.theta + 0.5 * dtheta)
            ny = self.y + travel * math.sin(self.theta + 0.5 * dtheta)
            if not inside_map(nx, ny):
                # Keep pose on the map so Nav2 can still plan. Stop drive.
                self.target_vl = 0.0
                self.target_vr = 0.0
                self.slewed_vl = 0.0
                self.slewed_vr = 0.0
                self.v_filt = 0.0
                self.w_filt = 0.0
                self._publish_odom(0.0, 0.0)
                continue
            self.x = nx
            self.y = ny
            self.theta += dtheta

            v_meas = travel / dt
            w_meas = dtheta / dt
            self.v_filt = 0.75 * v_meas + 0.25 * self.v_filt
            self.w_filt = 0.75 * w_meas + 0.25 * self.w_filt
            self._pose_from_enc_at = now
            self.last_time = time.time()
            self._publish_odom(self.v_filt, self.w_filt)

    def _glide_odom(self):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0 or dt > 0.5:
            self.last_time = now
            self._publish_odom(self.v_filt, self.w_filt)
            return
        self.last_time = now

        # Pose already stepped from the latest encoder packet — only republish TF.
        if self._pose_from_enc_at and (
                time.monotonic() - self._pose_from_enc_at < 0.05):
            self._publish_odom(self.v_filt, self.w_filt)
            return

        if self.last_enc_mono and (
                time.monotonic() - self.last_enc_mono > 0.4):
            self.v_filt *= 0.4
            self.w_filt *= 0.4

        # Coast between Mega reports so the model keeps moving smoothly.
        if abs(self.v_filt) > 1e-4 or abs(self.w_filt) > 1e-4:
            if not self._step_inside_map(self.v_filt, self.w_filt, dt):
                self.target_vl = 0.0
                self.target_vr = 0.0
                self.v_filt = 0.0
                self.w_filt = 0.0
        self._publish_odom(self.v_filt, self.w_filt)

    def _wheel_sample(self, lines):
        """(meters, seconds) since the previous encoder line, or None.

        Counts only increase, so this is distance, not direction. The time
        span is the gap between wheel reports (about 0.25 s), not the 20 Hz
        timer. Using the timer made RViz record about one fifth of the
        real motion.
        """
        sample = None
        for line in lines:
            if not line.startswith('L:'):
                continue
            try:
                parts = line.replace('L:', '').split('R:')
                left = int(parts[0].strip())
                right = int(parts[1].strip())
            except (IndexError, ValueError):
                continue

            now = time.monotonic()
            if self.last_left is None:
                self.last_left = left
                self.last_right = right
                self.last_enc_mono = now
                sample = (0.0, 0.0)
                continue

            dt = now - self.last_enc_mono
            dl = left - self.last_left
            dr = right - self.last_right
            self.last_left = left
            self.last_right = right
            self.last_enc_mono = now

            if (dt <= 0.0 or dt > 1.0 or dl < 0 or dr < 0
                    or dl > 120 or dr > 120):
                sample = (0.0, dt)
                continue

            dist = self.get_parameter('distance_per_pulse').value
            r_mult = self.get_parameter('right_encoder_multiplier').value
            left_m = dl * dist
            right_m = dr * dist * r_mult
            travel = wheel_travel(left_m, right_m, MIN_TRAVEL_M)
            sample = (travel, dt)
        return sample

    def _note_encoder_motion(self, travel):
        """Warn once when the motors are driven and both counts stay at 0.

        The encoder LEDs are powered from the Mega 5V pin, not the motor
        battery. A dark encoder cannot produce a count, so the pose stays put.
        """
        moving = abs(self.slewed_vl) > 0.05 or abs(self.slewed_vr) > 0.05
        if travel >= 0.004 or not moving:
            self._enc_dead_since = None
            return

        now = time.monotonic()
        if self._enc_dead_since is None:
            self._enc_dead_since = now
            return
        if self._enc_dead_logged or now - self._enc_dead_since < 2.0:
            return
        self._enc_dead_logged = True
        self.get_logger().error(
            'Motors are commanded but both encoder counts stayed at 0. '
            'Encoder lights need Mega 5V: yellow wire to 5V, black wire to GND.')

    def _motors_commanded(self):
        return abs(self.slewed_vl) > 0.008 or abs(self.slewed_vr) > 0.008

    def _assist_command_odom(self, dt):
        self._command_open_loop(min(dt, 0.05))

    def command_odom(self, lines):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0 or dt > 0.5:
            self.last_time = now
            return
        self.last_time = now

        open_loop = bool(self.get_parameter('open_loop_odom').value)

        if open_loop:
            if self.drive_confirmed():
                sample = self._wheel_sample(lines)
                if sample is not None:
                    self._note_encoder_motion(sample[0])
            self._command_open_loop(min(dt, 0.05))
            return

        if not self.drive_confirmed():
            self._publish_odom(0.0, 0.0)
            return

        sample = self._wheel_sample(lines)
        if sample is None:
            if self._motors_commanded():
                self._assist_command_odom(dt)
            else:
                self._publish_odom(self.v_filt, self.w_filt)
            return

        travel, dt_enc = sample
        self._note_encoder_motion(travel)

        if dt_enc <= 0.0 or travel < MIN_TRAVEL_M:
            if self._motors_commanded():
                self._assist_command_odom(dt)
            else:
                self.v_filt = 0.0
                self.w_filt = 0.0
                self._publish_odom(0.0, 0.0)
            return

        vl = self.slewed_vl
        vr = self.slewed_vr
        wheel_base = self.get_parameter('wheel_base').value
        wheel = 0.5 * (abs(vl) + abs(vr))
        if wheel < 0.001:
            self.v_filt = 0.0
            self.w_filt = 0.0
            self._publish_odom(0.0, 0.0)
            return

        meas = travel / dt_enc
        scale = min(1.0, meas / wheel)
        v = 0.5 * (vl + vr) * scale
        w = (vr - vl) / wheel_base * scale
        if abs(v) > 1.0:
            cap = 1.0 / abs(v)
            v *= cap
            w *= cap

        self.v_filt = v
        self.w_filt = w
        if not self._step_inside_map(v, w, dt_enc):
            self.target_vl = 0.0
            self.target_vr = 0.0
            self.v_filt = 0.0
            self.w_filt = 0.0
            self._publish_odom(0.0, 0.0)
            return
        self._publish_odom(v, w)

    def _command_open_loop(self, dt):
        """Advance the pose from commanded wheel speeds for RViz dry-run."""
        vl = self.slewed_vl
        vr = self.slewed_vr
        if abs(vl) < 0.008 and abs(vr) < 0.008:
            self.v_filt = 0.0
            self.w_filt = 0.0
            self._publish_odom(0.0, 0.0)
            return

        wheel_base = self.get_parameter('wheel_base').value
        v = 0.5 * (vl + vr)
        w = (vr - vl) / wheel_base
        self.v_filt = v
        self.w_filt = w
        if not self._step_inside_map(v, w, dt):
            self.target_vl = 0.0
            self.target_vr = 0.0
            self.v_filt = 0.0
            self.w_filt = 0.0
            self._publish_odom(0.0, 0.0)
            return
        self._publish_odom(v, w)

    def _step_inside_map(self, v, w, dt):
        """Advance the pose only while it stays on the 28 x 40 ft map."""
        nx = self.x + v * math.cos(self.theta) * dt
        ny = self.y + v * math.sin(self.theta) * dt
        if not inside_map(nx, ny):
            return False
        self.x = nx
        self.y = ny
        self.theta += w * dt
        return True

    def _integrate(self, v, w, dt):
        if not self._step_inside_map(v, w, dt):
            self.v_filt = 0.0
            self.w_filt = 0.0

    def _ensure_inside_map(self):
        """If pose already left the map (old run), snap back to home."""
        if inside_map(self.x, self.y):
            return
        self.get_logger().warn(
            f'Pose left the map (odom x={self.x:.2f} y={self.y:.2f}); '
            'resetting to home'
        )
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_filt = 0.0
        self.w_filt = 0.0
        self.target_vl = 0.0
        self.target_vr = 0.0
        self.slewed_vl = 0.0
        self.slewed_vr = 0.0

    def _publish_odom(self, v, w):
        self._ensure_inside_map()
        stamp = self.get_clock().now().to_msg()
        q = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.theta * 0.5),
            w=math.cos(self.theta * 0.5),
        )
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
