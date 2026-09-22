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


class BaseController(Node):
    BOOT_TIME = 1.8
    RETRY_PERIOD = 0.3

    def disconnect_serial(self, reason):
        self.get_logger().warn(f"Arduino link lost ({reason}); reconnecting")
        try:
            if self.ser is not None:
                self.ser.close()
        except (serial.SerialException, OSError, termios.error):
            pass
        self.ser = None
        self.ready_at = 0.0
        self.next_attempt = time.monotonic() + self.RETRY_PERIOD

    def try_connect(self):
        now = time.monotonic()
        if now < self.next_attempt:
            return
        self.next_attempt = now + self.RETRY_PERIOD
        try:
            self.ser = serial.Serial(
                self.get_parameter('arduino_port').value,
                115200,
                timeout=0,
                write_timeout=0.1,
            )
            self.ready_at = now + self.BOOT_TIME
            self.last_left = None
            self.last_right = None
            self.rx_buf = b''
            self.booted = False
            self.get_logger().info("Arduino port opened, waiting for boot")
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
        self.declare_parameter('max_pulses_per_sample', 4)
        # 'encoder' or 'command' (integrate sent wheel speeds)
        self.declare_parameter('odom_source', 'encoder')
        # Seconds without motion before closing the Arduino port; 0 = never
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

        self.declare_parameter('wheel_base', 0.4318)
        self.declare_parameter('right_encoder_multiplier', 1.0)
        self.declare_parameter('distance_per_pulse', 0.00531)
        self.declare_parameter('motor_speed_multiplier', 1.0)

        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.wheel_radius = 0.0762
        self.last_time = time.time()

        self.timer = self.create_timer(0.05, self.update_sensors_and_odom)

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_nav_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self.cmd_vel_callback, 10)
        self.cmd_smooth_sub = self.create_subscription(
            Twist, '/cmd_vel_smoothed', self.cmd_vel_callback, 10)

        self.get_logger().info("Base Controller started")

    def scale_for_motors(self, speed):
        if abs(speed) < 0.008:
            return 0.0
        sign = 1.0 if speed > 0.0 else -1.0
        # Floor stiction: usable command between 0.08 and 0.12 m/s
        return sign * min(0.12, max(0.08, abs(speed)))

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        v = linear
        w = angular
        wheel_base = self.get_parameter('wheel_base').value
        speed_mult = self.get_parameter('motor_speed_multiplier').value

        vl = (v - (wheel_base / 2.0) * w) * speed_mult
        vr = (v + (wheel_base / 2.0) * w) * speed_mult
        vl = self.scale_for_motors(vl)
        vr = self.scale_for_motors(vr)

        self.target_vl = vl
        self.target_vr = vr
        now = time.monotonic()
        self.last_cmd_time = now
        if abs(vl) > 0.0 or abs(vr) > 0.0:
            self.last_motion_time = now
        self.send_target()

    def link_ready(self):
        if self.ser is None or not self.ser.is_open:
            return False
        if time.monotonic() < self.ready_at:
            return False
        return self.booted

    def send_target(self):
        if not self.link_ready():
            return
        command = f"VL:{self.target_vl:.2f} VR:{self.target_vr:.2f}\n"
        try:
            self.ser.write(command.encode())
        except (serial.SerialException, OSError, termios.error) as e:
            self.disconnect_serial(e)

    def destroy_node(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.write(b"VL:0.00 VR:0.00\n")
                self.ser.flush()
                time.sleep(0.05)
                self.ser.close()
        except (serial.SerialException, OSError, termios.error) as e:
            self.get_logger().error(f"Failed to send shutdown stop: {e}")
        super().destroy_node()

    def wants_link(self):
        idle_close = self.get_parameter('idle_close_s').value
        return idle_close <= 0 or time.monotonic() - self.last_motion_time < idle_close

    def read_lines(self):
        if not self.wants_link():
            if self.ser is not None:
                try:
                    if self.ser.is_open:
                        self.ser.write(b"VL:0.00 VR:0.00\n")
                        self.ser.flush()
                        time.sleep(0.05)
                        self.ser.close()
                except (serial.SerialException, OSError, termios.error):
                    pass
                self.ser = None
                self.booted = False
                self.get_logger().info("Idle: Arduino port closed")
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
                self.get_logger().info("Arduino ready")
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
            return [p.decode('utf-8', errors='ignore').strip() for p in parts[:-1]]
        except (serial.SerialException, OSError, termios.error) as e:
            self.disconnect_serial(e)
            return []

    def update_sensors_and_odom(self):
        now = time.monotonic()
        if now - self.last_cmd_time < 0.5 and int(now * 20) % 4 == 0:
            self.send_target()

        odom_source = self.get_parameter('odom_source').value
        lines = self.read_lines()

        if odom_source == 'command':
            self.command_odom()
            for line in lines:
                pass  # still drain serial
            return

        for line in lines:
            if not line.startswith('L:'):
                continue
            try:
                # L:<left> R:<right>
                parts = line.replace('L:', '').split('R:')
                left = int(parts[0].strip())
                right = int(parts[1].strip())
            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Parse error: {e}")
                continue

            if self.last_left is None:
                self.last_left = left
                self.last_right = right
                self.last_time = time.time()
                continue

            dl = left - self.last_left
            dr = right - self.last_right
            self.last_left = left
            self.last_right = right

            max_p = int(self.get_parameter('max_pulses_per_sample').value)
            dl = max(-max_p, min(max_p, dl))
            dr = max(-max_p, min(max_p, dr))

            dist = self.get_parameter('distance_per_pulse').value
            r_mult = self.get_parameter('right_encoder_multiplier').value
            wheel_base = self.get_parameter('wheel_base').value

            dl_m = dl * dist
            dr_m = dr * dist * r_mult
            dc = 0.5 * (dl_m + dr_m)
            dtheta = (dr_m - dl_m) / wheel_base

            self.x += dc * math.cos(self.theta + dtheta / 2.0)
            self.y += dc * math.sin(self.theta + dtheta / 2.0)
            self.theta += dtheta

            self.left_wheel_angle += dl_m / self.wheel_radius
            self.right_wheel_angle += dr_m / self.wheel_radius
            self._publish_odom(0.0, 0.0)

    def command_odom(self):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0 or dt > 0.5:
            self.last_time = now
            return
        self.last_time = now

        moving = abs(self.target_vl) > 0.0 or abs(self.target_vr) > 0.0
        if not self.link_ready():
            vl = vr = 0.0
        else:
            vl = self.target_vl if moving else 0.0
            vr = self.target_vr if moving else 0.0

        wheel_base = self.get_parameter('wheel_base').value
        v = 0.5 * (vl + vr)
        w = (vr - vl) / wheel_base

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        self.left_wheel_angle += (vl * dt) / self.wheel_radius
        self.right_wheel_angle += (vr * dt) / self.wheel_radius
        self._publish_odom(v, w)

    def _publish_odom(self, v, w):
        stamp = self.get_clock().now().to_msg()
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

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

        js = JointState()
        js.header.stamp = stamp
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
