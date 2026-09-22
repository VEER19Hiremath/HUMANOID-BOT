#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.write(b"VL:0.00 VR:0.00\n")
        self.ser.flush()

        self.wheel_base = 0.4699

        self.get_logger().info("Motor driver started")


    def callback(self,msg):

        v = msg.linear.x
        w = msg.angular.z

        vr = v + (self.wheel_base/2)*w
        vl = v - (self.wheel_base/2)*w

        command = f"VL:{vl:.2f} VR:{vr:.2f}\n"

        self.get_logger().info(f"Sending: {command.strip()}")
        self.ser.write(command.encode())
        self.ser.flush()
        
        # Read everything available in buffer
        while self.ser.in_waiting > 0:
            rx = self.ser.readline().decode(errors='ignore').strip()
            if rx:
                self.get_logger().info(f"Arduino says: {rx}")

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b"VL:0.00 VR:0.00\n")
                self.ser.flush()
                time.sleep(0.05)
                self.ser.close()
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Failed to send shutdown stop: {e}")
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
