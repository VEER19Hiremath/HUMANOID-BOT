#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

import sys

import termios

import tty

import select

class TeleopKeyboard(Node):

    def __init__(self):

        super().__init__('teleop_keyboard')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed = 0.4

        self.angular_speed = 0.8

        self.get_logger().info("Teleop Started")

        self.get_logger().info("W/S = forward/back")

        self.get_logger().info("A/D = rotate")

        self.get_logger().info("Q/E = speed +/-")

        self.get_logger().info("SPACE = stop")

        self.run()

    def get_key(self):

        tty.setraw(sys.stdin.fileno())

        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

        if rlist:

            key = sys.stdin.read(1)

        else:

            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        return key

    def run(self):

        self.current_v = 0.0
        self.current_w = 0.0

        while True:

            key = self.get_key()

            msg = Twist()

            if key == 'w':

                self.current_v = self.linear_speed
                self.current_w = 0.0

            elif key == 's':

                self.current_v = -self.linear_speed
                self.current_w = 0.0

            elif key == 'a':

                self.current_v = 0.0
                self.current_w = self.angular_speed

            elif key == 'd':

                self.current_v = 0.0
                self.current_w = -self.angular_speed

            elif key == 'q':

                self.linear_speed += 0.05

                self.angular_speed += 0.05

                print(f"Speed: {self.linear_speed:.2f}")

            elif key == 'e':

                self.linear_speed -= 0.05

                self.angular_speed -= 0.05

                print(f"Speed: {self.linear_speed:.2f}")

            elif key == ' ':

                self.current_v = 0.0

                self.current_w = 0.0

            elif key == '\x03':

                break

            msg.linear.x = float(self.current_v)
            msg.angular.z = float(self.current_w)

            self.pub.publish(msg)

def main():

    global settings

    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    node = TeleopKeyboard()

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()