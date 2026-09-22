#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import math
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion



class WheelOdom(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200)

        self.odom_pub = self.create_publisher(Odometry,'/odom',10)

        self.timer = self.create_timer(0.05,self.update)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left = 0
        self.last_right = 0

        self.distance_per_pulse = 0.00531
        self.wheel_base = 0.4699

        self.last_time = time.time()


    def update(self):

        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except:
            return

        if "L:" not in line:
            return

        parts = line.split()

        left = int(parts[0].split(":")[1])
        right = int(parts[1].split(":")[1])

        dl = (left - self.last_left) * self.distance_per_pulse
        dr = (right - self.last_right) * self.distance_per_pulse

        self.last_left = left
        self.last_right = right

        d = (dl + dr)/2
        dtheta = (dr - dl)/self.wheel_base

        self.theta += dtheta
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        vx = d / dt if dt > 0 else 0
        vth = dtheta / dt if dt > 0 else 0

        odom = Odometry()

        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = [0.0, 0.0, math.sin(self.theta * 0.5), math.cos(self.theta * 0.5)]

        odom.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = WheelOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
