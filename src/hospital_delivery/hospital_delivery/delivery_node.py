#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class DeliveryNode(Node):

    def __init__(self):
        super().__init__('delivery_node')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Hospital room coordinates
        self.rooms = {
            "room1": (7.9, 3.5),
            "room2": (-2.6, -1.6),
            "room3": (-2.6, 1.6),
            "room4": (-7.9, -3.5),
            "room5": (-7.9, 3.5)
        }

        self.status_names = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }

    def send_goal(self, room_name):

        if room_name not in self.rooms:
            self.get_logger().error(
                f"Unknown room: {room_name}"
            )
            return

        x, y = self.rooms[room_name]

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Navigating to {room_name}"
        )

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "NavigateToPose action server not available."
            )
            return

        future = self.nav_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error(
                "Failed to send navigation goal."
            )
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                "Goal rejected."
            )
            return

        self.get_logger().info(
            "Goal accepted."
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if result is None:
            self.get_logger().error(
                "No result received from navigation."
            )
            return

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Arrived at {room_name}"
            )

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(
                f"Navigation to {room_name} SUCCEEDDED"
            )

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(
                f"Navigation to {room_name} CANCELED"
            )

        else:
            self.get_logger().error(
                f"Navigation finished with status: "
                f"{self.status_names.get(status, str(status))}"
            )


def main():

    rclpy.init()

    if len(sys.argv) < 2:
        print(
            "Usage:\n"
            "ros2 run hospital_delivery delivery_node room1"
        )
        rclpy.shutdown()
        return

    room = sys.argv[1].lower()

    node = DeliveryNode()

    node.send_goal(room)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()