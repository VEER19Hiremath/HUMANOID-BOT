"""Draw the rooms of the loaded map in RViz (/room_markers).

Each room gets a coloured floor zone, its name and a goal pin; the room the
robot is driving to is highlighted. Rooms come from the map yaml's
"# room: name x y" lines (scripts/make_area_map.py), so RViz always shows the
rooms the voice commands use. The map image stays plain: zones drawn into it
would be obstacles to the planner.
"""

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from hospital_delivery.room_config import ROOM_COORDS, rooms_from_map

ZONE = 0.9          # m, side of the drawn room zone
COLORS = [(0.20, 0.60, 1.00), (1.00, 0.55, 0.15), (0.30, 0.80, 0.40),
          (0.80, 0.35, 0.85), (0.95, 0.80, 0.20), (0.20, 0.80, 0.80)]
HOME_COLOR = (0.85, 0.85, 0.85)


class RoomMarkers(Node):
    def __init__(self):
        super().__init__('room_markers')
        self.declare_parameter('map_yaml', '')
        map_yaml = self.get_parameter('map_yaml').value
        self.rooms = (rooms_from_map(map_yaml) if map_yaml else None) or ROOM_COORDS
        self.active = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'room_markers', latched)
        # The planned path ends at the current goal (voice goals go through
        # the Nav2 action, not /goal_pose): highlight that room.
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.create_timer(2.0, self.publish)
        self.publish()
        self.get_logger().info(f'Drawing {len(self.rooms)} rooms in RViz (/room_markers)')

    def _on_plan(self, msg):
        if not msg.poses:
            return
        p = msg.poses[-1].pose.position
        name, (x, y) = min(self.rooms.items(),
                           key=lambda kv: (kv[1][0] - p.x) ** 2 + (kv[1][1] - p.y) ** 2)
        active = name if (x - p.x) ** 2 + (y - p.y) ** 2 < 0.5 ** 2 else None
        if active != self.active:
            self.active = active
            self.publish()

    def _marker(self, mid, mtype, ns):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish(self):
        out = MarkerArray()
        for i, (name, (x, y)) in enumerate(sorted(self.rooms.items())):
            r, g, b = HOME_COLOR if name == 'home' else COLORS[i % len(COLORS)]
            active = name == self.active
            zone = self._marker(i, Marker.CUBE, 'zones')
            zone.pose.position.x, zone.pose.position.y = x, y
            zone.pose.position.z = 0.005
            zone.scale.x = zone.scale.y = ZONE
            zone.scale.z = 0.01
            zone.color.r, zone.color.g, zone.color.b = r, g, b
            zone.color.a = 0.55 if active else 0.25
            pin = self._marker(i, Marker.CYLINDER, 'goals')
            pin.pose.position.x, pin.pose.position.y = x, y
            pin.pose.position.z = 0.15
            pin.scale.x = pin.scale.y = 0.12
            pin.scale.z = 0.30
            pin.color.r, pin.color.g, pin.color.b, pin.color.a = r, g, b, 1.0
            label = self._marker(i, Marker.TEXT_VIEW_FACING, 'labels')
            label.pose.position.x, label.pose.position.y = x, y + ZONE / 2 + 0.15
            label.pose.position.z = 0.4
            label.scale.z = 0.28
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            text = 'Home' if name == 'home' else name.replace('room', 'Room ')
            label.text = f'> {text} <' if active else text
            out.markers += [zone, pin, label]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = RoomMarkers()
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
