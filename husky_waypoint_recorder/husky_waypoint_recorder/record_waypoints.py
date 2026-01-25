import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml
import sys

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('husky_waypoint_recorder')
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )
        self.waypoints = []
        self.get_logger().info("Click points in RViz. Press ENTER to save waypoint.")

    def point_callback(self, msg):
        x = float(msg.point.x)
        y = float(msg.point.y)

        waypoint = {
            'x': round(x, 3),
            'y': round(y, 3),
            'yaw': 0.0   # can be improved later
        }

        self.waypoints.append(waypoint)
        self.get_logger().info(f"Waypoint added: {waypoint}")

    def save_to_yaml(self, filename="waypoints.yaml"):
        data = {'waypoints': self.waypoints}
        with open(filename, 'w') as file:
            yaml.dump(data, file)
        self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to {filename}")

def main():
    rclpy.init()
    node = WaypointRecorder()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_to_yaml()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
