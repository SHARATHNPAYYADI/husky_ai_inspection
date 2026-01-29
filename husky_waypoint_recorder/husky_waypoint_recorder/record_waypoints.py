import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import math
# from tf_transformations import euler_from_quaternion


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('husky_waypoint_recorder')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_callback,
            10
        )

        self.waypoints = []
        self.get_logger().info(
            "Use '2D goal pose' in RViz to record waypoints with orientation"
        )
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
    
    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        waypoint = {
            'x': round(x, 3),
            'y': round(y, 3),
            'yaw': round(yaw, 3)
        }

        self.waypoints.append(waypoint)
        self.get_logger().info(f"Waypoint added: {waypoint}")


    def save_to_yaml(self, filename="waypoints.yaml"):
        with open(filename, 'w') as f:
            yaml.dump({'waypoints': self.waypoints}, f)
        self.get_logger().info(
            f"Saved {len(self.waypoints)} waypoints to {filename}"
        )


def main():
    rclpy.init()
    node = WaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_to_yaml()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
