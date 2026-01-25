import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math

class WaypointFollowerClient(Node):

    def __init__(self):
        super().__init__('waypoint_follower_client')
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        pkg_path = get_package_share_directory('husky_waypoint_recorder')
        yaml_path = os.path.join(pkg_path, 'config', 'waypoints.yaml')

        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)

        poses = []
        for wp in data['waypoints']:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']

            yaw = wp.get('yaw', 0.0)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            poses.append(pose)

        self.get_logger().info(f"Loaded {len(poses)} waypoints")
        return poses

    def send_waypoints(self):
        self.client.wait_for_server()

        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints

        self.get_logger().info("Sending waypoints to Nav2...")
        self.client.send_goal_async(goal)

def main():
    rclpy.init()
    node = WaypointFollowerClient()
    node.send_waypoints()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
