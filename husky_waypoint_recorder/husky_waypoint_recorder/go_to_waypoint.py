import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from husky_msgs.srv import GoToWaypoint

import yaml
import math
import os
from ament_index_python.packages import get_package_share_directory


class GoToWaypointService(Node):

    def __init__(self):
        super().__init__('go_to_waypoint_service')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = self.load_waypoints()

        self.srv = self.create_service(
            GoToWaypoint,
            'go_to_waypoint',
            self.handle_request
        )

        self.get_logger().info("GoToWaypoint service ready")

    def load_waypoints(self):
        pkg_path = get_package_share_directory('husky_waypoint_recorder')
        yaml_path = os.path.join(pkg_path, 'config', 'waypoints.yaml')

        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)['waypoints']

    def handle_request(self, request, response):
        name = request.name

        if name not in self.waypoints:
            response.accepted = False
            response.message = f"Waypoint '{name}' not found"
            self.get_logger().error(response.message)
            return response

        wp = self.waypoints[name]

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']

        yaw = wp.get('yaw', 0.0)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.client.send_goal_async(goal)

        response.accepted = True
        response.message = f"Navigating to waypoint '{name}'"
        self.get_logger().info(response.message)

        return response


def main():
    rclpy.init()
    node = GoToWaypointService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
