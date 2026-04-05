import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from husky_msgs.srv import GoToWaypoint
from std_srvs.srv import Trigger

import yaml
import math
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor


class GoToWaypointService(Node):

    def __init__(self):
        super().__init__('go_to_waypoint_service')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.pending_waypoints = []

        self.waypoints = self.load_waypoints()

        self.goto_srv = self.create_service(
            GoToWaypoint,
            'go_to_waypoint',
            self.handle_goto_request
        )

        self.stop_srv = self.create_service(
            Trigger,
            'stop_navigation',
            self.handle_stop_request
        )

        self.get_logger().info("GoToWaypoint + StopNavigation services ready")

    # -----------------------------------------------------
    # Waypoint loading
    # -----------------------------------------------------
    def load_waypoints(self):
        pkg_path = get_package_share_directory('husky_waypoint_recorder')
        yaml_path = os.path.join(pkg_path, 'config', 'waypoints.yaml')

        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)['waypoints']

    # -----------------------------------------------------
    # GO TO WAYPOINT SERVICE
    # -----------------------------------------------------
    def handle_goto_request(self, request, response):
        names = [name.strip() for name in request.names if name.strip()]

        if not names:
            response.accepted = False
            response.message = "At least one waypoint is required"
            self.get_logger().error(response.message)
            return response

        if self.goal_handle is not None:
            response.accepted = False
            response.message = "Navigation already in progress"
            self.get_logger().warn(response.message)
            return response

        missing = [name for name in names if name not in self.waypoints]
        if missing:
            response.accepted = False
            response.message = f"Waypoint '{missing[0]}' not found"
            self.get_logger().error(response.message)
            return response

        self.pending_waypoints = list(names)
        self.client.wait_for_server()
        self.send_next_waypoint()

        response.accepted = True
        response.message = f"Navigating to {len(names)} waypoint(s)"
        self.get_logger().info(response.message)

        return response

    # -----------------------------------------------------
    # STOP NAVIGATION SERVICE
    # -----------------------------------------------------
    def handle_stop_request(self, request, response):
        if self.goal_handle is None:
            response.success = False
            response.message = "No active navigation goal"
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info("Cancelling active navigation goal...")
        self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.pending_waypoints = []
        response.success = True
        response.message = "Navigation cancelled"

        self.get_logger().info(response.message)
        return response

    # -----------------------------------------------------
    # ACTION CALLBACKS
    # -----------------------------------------------------
    def send_next_waypoint(self):
        if not self.pending_waypoints:
            self.get_logger().info("All requested waypoints completed")
            return

        name = self.pending_waypoints[0]
        wp = self.waypoints[name]

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']

        yaw = wp.get('yaw', 0.0)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f"Sending waypoint '{name}'")

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            self.pending_waypoints = []
            self.goal_handle = None
            return

        self.get_logger().info("Navigation goal accepted")
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status
        completed_name = self.pending_waypoints.pop(0) if self.pending_waypoints else None

        self.get_logger().info("Navigation finished")
        self.goal_handle = None

        if status == 4 and self.pending_waypoints:
            self.send_next_waypoint()
            return

        if status != 4:
            if completed_name:
                self.get_logger().warn(
                    f"Navigation stopped before completing full sequence at '{completed_name}'"
                )
            self.pending_waypoints = []

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Example:
        # self.get_logger().info(
        #     f"Distance remaining: {feedback.distance_remaining:.2f} m"
        # )


def main():
    rclpy.init()
    node = GoToWaypointService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
