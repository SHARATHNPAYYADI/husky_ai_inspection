# auto_initial_pose.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class AutoInitialPose(Node):
    def __init__(self):
        super().__init__('auto_initial_pose')
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        self.timer = self.create_timer(2.0, self.publish_pose)

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x =  -0.0714
        msg.pose.pose.position.y = -0.019
        msg.pose.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.get_logger().info('Initial pose published')
        self.timer.cancel()

def main():
    rclpy.init()
    node = AutoInitialPose()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
