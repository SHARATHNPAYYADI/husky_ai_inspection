# auto_initial_pose.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class AutoInitialPose(Node):
    def __init__(self):
        super().__init__('auto_initial_pose')

        # Match AMCL's QoS exactly — do NOT use default QoS
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos
        )

        # Subscribe to /amcl_pose to detect when AMCL is alive
        self.amcl_ready = False
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.attempts = 0
        self.max_attempts = 30  # 30 seconds max wait

        # Check every 1 second if AMCL is up
        self.timer = self.create_timer(1.0, self.check_and_publish)
        self.get_logger().info('Waiting for AMCL to become ready...')

    def amcl_pose_callback(self, msg):
        # Once AMCL publishes anything on /amcl_pose, it is alive
        if not self.amcl_ready:
            self.amcl_ready = True
            self.get_logger().info('AMCL detected as active via /amcl_pose!')

    def check_and_publish(self):
        self.attempts += 1

        # Check if AMCL has a publisher on /amcl_pose (it's alive)
        amcl_publishers = self.count_publishers('/amcl_pose')

        if amcl_publishers == 0 and not self.amcl_ready:
            self.get_logger().info(
                f'AMCL not ready yet... ({self.attempts}/{self.max_attempts})'
            )
            if self.attempts >= self.max_attempts:
                self.get_logger().error(
                    'AMCL never became ready after 30s. Giving up.'
                )
                self.timer.cancel()
            return

        # AMCL is ready — publish initial pose and stop
        self.publish_pose()
        self.timer.cancel()

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Pose from AMCL log: x=0.045, y=-0.003, yaw=0.009 rad
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        # yaw=0.009 rad → quaternion
        yaw = -0.043722
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Non-zero covariance so AMCL can refine from here
        cov = [0.0] * 36
        cov[0]  = 0.25   # x variance (±0.5m)
        cov[7]  = 0.25   # y variance (±0.5m)
        cov[35] = 0.068  # yaw variance (~±15 deg)
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.get_logger().info(
            f'✅ Initial pose published → '
            f'x={msg.pose.pose.position.x}, '
            f'y={msg.pose.pose.position.y}, '
            f'yaw={yaw:.4f} rad'
        )


def main():
    rclpy.init()
    node = AutoInitialPose()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()