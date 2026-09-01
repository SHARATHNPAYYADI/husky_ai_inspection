import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

        # Match Isaac Sim's RELIABLE publisher
        sub_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Publish RELIABLE for AMCL/costmap
        pub_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.cb, sub_qos)
        self.pub = self.create_publisher(LaserScan, '/scan', pub_qos)

    def cb(self, msg):
        msg.time_increment = 0.0
        msg.range_min = max(msg.range_min, 0.12)
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ScanFilter())