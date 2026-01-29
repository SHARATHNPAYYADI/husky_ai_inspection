import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from husky_msgs.srv import CaptureImage
import cv2
import os
from datetime import datetime


class ImageCaptureService(Node):

    def __init__(self):
        super().__init__('image_capture_service')

        self.bridge = CvBridge()
        self.latest_image = None

        self.image_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            10
        )

        self.srv = self.create_service(
            CaptureImage,
            'capture_image',
            self.capture_callback
        )

        self.save_dir = os.path.expanduser('~/captured_images')
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info('📸 Image capture service ready')

    def image_callback(self, msg):
        self.latest_image = msg

    def capture_callback(self, request, response):
        if self.latest_image is None:
            response.success = False
            response.message = 'No image received yet'
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                self.latest_image, desired_encoding='bgr8'
            )

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = request.filename or f'image_{timestamp}.jpg'
            filepath = os.path.join(self.save_dir, filename)

            cv2.imwrite(filepath, cv_image)

            response.success = True
            response.message = f'Saved image to {filepath}'
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response


def main():
    rclpy.init()
    node = ImageCaptureService()
    rclpy.spin(node)
    rclpy.shutdown()
