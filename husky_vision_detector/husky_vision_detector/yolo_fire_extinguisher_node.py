import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from husky_msgs.srv import InspectFireExtinguisher

import cv2
import numpy as np
from ultralytics import YOLO


class YoloFireExtinguisherNode(Node):

    def __init__(self):
        super().__init__('yolo_fire_extinguisher_node')

        # -------------------------
        # LOAD YOLO MODEL
        # -------------------------
        # You can later replace with a fine-tuned model
        self.model = YOLO("yolov8n.pt")

        self.bridge = CvBridge()
        self.latest_image = None

        # -------------------------
        # SUBSCRIBER
        # -------------------------
        self.image_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            10
        )

        # -------------------------
        # SERVICE
        # -------------------------
        self.srv = self.create_service(
            InspectFireExtinguisher,
            '/inspect/fire_extinguisher',
            self.inspect_callback
        )

        self.get_logger().info("🔥 YOLOv8 Fire Extinguisher Inspection Node started")

    # -------------------------
    # CAMERA CALLBACK
    # -------------------------
    def image_callback(self, msg):
        self.latest_image = msg

    # -------------------------
    # SERVICE CALLBACK
    # -------------------------
    def inspect_callback(self, request, response):

        if self.latest_image is None:
            self.get_logger().warn("No image available yet")
            response.present = False
            response.confidence = 0.0
            return response

        self.get_logger().info("🔍 Inspection service triggered")

        # ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(
            self.latest_image, 'bgr8'
        )

        # -------------------------
        # YOLO INFERENCE
        # -------------------------
        results = self.model(frame, verbose=False)

        detected = False
        best_conf = 0.0

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id]

                # COCO label name
                if label.lower() == "fire extinguisher":
                    detected = True
                    best_conf = max(best_conf, conf)

        response.present = detected
        response.confidence = best_conf

        self.get_logger().info(
            f"✅ Fire extinguisher present: {detected} (conf={best_conf:.2f})"
        )

        return response


def main():
    rclpy.init()
    node = YoloFireExtinguisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
