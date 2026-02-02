import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from husky_msgs.srv import InspectFireExtinguisher

import cv2
import numpy as np
from ultralytics import YOLO


class YoloFireExtinguisherNode(Node):

    CONF_THRESHOLD = 0.6
    IMAGE_TIMEOUT_SEC = 2.0

    def __init__(self):
        super().__init__('yolo_fire_extinguisher_node')

        # -------------------------
        # LOAD YOLO MODEL
        # -------------------------
        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO("yolov8n.pt")
        self.get_logger().info("YOLO loaded, warming up...")

        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = self.model(dummy, verbose=False)

        self.get_logger().info("YOLO ready for inspection")

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

        self.get_logger().info("🔥 YOLOv8 Fire Extinguisher Inspection Node ready")

    # -------------------------
    # CAMERA CALLBACK
    # -------------------------
    def image_callback(self, msg):
        self.latest_image = msg

    # -------------------------
    # WAIT FOR FRESH IMAGE
    # -------------------------
    def wait_for_image(self):
        self.latest_image = None
        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_image is not None:
                return self.latest_image
            if time.time() - start > self.IMAGE_TIMEOUT_SEC:
                return None

    # -------------------------
    # SERVICE CALLBACK
    # -------------------------
    def inspect_callback(self, request, response):

        self.get_logger().info("🔍 Fire extinguisher inspection triggered")

        # Get fresh frame
        image_msg = self.wait_for_image()
        if image_msg is None:
            self.get_logger().error("❌ Failed to capture image")
            response.present = False
            response.confidence = 0.0
            return response

        # ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # -------------------------
        # YOLO INFERENCE
        # -------------------------
        results = self.model(frame, verbose=False)

        detected = False
        best_conf = 0.0
        best_bbox = None

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id].lower()

                if label == "fire extinguisher" and conf >= self.CONF_THRESHOLD:
                    if conf > best_conf:
                        best_conf = conf
                        detected = True
                        best_bbox = box.xyxy[0].cpu().numpy().astype(int)

        # -------------------------
        # OPTIONAL VISUALIZATION
        # -------------------------
        if detected and best_bbox is not None:
            x1, y1, x2, y2 = best_bbox
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"Fire Extinguisher {best_conf:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            timestamp = int(time.time())
            cv2.imwrite(
                f"/tmp/fire_extinguisher_{timestamp}.jpg",
                frame
            )

        # -------------------------
        # RESPONSE
        # -------------------------
        response.present = detected
        response.confidence = float(best_conf)

        if detected:
            self.get_logger().info(
                f"✅ Fire extinguisher detected (conf={best_conf:.2f})"
            )
        else:
            self.get_logger().warn("⚠ Fire extinguisher NOT detected")

        return response


def main():
    rclpy.init()
    node = YoloFireExtinguisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
