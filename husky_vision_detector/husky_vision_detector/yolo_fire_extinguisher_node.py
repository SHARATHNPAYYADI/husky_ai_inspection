import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from husky_msgs.srv import InspectFireExtinguisher

import cv2
import os
import re
import numpy as np
from datetime import datetime
from ultralytics import YOLO


MODEL_PATH = os.getenv(
    "HUSKY_FIRE_EXT_MODEL",
    os.path.expanduser("~/husky_ws/src/husky_ai_inspection/husky_models/fire_extinguisher_yolo.pt")
)


class FireExtinguisherInspectionNode(Node):

    CONF_THRESHOLD = 0.2

    def __init__(self):
        super().__init__('fire_extinguisher_inspection_node')

        # ----------------------------
        # Camera Setup
        # ----------------------------
        self.bridge = CvBridge()
        self.latest_image = None

        self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            10
        )

        # ----------------------------
        # YOLO Model Load
        # ----------------------------
        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO(MODEL_PATH)

        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = self.model(dummy, verbose=False)

        self.get_logger().info("YOLO model ready.")

        # ----------------------------
        # Directory Setup
        # ----------------------------
        self.capture_dir = os.path.expanduser("~/captured_images")
        self.annotated_dir = os.path.expanduser("~/inspection_results")

        os.makedirs(self.capture_dir, exist_ok=True)
        os.makedirs(self.annotated_dir, exist_ok=True)

        # ----------------------------
        # Service
        # ----------------------------
        self.srv = self.create_service(
            InspectFireExtinguisher,
            '/inspect/fire_extinguisher',
            self.inspect_callback
        )

        self.get_logger().info("🔥 Inspection Service READY")

    # -------------------------------------------------
    # Camera Callback
    # -------------------------------------------------
    def image_callback(self, msg):
        self.latest_image = msg

    # -------------------------------------------------
    # Inspection Service Callback
    # -------------------------------------------------
    def inspect_callback(self, request, response):

        if self.latest_image is None:
            self.get_logger().error("No image received yet.")
            response.present = False
            response.confidence = 0.0
            response.image_path = ""
            response.annotated_path = ""
            return response

        try:
            # Convert ROS → OpenCV
            frame = self.bridge.imgmsg_to_cv2(
                self.latest_image,
                desired_encoding='bgr8'
            )

            # ----------------------------
            # Safe Goal Name
            # ----------------------------
            goal_name = request.goal_name or "Manual"
            safe_goal = re.sub(r'[^a-zA-Z0-9\-]', '', goal_name)

            # ----------------------------
            # Timestamp (Robot-side)
            # ----------------------------
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            base_filename = f"inspect_{safe_goal}_{timestamp}"

            raw_path = os.path.join(
                self.capture_dir,
                f"{base_filename}.jpg"
            )

            annotated_path = os.path.join(
                self.annotated_dir,
                f"{base_filename}_annotated.jpg"
            )

            # ----------------------------
            # Save Raw Image
            # ----------------------------
            cv2.imwrite(raw_path, frame)
            self.get_logger().info(f"Saved raw image: {raw_path}")

            # ----------------------------
            # YOLO Inference
            # ----------------------------
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

                    self.get_logger().info(
                        f"Detection → label={label}, conf={conf:.2f}"
                    )

                    if label == "fire_extinguisher" and conf >= self.CONF_THRESHOLD:
                        if conf > best_conf:
                            best_conf = conf
                            detected = True
                            best_bbox = box.xyxy[0].cpu().numpy().astype(int)

            # ----------------------------
            # Draw Annotation
            # ----------------------------
            annotated_frame = frame.copy()

            if detected and best_bbox is not None:
                x1, y1, x2, y2 = best_bbox

                cv2.rectangle(
                    annotated_frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    annotated_frame,
                    f"Extinguisher {best_conf:.2f}",
                    (x1, max(y1 - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                self.get_logger().info(
                    f"✅ Fire extinguisher detected (conf={best_conf:.2f})"
                )

            else:
                cv2.putText(
                    annotated_frame,
                    "NO FIRE EXTINGUISHER DETECTED",
                    (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2
                )

                self.get_logger().warn(
                    "⚠ Fire extinguisher NOT detected"
                )

            # Save annotated image
            cv2.imwrite(annotated_path, annotated_frame)
            self.get_logger().info(
                f"Saved annotated image: {annotated_path}"
            )

            # ----------------------------
            # Fill Response
            # ----------------------------
            response.present = detected
            response.confidence = float(best_conf)
            response.image_path = raw_path
            response.annotated_path = annotated_path

        except Exception as e:
            self.get_logger().error(f"Inspection failed: {str(e)}")
            response.present = False
            response.confidence = 0.0
            response.image_path = ""
            response.annotated_path = ""

        return response


def main():
    rclpy.init()
    node = FireExtinguisherInspectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
