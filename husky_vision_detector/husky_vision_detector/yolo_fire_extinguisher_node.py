import rclpy
from rclpy.node import Node
import os

import cv2
import numpy as np
from ultralytics import YOLO

from husky_msgs.srv import InspectFireExtinguisher


class YoloFireExtinguisherNode(Node):

    CONF_THRESHOLD = 0.2
    OUTPUT_DIR = "/home/sharathnpayyadi/inspection_results"

    def __init__(self):
        super().__init__('yolo_fire_extinguisher_node')

        # -------------------------
        # LOAD YOLO MODEL
        # -------------------------
        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO("yolov8s-world.pt")

        self.get_logger().info("YOLO loaded, warming up...")
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = self.model(dummy, verbose=False)

        self.get_logger().info("YOLO ready for inspection")

        # Ensure output directory exists
        os.makedirs(self.OUTPUT_DIR, exist_ok=True)

        # -------------------------
        # SERVICE
        # -------------------------
        self.srv = self.create_service(
            InspectFireExtinguisher,
            '/inspect/fire_extinguisher',
            self.inspect_callback
        )

        self.get_logger().info("🔥 Fire Extinguisher Inspection Service READY")

    # -------------------------
    # SERVICE CALLBACK
    # -------------------------
    def inspect_callback(self, request, response):

        image_path = request.image_path
        self.get_logger().info(f"🔍 Inspecting image: {image_path}")

        # -------------------------
        # VALIDATE IMAGE PATH
        # -------------------------
        if not os.path.exists(image_path):
            self.get_logger().error("❌ Image path does not exist")
            response.present = False
            response.confidence = 0.0
            return response

        frame = cv2.imread(image_path)
        if frame is None:
            self.get_logger().error("❌ Failed to load image")
            response.present = False
            response.confidence = 0.0
            return response

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

                self.get_logger().info(
                    f"RAW DETECTION → label={label}, conf={conf:.2f}"
                )

                # v1 heuristic: bottle ≈ fire extinguisher
                if label in ["fire extinguisher", "bottle"] and conf >= self.CONF_THRESHOLD:
                    if conf > best_conf:
                        best_conf = conf
                        detected = True
                        best_bbox = box.xyxy[0].cpu().numpy().astype(int)

        # -------------------------
        # SAVE ANNOTATED IMAGE
        # -------------------------
        base_name = os.path.basename(image_path)
        name, _ = os.path.splitext(base_name)

        annotated_path = os.path.join(
            self.OUTPUT_DIR,
            f"{name}_annotated.jpg"
        )

        if detected and best_bbox is not None:
            x1, y1, x2, y2 = best_bbox

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"Extinguisher {best_conf:.2f}",
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
        else:
            cv2.putText(
                frame,
                "NO FIRE EXTINGUISHER DETECTED",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2
            )

        cv2.imwrite(annotated_path, frame)
        self.get_logger().info(f"🖼 Saved annotated image: {annotated_path}")

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
