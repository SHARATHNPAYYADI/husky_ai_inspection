import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import google.generativeai as genai
from PIL import Image as PILImage
import cv2
import json
import re

# -------------------------
# JSON extractor (UNCHANGED)
# -------------------------
def extract_json(text: str):
    match = re.search(r"\{.*\}", text, re.DOTALL)
    if not match:
        raise ValueError("No JSON object found in model output")
    return json.loads(match.group(0))


class FireExtinguisherNode(Node):

    def __init__(self):
        super().__init__('fire_extinguisher_node')

        # -------------------------
        # CONFIG
        # -------------------------
        API_KEY = "AIzaSyBVCpVeCKT6vFmwJcwHb2iYqvdxsCOqAAU"   # 🔐 put your key here

        genai.configure(api_key=API_KEY)

        self.model = genai.GenerativeModel(
            model_name="models/gemini-2.5-flash"
        )

        self.prompt = """
You are an industrial safety inspection AI.

Task:
Determine whether the image contains a FIRE EXTINGUISHER.

If present:
- Return a bounding box around the fire extinguisher
- Bounding box must tightly cover the extinguisher

Rules:
- Answer strictly in JSON
- Do not explain anything
- Use normalized coordinates (0.0 to 1.0)

JSON format:
{
  "fire_extinguisher": true or false,
  "bounding_box": {
    "x_min": float,
    "y_min": float,
    "x_max": float,
    "y_max": float
  } OR null
}
"""

        self.bridge = CvBridge()

        # -------------------------
        # STATE
        # -------------------------
        self.latest_image_msg = None
        self.last_call_time = None

        # -------------------------
        # ROS2 I/O
        # -------------------------
        self.sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            10
        )

        self.pub_image = self.create_publisher(
            Image,
            '/inspection/image',
            10
        )

        self.pub_json = self.create_publisher(
            String,
            '/inspection/report',
            10
        )

        # 🔑 Run inspection every 60 seconds (safe for Gemini free tier)
        self.timer = self.create_timer(60.0, self.run_inspection)

        self.get_logger().info("🔥 Fire extinguisher inspection node started")

    # -------------------------
    # CAMERA CALLBACK
    # -------------------------
    def image_callback(self, msg):
        self.latest_image_msg = msg

    # -------------------------
    # INSPECTION LOGIC
    # -------------------------
    def run_inspection(self):

        if self.latest_image_msg is None:
            self.get_logger().warn("📷 Waiting for camera images...")
            return

        self.get_logger().info("🔍 Running fire extinguisher inspection")

        try:
            # ROS → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(
                self.latest_image_msg, 'bgr8'
            )
            height, width, _ = cv_image.shape

            # OpenCV → PIL
            pil_image = PILImage.fromarray(
                cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            )

            # -------------------------
            # GEMINI CALL
            # -------------------------
            response = self.model.generate_content(
                [self.prompt, pil_image],
                generation_config={"temperature": 0.1}
            )

            result = extract_json(response.text)

            # Publish JSON
            self.pub_json.publish(
                String(data=json.dumps(result))
            )

            # -------------------------
            # DRAW BOUNDING BOX
            # -------------------------
            if result["fire_extinguisher"] and result["bounding_box"]:

                bb = result["bounding_box"]

                x_min = int(bb["x_min"] * width)
                y_min = int(bb["y_min"] * height)
                x_max = int(bb["x_max"] * width)
                y_max = int(bb["y_max"] * height)

                cv2.rectangle(
                    cv_image,
                    (x_min, y_min),
                    (x_max, y_max),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    cv_image,
                    "Fire Extinguisher",
                    (x_min, max(0, y_min - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

            # OpenCV → ROS
            out_msg = self.bridge.cv2_to_imgmsg(
                cv_image, 'bgr8'
            )
            self.pub_image.publish(out_msg)

            self.get_logger().info("✅ Inspection completed successfully")

        except Exception as e:
            self.get_logger().error(f"❌ Inspection failed: {e}")


def main():
    rclpy.init()
    node = FireExtinguisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
