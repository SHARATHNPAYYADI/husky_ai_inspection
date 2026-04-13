from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi import HTTPException
from datetime import datetime
from fastapi.staticfiles import StaticFiles
from pathlib import Path

from fastapi import WebSocket
from geometry_msgs.msg import Twist
import time

import os
import time
import json

import threading
from std_msgs.msg import String

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from husky_msgs.msg import MissionResult

app = FastAPI()
ros_node = None
camera_node = None
stop_service_node = None
report_generated = False
IMAGE_DIR = "/home/sharathnpayyadi/captured_images"
BASE_DIR = Path(__file__).resolve().parent.parent
FRONTEND_DIR = BASE_DIR / "frontend"
INSPECTION_DIR = "/home/sharathnpayyadi/inspection_results"

current_mission = {
    "mission_id": None,
    "results": []
}

import rclpy

if not rclpy.ok():
    rclpy.init()
    executor = MultiThreadedExecutor()

class MissionStatusSubscriber(Node):
    def __init__(self):
        super().__init__("mission_status_http_bridge")
        self.subscription = self.create_subscription(
            String,
            "/mission_status",
            self.status_callback,
            10
        )

    def status_callback(self, msg):
        robot_status["state"] = msg.data
        robot_status["last_updated"] = datetime.now().isoformat()

        if msg.data == "IDLE":
            robot_status["current_goal"] = None
        
        global report_generated

class MissionResultSubscriber(Node):
    def __init__(self):
        super().__init__("mission_result_bridge")

        self.subscription = self.create_subscription(
            MissionResult,
            "/mission_results",
            self.callback,
            10
        )

    def callback(self, msg):
        global current_mission

        # 🔥 If new mission → reset
        # if current_mission["mission_id"] != msg.mission_id:
        #     current_mission["mission_id"] = msg.mission_id
        #     current_mission["results"] = []

        # ---- Format time ----
        try:
            t = float(msg.timestamp)
            dt = datetime.fromtimestamp(t)
            time_str = dt.strftime("%H:%M:%S")
        except:
            time_str = msg.timestamp

        # ---- Append result ----
        current_mission["results"].append({
            "point": msg.goal_name,
            "status": "found" if msg.present else "missing",
            "confidence": msg.confidence,
            "image": f"/inspection_results/{os.path.basename(msg.image_path)}",
            "annotated": f"/inspection_results/{os.path.basename(msg.annotated_path)}",
            "time": time_str
        })

        global report_generated

        # 🔥 Trigger AFTER result is appended
        if "total_waypoints" in current_mission:
            if len(current_mission["results"]) == current_mission["total_waypoints"]:
                if not report_generated:
                    report_generated = True

                    self.get_logger().info("📝 All waypoints processed → generating report")

                    generate_report_internal()

        self.get_logger().info(
            f"📥 Result received: {msg.goal_name} → {'FOUND' if msg.present else 'MISSING'}"
        )

def start_ros_spin():
    node = MissionStatusSubscriber()
    rclpy.spin(node)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount(
    "/images",
    StaticFiles(directory=IMAGE_DIR),
    name="images"
)
app.mount(
    "/ui",
    StaticFiles(directory=str(FRONTEND_DIR), html=True),
    name="frontend"
)

app.mount(
    "/inspection_results",
    StaticFiles(directory=INSPECTION_DIR),
    name="inspection_results"
)

# -----------------------
# In-memory robot state
# -----------------------

robot_status = {
    "state": "IDLE",
    "current_goal": "None",
    "last_updated": datetime.now().isoformat()
}

logs = []
images = []
image_counter = 0


def log_event(message: str):
    logs.insert(0, {
        "timestamp": datetime.now().isoformat(),
        "message": message
    })


# -----------------------
# Control APIs
# -----------------------

@app.post("/go")
def go_to_waypoint(data: dict):
    waypoint = data.get("waypoint")

    if not waypoint:
        raise HTTPException(status_code=400, detail="Waypoint required")

    try:
        node = get_ros_client()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

    if not node.client.wait_for_service(timeout_sec=1.0):
        raise HTTPException(
            status_code=503,
            detail="/go_to_waypoint service not available"
        )

    request = node.client.srv_type.Request()
    request.name = waypoint

    future = node.client.call_async(request)

    import rclpy
    # rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    while not future.done():
        time.sleep(0.05)

    if future.result() is None:
        raise HTTPException(
            status_code=500,
            detail="ROS service call failed"
        )

    # Update mock status/logs (still useful!)
    robot_status["state"] = "MOVING"
    robot_status["current_goal"] = waypoint
    robot_status["last_updated"] = datetime.now().isoformat()
    log_event(f"ROS: moving to {waypoint}")

    return {"success": True}
@app.on_event("startup")
def start_ros_executor():

    def spin():
        executor.spin()

    mission_node = MissionStatusSubscriber()
    executor.add_node(mission_node)

    result_node = MissionResultSubscriber()
    executor.add_node(result_node)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

@app.post("/stop")
def stop_robot():
    try:
        node = get_stop_service_client()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

    if not node.client.wait_for_service(timeout_sec=1.0):
        raise HTTPException(
            status_code=503,
            detail="/stop_navigation service not available"
        )

    request = node.client.srv_type.Request()
    future = node.client.call_async(request)

    import rclpy
    # rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    while not future.done():
        time.sleep(0.05)

    response = future.result()
    if response is None or not response.success:
        raise HTTPException(
            status_code=500,
            detail=response.message if response else "Stop service failed"
        )

    # Update backend state
    robot_status["state"] = "IDLE"
    robot_status["current_goal"] = None
    robot_status["last_updated"] = datetime.now().isoformat()

    log_event("Navigation STOP requested")

    return {
        "success": True,
        "message": response.message
    }


@app.post("/capture")
def capture_image():
    try:
        node = get_camera_client()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

    if not node.client.wait_for_service(timeout_sec=1.0):
        raise HTTPException(
            status_code=503,
            detail="/capture_image service not available"
        )

    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    source = robot_status.get("current_goal") or "Manual"
    safe_source = source.replace(" ", "_")
    safe_source = safe_source.replace("_", "")

    filename = f"capture_{safe_source}_{timestamp}.jpg"

    request = node.client.srv_type.Request()
    request.filename = filename

    future = node.client.call_async(request)

    import rclpy
    # rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    while not future.done():
        time.sleep(0.05)

    response = future.result()
    if response is None or not response.success:
        raise HTTPException(
            status_code=500,
            detail=response.message if response else "Camera service failed"
        )

    # Store metadata for frontend
    img = {
        "id": len(images) + 1,
        "timestamp": datetime.now().isoformat(),
        "source": robot_status.get("current_goal") or "Manual",
        "thumbnail_url": f"/images/{filename}",
        "full_url": f"/images/{filename}"
    }

    images.insert(0, img)
    log_event(f"Image captured: {filename}")

    return {
        "success": True,
        "filename": filename,
        "message": response.message
    }

# @app.post("/inspect")
# def run_mission(data: dict):

#     waypoint = data.get("waypoint")

#     if not waypoint:
#         raise HTTPException(status_code=400, detail="Waypoint required")

#     node = get_mission_client()

#     if not node.client.wait_for_service(timeout_sec=1.0):
#         raise HTTPException(
#             status_code=503,
#             detail="/start_mission service not available"
#         )

#     request = node.client.srv_type.Request()
#     request.name = waypoint

#     future = node.client.call_async(request)
#     rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

#     response = future.result()
#     if response is None or not response.accepted:
#         raise HTTPException(
#             status_code=500,
#             detail=response.message if response else "Mission failed"
#         )

#     return {
#         "success": True,
#         "message": "Mission started"
#     }

@app.post("/inspect")
def run_mission(data: dict):
    global current_mission
    global report_generated

    report_generated = False

    
    waypoints = data.get("waypoints")

    current_mission = {
        "mission_id": None,
        "results": [],
        "total_waypoints": len(waypoints)
    }

    if not waypoints or not isinstance(waypoints, list):
        raise HTTPException(status_code=400, detail="Waypoints list required")

    log_event(f"Mission started: {waypoints}")

    # Update UI state
    robot_status["state"] = "MOVING"
    robot_status["current_goal"] = waypoints[0]

    node = get_mission_sequence_client()

    if not node.client.wait_for_service(timeout_sec=1.0):
        robot_status["state"] = "ERROR"
        raise HTTPException(
            status_code=503,
            detail="/start_mission_sequence not available"
        )

    request = node.client.srv_type.Request()
    request.waypoint_names = waypoints   # 🔥 KEY LINE

    future = node.client.call_async(request)

    while not future.done():
        time.sleep(0.05)

    response = future.result()

    if response is None or not response.success:
        robot_status["state"] = "ERROR"
        raise HTTPException(
            status_code=500,
            detail=response.message if response else "Mission failed"
        )

    return {
        "success": True,
        "message": "Mission sequence started"
    }
    
@app.post("/stop_mission")
def stop_mission():

    # 🔥 Cancel BT
    node1 = get_cancel_mission_client()

    if node1.client.wait_for_service(timeout_sec=1.0):
        future1 = node1.client.call_async(node1.client.srv_type.Request())
        while not future1.done():
            time.sleep(0.05)

    # 🔥 Cancel mission interface
    node2 = get_cancel_sequence_client()

    if node2.client.wait_for_service(timeout_sec=1.0):
        future2 = node2.client.call_async(node2.client.srv_type.Request())
        while not future2.done():
            time.sleep(0.05)

    log_event("Mission fully cancelled")

    return {
        "success": True,
        "message": "Mission cancelled (BT + queue cleared)"
    }


# -----------------------
# Data APIs
# -----------------------

@app.get("/status")
def get_status():
    return robot_status

@app.get("/mission_json")
def get_mission_json():
    return current_mission


@app.get("/logs")
def get_logs():
    return logs[:50]


@app.get("/images")
def get_images():
    result = []

    for idx, filename in enumerate(sorted(os.listdir(IMAGE_DIR), reverse=True)):

        if not filename.lower().endswith(".jpg"):
            continue

        filepath = os.path.join(IMAGE_DIR, filename)

        timestamp = datetime.fromtimestamp(
            os.path.getmtime(filepath)
        ).isoformat()
        parts = filename.split("_")

        if len(parts) >= 3:
            source = parts[1]
        else:
            source = "Unknown"
        result.append({
            "id": idx + 1,
            "timestamp": timestamp,
            "source": source,
            "thumbnail_url": f"/images/{filename}",
            "full_url": f"/images/{filename}"
        })

    return result

@app.get("/inspection_images")
def get_inspection_images():
    if not os.path.exists(INSPECTION_DIR):
        return []

    results = []

    for filename in sorted(os.listdir(INSPECTION_DIR), reverse=True):
        if not filename.lower().endswith(".jpg"):
            continue

        filepath = os.path.join(INSPECTION_DIR, filename)
        timestamp = datetime.fromtimestamp(
            os.path.getmtime(filepath)
        ).isoformat()

        results.append({
            "filename": filename,
            "timestamp": timestamp,
            "url": f"/inspection_results/{filename}"
        })

    return results

@app.get("/generate_report")
def generate_report():
    generate_report_internal()
    return {"success": True}

# @app.get("/generate_report")
# def generate_report():

#     global current_mission

#     if not current_mission["results"]:
#         return {"error": "No mission data available"}

#     # ---- Create missions folder ----
#     missions_dir = os.path.join(INSPECTION_DIR, "missions")
#     os.makedirs(missions_dir, exist_ok=True)

#     # ---- Create unique mission folder ----
#     folder_name = datetime.now().strftime("mission_%Y%m%d_%H%M%S")
#     mission_folder = os.path.join(missions_dir, folder_name)
#     os.makedirs(mission_folder, exist_ok=True)

#     # ---- Save JSON ----
#     json_path = os.path.join(mission_folder, "mission.json")

#     with open(json_path, "w") as f:
#         json.dump(current_mission, f, indent=2)

#     # ---- Summary ----
#     total_points = len(current_mission["results"])
#     detected = sum(1 for r in current_mission["results"] if r["status"] == "found")
#     missing = sum(1 for r in current_mission["results"] if r["status"] == "missing")

#     # ---- Generate rows ----
#     rows_html = ""

#     for r in current_mission["results"]:

#         status_class = "found" if r["status"] == "found" else "missing"
#         row_class = "missing-row" if r["status"] == "missing" else ""

#         rows_html += f"""
#         <tr class="{row_class}">
#             <td>{r['point']}</td>
#             <td>
#                 <a href="{r['annotated']}" target="_blank">
#                     <img src="{r['annotated']}">
#                 </a>
#             </td>
#             <td class="{status_class}">{r['status'].upper()}</td>
#             <td>{r['time']}</td>
#         </tr>
#         """

#     # ---- Load template ----
#     template_path = os.path.join(BASE_DIR, "report_template.html")

#     with open(template_path, "r") as f:
#         template = f.read()

#     # ---- Fill template ----
#     html_content = template.replace(
#         "{{generated_time}}",
#         datetime.now().strftime("%Y-%m-%d %H:%M:%S")
#     )
#     html_content = html_content.replace("{{total_points}}", str(total_points))
#     html_content = html_content.replace("{{inspected}}", str(total_points))
#     html_content = html_content.replace("{{detected}}", str(detected))
#     html_content = html_content.replace("{{missing}}", str(missing))
#     html_content = html_content.replace("{{rows}}", rows_html)

#     # ---- Save HTML ----
#     html_path = os.path.join(mission_folder, "report.html")

#     with open(html_path, "w") as f:
#         f.write(html_content)

#     return {
#         "success": True,
#         "mission_folder": mission_folder,
#         "report_url": f"/inspection_results/missions/{folder_name}/report.html"
#     }


@app.websocket("/ws/teleop")
async def teleop_ws(ws: WebSocket):
    await ws.accept()
    node = get_cmd_vel_node()

    try:
        while True:
            data = await ws.receive_json()

            twist = Twist()
            twist.linear.x = float(data.get("linear", 0.0)) * 0.5
            twist.angular.z = float(data.get("angular", 0.0)) * 1.0

            node.pub.publish(twist)

    except Exception:
        # Stop robot if client disconnects or errors
        node.pub.publish(Twist())
def register_node(node):
    executor.add_node(node)
    return node
def get_ros_client():
    global ros_node

    if ros_node is not None:
        return ros_node

    import rclpy
    from rclpy.node import Node
    from husky_msgs.srv import GoToWaypoint

    # rclpy.init()

    class WaypointClient(Node):
        def __init__(self):
            super().__init__("go_to_waypoint_http_bridge")
            self.client = self.create_client(
                GoToWaypoint,
                "/go_to_waypoint"
            )

    # ros_node = WaypointClient()
    ros_node = register_node(WaypointClient())
    return ros_node

def generate_report_internal():
    global current_mission

    if not current_mission["results"]:
        print("No mission data, skipping report")
        return

    missions_dir = os.path.join(INSPECTION_DIR, "missions")
    os.makedirs(missions_dir, exist_ok=True)

    folder_name = datetime.now().strftime("mission_%Y%m%d_%H%M%S")
    mission_folder = os.path.join(missions_dir, folder_name)
    os.makedirs(mission_folder, exist_ok=True)

    # Save JSON
    json_path = os.path.join(mission_folder, "mission.json")
    with open(json_path, "w") as f:
        json.dump(current_mission, f, indent=2)

    # Generate HTML (same as before)
    # (reuse your existing logic here)

    print(f"✅ Report saved: {mission_folder}")

def get_camera_client():
    global camera_node

    if camera_node is not None:
        return camera_node

    import rclpy
    from rclpy.node import Node
    from husky_msgs.srv import CaptureImage

    # if not rclpy.ok():
    #     rclpy.init()

    class CameraClient(Node):
        def __init__(self):
            super().__init__("camera_http_bridge")
            self.client = self.create_client(
                CaptureImage,
                "/capture_image"
            )

    # camera_node = CameraClient()
    camera_node = register_node(CameraClient())
    return camera_node
def get_stop_service_client():
    global stop_service_node

    if stop_service_node is not None:
        return stop_service_node

    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    # if not rclpy.ok():
    #     rclpy.init()

    class StopServiceNode(Node):
        def __init__(self):
            super().__init__("stop_navigation_http_bridge")
            self.client = self.create_client(
                Trigger,
                "/stop_navigation"
            )

    # stop_service_node =  StopServiceNode()
    stop_service_node =  register_node(StopServiceNode())
    return stop_service_node

def get_cmd_vel_node():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    # if not rclpy.ok():
    #     rclpy.init()

    class CmdVelNode(Node):
        def __init__(self):
            super().__init__("web_teleop_cmd_vel")
            self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    if not hasattr(get_cmd_vel_node, "node"):
        get_cmd_vel_node.node = CmdVelNode()

    return get_cmd_vel_node.node

def get_inspection_client():
    import rclpy
    from rclpy.node import Node
    from husky_msgs.srv import InspectFireExtinguisher

    if hasattr(get_inspection_client, "node"):
        return get_inspection_client.node

    class InspectionClient(Node):
        def __init__(self):
            super().__init__("inspect_fire_extinguisher_http_bridge")
            self.client = self.create_client(
                InspectFireExtinguisher,
                "/inspect/fire_extinguisher"
            )

    get_inspection_client.node = InspectionClient()
    return get_inspection_client.node

def get_mission_client():
    import rclpy
    from rclpy.node import Node
    from husky_msgs.srv import GoToWaypoint

    if hasattr(get_mission_client, "node"):
        return get_mission_client.node

    class MissionClient(Node):
        def __init__(self):
            super().__init__("mission_http_bridge")
            self.client = self.create_client(
                GoToWaypoint,
                "/start_mission"
            )

    # get_mission_client.node = MissionClient()
    get_mission_client.node =register_node(MissionClient())
    return get_mission_client.node

def get_cancel_mission_client():
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    if hasattr(get_cancel_mission_client, "node"):
        return get_cancel_mission_client.node

    class CancelMissionClient(Node):
        def __init__(self):
            super().__init__("cancel_mission_http_bridge")
            self.client = self.create_client(
                Trigger,
                "/cancel_mission"
            )

    get_cancel_mission_client.node = register_node(CancelMissionClient())
    return get_cancel_mission_client.node

def get_cancel_sequence_client():
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    if hasattr(get_cancel_sequence_client, "node"):
        return get_cancel_sequence_client.node

    class CancelSeqClient(Node):
        def __init__(self):
            super().__init__("cancel_sequence_http_bridge")
            self.client = self.create_client(
                Trigger,
                "/cancel_mission_sequence"
            )

    get_cancel_sequence_client.node = register_node(CancelSeqClient())
    return get_cancel_sequence_client.node

def get_mission_sequence_client():
    import rclpy
    from rclpy.node import Node
    from husky_msgs.srv import StartMissionSequence

    if hasattr(get_mission_sequence_client, "node"):
        return get_mission_sequence_client.node

    class MissionSequenceClient(Node):
        def __init__(self):
            super().__init__("mission_sequence_http_bridge")
            self.client = self.create_client(
                StartMissionSequence,
                "/start_mission_sequence"
            )

    get_mission_sequence_client.node = register_node(MissionSequenceClient())
    return get_mission_sequence_client.node
