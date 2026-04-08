#include <memory>
#include <string>
#include <queue>
#include <unordered_map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "husky_msgs/srv/go_to_waypoint.hpp"
#include "husky_msgs/srv/start_mission_sequence.hpp"   // NEW
#include "husky_msgs/msg/mission_goal.hpp"
#include "husky_msgs/msg/mission_complete.hpp"         // NEW

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

class MissionInterfaceNode : public rclcpp::Node
{
public:
  MissionInterfaceNode()
  : Node("mission_interface_node")
  {
    load_waypoints();

    // ----------------------------------------------------------------
    // Publisher: /mission_goal  (latched — transient_local)
    // ----------------------------------------------------------------
    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.reliable();
    latched_qos.transient_local();

    goal_pub_ = this->create_publisher<husky_msgs::msg::MissionGoal>(
      "/mission_goal", latched_qos);

    // ----------------------------------------------------------------
    // Service 1: /start_mission  (single waypoint — unchanged)
    // ----------------------------------------------------------------
    goto_srv_ = this->create_service<husky_msgs::srv::GoToWaypoint>(
      "/start_mission",
      std::bind(&MissionInterfaceNode::goto_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    // ----------------------------------------------------------------
    // Service 2: /start_mission_sequence  (NEW — ordered list)
    // ----------------------------------------------------------------
    sequence_srv_ = this->create_service<husky_msgs::srv::StartMissionSequence>(
      "/start_mission_sequence",
      std::bind(&MissionInterfaceNode::sequence_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    // ----------------------------------------------------------------
    // Subscriber: /mission_goal_complete  (NEW — BT signals done)
    // ----------------------------------------------------------------
    complete_sub_ = this->create_subscription<husky_msgs::msg::MissionComplete>(
      "/mission_goal_complete",
      rclcpp::QoS(10).reliable(),
      std::bind(&MissionInterfaceNode::complete_cb, this, std::placeholders::_1));

    cancel_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/cancel_mission_sequence",
      std::bind(&MissionInterfaceNode::cancel_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "Mission Interface Node ready  |  services: /start_mission, /start_mission_sequence");
  }

private:
  // ====================================================================
  // Waypoint loading
  // ====================================================================
  void load_waypoints()
  {
    auto pkg_path = ament_index_cpp::get_package_share_directory(
      "husky_waypoint_recorder");
    auto yaml_path = pkg_path + "/config/waypoints.yaml";

    YAML::Node yaml = YAML::LoadFile(yaml_path);
    auto wps = yaml["waypoints"];

    if (!wps || !wps.IsMap()) {
      throw std::runtime_error("waypoints.yaml: 'waypoints' must be a map");
    }

    for (auto it = wps.begin(); it != wps.end(); ++it) {
      const std::string name = it->first.as<std::string>();
      const YAML::Node wp   = it->second;

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = wp["x"].as<double>();
      pose.pose.position.y = wp["y"].as<double>();

      double yaw = wp["yaw"].as<double>();
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);

      waypoints_[name] = pose;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());
  }

  // ====================================================================
  // Service 1: /start_mission  (single waypoint — existing behaviour)
  // ====================================================================
  void goto_cb(
    const std::shared_ptr<husky_msgs::srv::GoToWaypoint::Request> req,
    std::shared_ptr<husky_msgs::srv::GoToWaypoint::Response> res)
  {
    if (mission_running_) {
      res->accepted = false;
      res->message  = "A mission sequence is already running";
      RCLCPP_WARN(this->get_logger(),
        "/start_mission rejected: sequence already in progress");
      return;
    }

    auto it = waypoints_.find(req->name);
    if (it == waypoints_.end()) {
      res->accepted = false;
      res->message  = "Waypoint '" + req->name + "' not found";
      return;
    }

    publish_goal(req->name, it->second);

    res->accepted = true;
    res->message  = "Single-waypoint mission published";
  }

  // ====================================================================
  // Service 2: /start_mission_sequence  (NEW — multi-waypoint)
  // ====================================================================
  void sequence_cb(
    const std::shared_ptr<husky_msgs::srv::StartMissionSequence::Request> req,
    std::shared_ptr<husky_msgs::srv::StartMissionSequence::Response> res)
  {
    if (mission_running_) {
      res->accepted = false;
      res->message  = "A mission sequence is already running";
      RCLCPP_WARN(this->get_logger(),
        "/start_mission_sequence rejected: already running");
      return;
    }

    if (req->waypoint_names.empty()) {
      res->accepted = false;
      res->message  = "waypoint_names list is empty";
      return;
    }

    // Validate all names before accepting
    for (const auto & name : req->waypoint_names) {
      if (waypoints_.find(name) == waypoints_.end()) {
        res->accepted = false;
        res->message  = "Unknown waypoint: '" + name + "'";
        RCLCPP_WARN(this->get_logger(),
          "Sequence rejected: unknown waypoint '%s'", name.c_str());
        return;
      }
    }

    // Build queue
    while (!mission_queue_.empty()) mission_queue_.pop();  // clear any stale state
    for (const auto & name : req->waypoint_names) {
      mission_queue_.push(name);
    }

    mission_running_ = true;
    total_waypoints_ = mission_queue_.size();
    completed_waypoints_ = 0;

    RCLCPP_INFO(this->get_logger(),
      "Mission sequence accepted: %zu waypoints", total_waypoints_);

    publish_next_goal();  // kick off first waypoint

    res->accepted = true;
    res->message  = "Mission sequence started (" +
                    std::to_string(total_waypoints_) + " waypoints)";
  }

  // ====================================================================
  // Subscriber: /mission_goal_complete  (BT finished a waypoint)
  // ====================================================================
  void complete_cb(const husky_msgs::msg::MissionComplete::SharedPtr msg)
  {
    // Guard: only process if it matches the currently active mission_id
    if (msg->mission_id != current_mission_id_) {
      RCLCPP_WARN(this->get_logger(),
        "complete_cb: got mission_id=%lu but expected %lu — ignoring",
        msg->mission_id, current_mission_id_);
      return;
    }

    completed_waypoints_++;

    RCLCPP_INFO(this->get_logger(),
      "Waypoint %lu/%zu done (success=%s)",
      completed_waypoints_, total_waypoints_,
      msg->success ? "true" : "false");

    if (mission_queue_.empty()) {
      // All waypoints done
      mission_running_ = false;
      RCLCPP_INFO(this->get_logger(),
        "===  Mission sequence complete: all %zu waypoints visited  ===",
        total_waypoints_);
      return;
    }

    publish_next_goal();
  }

  // ====================================================================
  // Helpers
  // ====================================================================
  void publish_next_goal()
  {
    const std::string name = mission_queue_.front();
    mission_queue_.pop();
    publish_goal(name, waypoints_.at(name));
  }

  void publish_goal(
    const std::string & name,
    const geometry_msgs::msg::PoseStamped & pose)
  {
    current_mission_id_++;

    husky_msgs::msg::MissionGoal msg;
    msg.mission_id = current_mission_id_;
    msg.goal_name  = name;
    msg.pose       = pose;
    msg.pose.header.stamp = this->now();

    goal_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
      "Published goal: name='%s'  mission_id=%lu  (queue remaining: %zu)",
      name.c_str(), current_mission_id_, mission_queue_.size());
  }

  void cancel_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_WARN(this->get_logger(), "MissionInterface: CANCEL RECEIVED");

    // 🔥 Stop mission
    mission_running_ = false;

    // 🔥 Clear queue
    while (!mission_queue_.empty()) {
      mission_queue_.pop();
    }

    total_waypoints_ = 0;
    completed_waypoints_ = 0;

    res->success = true;
    res->message = "Mission sequence cancelled and cleared";
  }

  // ====================================================================
  // Members
  // ====================================================================

  // Publishers / Services / Subscribers
  rclcpp::Publisher<husky_msgs::msg::MissionGoal>::SharedPtr      goal_pub_;
  rclcpp::Service<husky_msgs::srv::GoToWaypoint>::SharedPtr        goto_srv_;
  rclcpp::Service<husky_msgs::srv::StartMissionSequence>::SharedPtr sequence_srv_;
  rclcpp::Subscription<husky_msgs::msg::MissionComplete>::SharedPtr complete_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;

  // Waypoint map loaded from YAML
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  // Mission state
  std::queue<std::string> mission_queue_;
  uint64_t  current_mission_id_{0};
  bool      mission_running_{false};
  size_t    total_waypoints_{0};
  size_t    completed_waypoints_{0};
};

// ======================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}