#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "husky_msgs/srv/go_to_waypoint.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <yaml-cpp/yaml.h>

class WaitForWaypointBT : public BT::StatefulActionNode
{
public:
  WaitForWaypointBT(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config),
    node_(node)
  {
    load_waypoints();

    service_ = node_->create_service<husky_msgs::srv::GoToWaypoint>(
      "go_to_waypoint",
      std::bind(
        &WaitForWaypointBT::service_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

  BT::NodeStatus onStart() override
  {
    if (goal_ready_) {
      setOutput("goal", goal_);
      goal_ready_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (goal_ready_) {
      setOutput("goal", goal_);
      goal_ready_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  void service_callback(
    const std::shared_ptr<husky_msgs::srv::GoToWaypoint::Request> req,
    std::shared_ptr<husky_msgs::srv::GoToWaypoint::Response> res)
  {
    if (req->names.empty()) {
      res->accepted = false;
      res->message = "At least one waypoint is required";
      return;
    }

    const auto & name = req->names.front();
    auto it = waypoints_.find(name);
    if (it == waypoints_.end()) {
      res->accepted = false;
      res->message = "Waypoint not found";
      return;
    }

    goal_ = it->second;
    goal_ready_ = true;

    res->accepted = true;
    res->message = "Waypoint accepted";
    RCLCPP_INFO(node_->get_logger(),
      "BT: Waypoint '%s' accepted", name.c_str());
  }

 void load_waypoints()
  {
    auto pkg =
      ament_index_cpp::get_package_share_directory(
        "husky_waypoint_recorder");

    auto yaml_path = pkg + "/config/waypoints.yaml";
    YAML::Node yaml = YAML::LoadFile(yaml_path);

    auto wps = yaml["waypoints"];
    if (!wps || !wps.IsMap()) {
      throw std::runtime_error(
        "waypoints.yaml: 'waypoints' must be a map");
    }

    for (auto it = wps.begin(); it != wps.end(); ++it) {
      const std::string name = it->first.as<std::string>();
      const YAML::Node wp = it->second;

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";

      pose.pose.position.x = wp["x"].as<double>();
      pose.pose.position.y = wp["y"].as<double>();

      double yaw = wp["yaw"].as<double>();
      pose.pose.orientation.z = sin(yaw / 2.0);
      pose.pose.orientation.w = cos(yaw / 2.0);

      waypoints_[name] = pose;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Loaded %zu waypoints",
      waypoints_.size());
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<husky_msgs::srv::GoToWaypoint>::SharedPtr service_;

  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  geometry_msgs::msg::PoseStamped goal_;
  bool goal_ready_{false};
};
