#include <memory>
#include <string>
#include <unordered_map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "husky_msgs/srv/go_to_waypoint.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

class MissionInterfaceNode : public rclcpp::Node
{
public:
  MissionInterfaceNode()
  : Node("mission_interface_node")
  {
    load_waypoints();

    // QoS: latched mission goal
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();

    goal_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mission_goal", qos);

    goto_srv_ =
      this->create_service<husky_msgs::srv::GoToWaypoint>(
        "/start_mission",
        std::bind(
          &MissionInterfaceNode::goto_cb,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Mission Interface Node ready (using GoToWaypoint.srv)");
  }

private:
  // --------------------------------------------------
  // Load waypoints.yaml
  // --------------------------------------------------
  void load_waypoints()
  {
    auto pkg_path =
      ament_index_cpp::get_package_share_directory(
        "husky_waypoint_recorder");

    auto yaml_path = pkg_path + "/config/waypoints.yaml";

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
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);

      waypoints_[name] = pose;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded %zu waypoints",
      waypoints_.size());
  }

  // --------------------------------------------------
  // Service callback (GoToWaypoint)
  // --------------------------------------------------
  void goto_cb(
    const std::shared_ptr<husky_msgs::srv::GoToWaypoint::Request> req,
    std::shared_ptr<husky_msgs::srv::GoToWaypoint::Response> res)
  {
    auto it = waypoints_.find(req->name);
    if (it == waypoints_.end()) {
      res->accepted = false;
      res->message = "Waypoint not found";
      RCLCPP_WARN(
        this->get_logger(),
        "Mission rejected: unknown waypoint '%s'",
        req->name.c_str());
      return;
    }

    auto goal = it->second;
    goal.header.stamp = this->now();

    goal_pub_->publish(goal);

    res->accepted = true;
    res->message = "Mission goal published";

    RCLCPP_INFO(
      this->get_logger(),
      "Mission goal published for waypoint '%s'",
      req->name.c_str());
  }

  // --------------------------------------------------
  // Members
  // --------------------------------------------------
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Service<husky_msgs::srv::GoToWaypoint>::SharedPtr goto_srv_;

  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
