#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "navigate_to_pose_bt.cpp"
#include "wait_for_waypoint_bt.cpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("husky_mission_bt");

  BT::BehaviorTreeFactory factory;

  // 🔴 THIS IS THE MISSING PIECE

  factory.registerBuilder<NavigateToPoseBT>(
    "NavigateToPoseBT",
    [node](const std::string & name,
           const BT::NodeConfiguration & config)
    {
      return std::make_unique<NavigateToPoseBT>(
        name, config, node);
    });
    factory.registerBuilder<WaitForWaypointBT>(
    "WaitForWaypointBT",
    [node](const std::string& name,
            const BT::NodeConfiguration& config)
    {
        return std::make_unique<WaitForWaypointBT>(
        name, config, node);
    });

  auto xml_path =
    ament_index_cpp::get_package_share_directory(
      "husky_mission_bt") +
    "/bt_xml/mission_nav_only.xml";

  auto tree = factory.createTreeFromFile(xml_path);

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
