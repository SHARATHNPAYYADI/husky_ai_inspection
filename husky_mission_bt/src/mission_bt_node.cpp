#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "navigate_to_pose_bt.cpp"
#include "wait_for_mission_goal_bt.cpp"
#include "inspect_fire_extinguisher_bt.cpp"

#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("husky_mission_bt");

  BT::BehaviorTreeFactory factory;

  factory.registerBuilder<NavigateToPoseBT>(
    "NavigateToPoseBT",
    [node](const std::string & name,
           const BT::NodeConfiguration & config)
    {
      return std::make_unique<NavigateToPoseBT>(name, config, node);
    });

  factory.registerBuilder<WaitForMissionGoalBT>(
    "WaitForMissionGoalBT",
    [node](const std::string& name,
           const BT::NodeConfiguration& config)
    {
      return std::make_unique<WaitForMissionGoalBT>(name, config, node);
    });

  factory.registerBuilder<InspectFireExtinguisherBT>(
    "InspectFireExtinguisherBT",
    [node](const std::string& name,
           const BT::NodeConfiguration& config)
    {
      return std::make_unique<InspectFireExtinguisherBT>(name, config, node);
    });

  auto xml_path =
    ament_index_cpp::get_package_share_directory("husky_mission_bt") +
    "/bt_xml/mission_nav_only.xml";

  auto tree = factory.createTreeFromFile(xml_path);

  // 🔥 Direct Nav2 Action Client for Emergency Cancel
  auto nav2_client =
    rclcpp_action::create_client<NavigateToPose>(
      node, "navigate_to_pose");

  // Cancel Mission Service
  auto cancel_srv =
    node->create_service<std_srvs::srv::Trigger>(
      "/cancel_mission",
      [node, &tree, nav2_client](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        RCLCPP_WARN(node->get_logger(),
                    "Mission cancel requested");

        // 1️⃣ Cancel Nav2 directly
        if (nav2_client->wait_for_action_server(std::chrono::seconds(1))) {
          nav2_client->async_cancel_all_goals();
          RCLCPP_WARN(node->get_logger(),
                      "Nav2 goals cancelled directly");
        }

        // 2️⃣ Halt BT
        tree.haltTree();
        tree.rootBlackboard()->clear();

        // 3️⃣ Force stop cmd_vel
        auto stop_pub =
          node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        geometry_msgs::msg::Twist stop_msg;
        stop_pub->publish(stop_msg);

        response->success = true;
        response->message = "Mission cancelled";
      });

  rclcpp::Rate rate(20);

  while (rclcpp::ok()) {

    auto status = tree.tickRoot();

    if (status == BT::NodeStatus::SUCCESS ||
        status == BT::NodeStatus::FAILURE)
    {
      tree.haltTree();
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
