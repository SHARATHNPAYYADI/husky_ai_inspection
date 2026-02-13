#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "husky_msgs/msg/mission_goal.hpp"

class WaitForMissionGoalBT : public BT::StatefulActionNode
{
public:
  WaitForMissionGoalBT(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config),
    node_(node)
  {
    sub_ =
      node_->create_subscription<husky_msgs::msg::MissionGoal>(
        "/mission_goal",
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(
          &WaitForMissionGoalBT::goal_cb,
          this,
          std::placeholders::_1));
  }

  // --------------------------------------------------
  // BT Ports
  // --------------------------------------------------
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<uint64_t>("mission_id"),
      BT::OutputPort<std::string>("goal_name")
    };
  }

  // --------------------------------------------------
  // BT Lifecycle
  // --------------------------------------------------
  BT::NodeStatus onStart() override
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (goal_received_) {

      mission_id_++;

      setOutput("goal", goal_);
      setOutput("mission_id", mission_id_);
      setOutput("goal_name", goal_name_);

      goal_received_ = false;

      RCLCPP_INFO(
        node_->get_logger(),
        "BT: Mission %lu received (name=%s)",
        mission_id_,
        goal_name_.c_str());

      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:

  // --------------------------------------------------
  // ROS Callback
  // --------------------------------------------------
  void goal_cb(
    const husky_msgs::msg::MissionGoal::SharedPtr msg)
  {
    goal_ = msg->pose;
    goal_name_ = msg->goal_name;
    goal_received_ = true;
  }

  // --------------------------------------------------
  // Members
  // --------------------------------------------------
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<husky_msgs::msg::MissionGoal>::SharedPtr sub_;

  geometry_msgs::msg::PoseStamped goal_;
  std::string goal_name_;

  bool goal_received_{false};
  uint64_t mission_id_{0};
};
