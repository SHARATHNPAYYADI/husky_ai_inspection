#include <memory>
#include <string>
#include <vector>

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
  // PORTS (UPDATED 🔥)
  // --------------------------------------------------
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<std::string>("goal_name"),
      BT::OutputPort<uint64_t>("mission_id"),

      // 🔥 NEW PORTS
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses"),
      BT::OutputPort<std::vector<std::string>>("goal_names"),
      BT::OutputPort<int>("index")
    };
  }

  // --------------------------------------------------
  BT::NodeStatus onStart() override
  {
    return BT::NodeStatus::RUNNING;
  }

  // --------------------------------------------------
  BT::NodeStatus onRunning() override
  {
    if (!goal_received_) {
      return BT::NodeStatus::RUNNING;
    }

    mission_id_++;

    // 🔥 SEND EVERYTHING TO BLACKBOARD
    setOutput("goal", poses_[0]);
    setOutput("goal_name", goal_names_[0]);
    setOutput("mission_id", mission_id_);

    setOutput("poses", poses_);
    setOutput("goal_names", goal_names_);
    int index = 0;
    setOutput("index", index);

    goal_received_ = false;

    RCLCPP_INFO(
      node_->get_logger(),
      "BT: Multi-goal mission received (%zu points)",
      poses_.size());

    RCLCPP_INFO(node_->get_logger(),
      "DEBUG: poses=%zu, names=%zu",
      poses_.size(), goal_names_.size());

    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:

  // --------------------------------------------------
  void goal_cb(
    const husky_msgs::msg::MissionGoal::SharedPtr msg)
  {
    poses_ = msg->poses;
    goal_names_ = msg->goal_names;

    if (poses_.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Received empty mission");
      return;
    }

    goal_received_ = true;
  }

  // --------------------------------------------------
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<husky_msgs::msg::MissionGoal>::SharedPtr sub_;

  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  std::vector<std::string> goal_names_;

  bool goal_received_{false};
  uint64_t mission_id_{0};
};