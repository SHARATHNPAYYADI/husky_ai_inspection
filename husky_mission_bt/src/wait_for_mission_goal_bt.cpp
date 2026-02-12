#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

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
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mission_goal",
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(
          &WaitForMissionGoalBT::goal_cb,
          this,
          std::placeholders::_1));
  }

  static BT::PortsList providedPorts()
    {
    return {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::OutputPort<uint64_t>("mission_id")
    };
    }

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

        goal_received_ = false;

        RCLCPP_INFO(
        node_->get_logger(),
        "BT: Mission %lu received", mission_id_);

        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
    }


  void onHalted() override {}

private:
  void goal_cb(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = *msg;
    goal_received_ = true;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

  geometry_msgs::msg::PoseStamped goal_;
  bool goal_received_{false};
  uint64_t mission_id_{0};
};
