#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavigateToPoseBT : public BT::StatefulActionNode
{
public:
  NavigateToPoseBT(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config),
    node_(node)
  {
    client_ =
      rclcpp_action::create_client<NavigateToPose>(
        node_, "navigate_to_pose");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<uint64_t>("mission_id")
    };
  }

  BT::NodeStatus onStart() override
    {
    getInput("mission_id", current_mission_id_);

    if (current_mission_id_ != last_mission_id_) {
      // New mission detected → reset state
      finished_ = false;
      succeeded_ = false;
      failed_ = false;
      goal_sent_ = false;
    }
    if (finished_) {
        return succeeded_
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
    }
    if (goal_sent_) {
      return BT::NodeStatus::RUNNING;
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getInput("goal", goal_pose)) {
    RCLCPP_ERROR(node_->get_logger(),
        "BT: No goal in blackboard");
    return BT::NodeStatus::FAILURE;
    }

    NavigateToPose::Goal goal;
    goal.pose = goal_pose;

    auto options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.result_callback =
        [this](auto result) {
        if (result.code ==
            rclcpp_action::ResultCode::SUCCEEDED) {
            succeeded_ = true;
        } else {
            failed_ = true;
        }
        finished_ = true;
        };

    client_->async_send_goal(goal, options);
    goal_sent_ = true;

    RCLCPP_INFO(node_->get_logger(),
        "BT: NavigateToPose goal sent");

    return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus onRunning() override
    {
    if (!finished_) {
        return BT::NodeStatus::RUNNING;
    }
    last_mission_id_ = current_mission_id_;

    return succeeded_
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
    }


  void onHalted() override
    {
    goal_sent_ = false;
    finished_ = false;
    succeeded_ = false;
    failed_ = false;
    }

private:

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  bool succeeded_{false};
  bool failed_{false};
  bool goal_sent_{false};
  bool finished_{false};
  uint64_t last_mission_id_{0};
  uint64_t current_mission_id_{0};
};
