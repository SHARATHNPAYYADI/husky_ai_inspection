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
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
    }

  BT::NodeStatus onStart() override
    {
    if (finished_) {
        return succeeded_
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getInput("goal", goal_pose)) {
    RCLCPP_ERROR(node_->get_logger(),
        "BT: No goal in blackboard");
    return BT::NodeStatus::FAILURE;
    }

    NavigateToPose::Goal goal;
    goal.pose = create_goal();

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
  geometry_msgs::msg::PoseStamped create_goal()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = node_->now();

    pose.pose.position.x = -3.68;
    pose.pose.position.y = -1.78;
    pose.pose.orientation.w = 0.0;

    return pose;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  bool succeeded_{false};
  bool failed_{false};
  bool goal_sent_{false};
  bool finished_{false};
};
