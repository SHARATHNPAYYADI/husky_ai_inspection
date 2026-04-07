#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "husky_msgs/msg/mission_goal.hpp"

/**
 * WaitForMissionGoalBT
 *
 * Waits on /mission_goal until a goal with a NEW mission_id arrives.
 * The mission_id guard prevents re-processing the latched message when
 * the BT sequence restarts after completing a waypoint.
 *
 * Outputs:
 *   goal       - PoseStamped to navigate to
 *   mission_id - uint64, echoed to PublishMissionCompleteBT
 *   goal_name  - string name of the waypoint
 */
class WaitForMissionGoalBT : public BT::StatefulActionNode
{
public:
  WaitForMissionGoalBT(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config),
    node_(node)
  {
    sub_ = node_->create_subscription<husky_msgs::msg::MissionGoal>(
      "/mission_goal",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&WaitForMissionGoalBT::goal_cb, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<uint64_t>("mission_id"),
      BT::OutputPort<std::string>("goal_name")
    };
  }

  // -----------------------------------------------------------------------
  BT::NodeStatus onStart() override
  {
    // Nothing to do — subscriber is always active
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!goal_received_) {
      return BT::NodeStatus::RUNNING;
    }

    // KEY GUARD: ignore if this mission_id was already processed
    if (incoming_mission_id_ == last_processed_id_) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "WaitForMissionGoal: mission_id %lu already processed, still waiting",
        incoming_mission_id_);
      goal_received_ = false;
      return BT::NodeStatus::RUNNING;
    }

    // New goal — accept it
    last_processed_id_ = incoming_mission_id_;
    goal_received_ = false;

    setOutput("goal",       goal_);
    setOutput("mission_id", incoming_mission_id_);
    setOutput("goal_name",  goal_name_);

    RCLCPP_INFO(
      node_->get_logger(),
      "WaitForMissionGoal: accepted mission_id=%lu  name='%s'",
      incoming_mission_id_,
      goal_name_.c_str());

    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  void goal_cb(const husky_msgs::msg::MissionGoal::SharedPtr msg)
  {
    goal_         = msg->pose;
    goal_name_    = msg->goal_name;
    incoming_mission_id_ = msg->mission_id;
    goal_received_ = true;
  }

  // -----------------------------------------------------------------------
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<husky_msgs::msg::MissionGoal>::SharedPtr sub_;

  geometry_msgs::msg::PoseStamped goal_;
  std::string                     goal_name_;
  uint64_t                        incoming_mission_id_{0};

  // Persists across BT restarts (member variable, not reset on onStart)
  uint64_t last_processed_id_{0};

  bool goal_received_{false};
};