#pragma once

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "husky_msgs/msg/mission_complete.hpp"

/**
 * PublishMissionCompleteBT
 *
 * Synchronous leaf node — placed at the END of the BT sequence.
 * Publishes MissionComplete on /mission_goal_complete so that
 * MissionInterfaceNode knows to send the next waypoint in the queue.
 *
 * Input ports:
 *   mission_id - echoed from WaitForMissionGoalBT output
 *   success    - (optional) whether inspection passed, default true
 */
class PublishMissionCompleteBT : public BT::SyncActionNode
{
public:
  PublishMissionCompleteBT(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config),
    node_(node)
  {
    pub_ = node_->create_publisher<husky_msgs::msg::MissionComplete>(
      "/mission_goal_complete",
      rclcpp::QoS(10).reliable());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<uint64_t>("mission_id"),
      BT::InputPort<bool>("success",  true, "Whether the full waypoint task succeeded")
    };
  }

  BT::NodeStatus tick() override
  {
    uint64_t mission_id = 0;
    getInput("mission_id", mission_id);

    bool success = true;
    getInput("success", success);

    husky_msgs::msg::MissionComplete msg;
    msg.mission_id = mission_id;
    msg.success    = success;
    msg.message    = success ? "Waypoint completed successfully" : "Waypoint completed with errors";

    pub_->publish(msg);

    RCLCPP_INFO(
      node_->get_logger(),
      "PublishMissionComplete: mission_id=%lu  success=%s",
      mission_id,
      success ? "true" : "false");

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<husky_msgs::msg::MissionComplete>::SharedPtr pub_;
};