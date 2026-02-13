#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "husky_msgs/srv/inspect_fire_extinguisher.hpp"

class InspectFireExtinguisherBT : public BT::StatefulActionNode
{
public:
  InspectFireExtinguisherBT(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config),
    node_(node)
  {
    client_ =
      node_->create_client<husky_msgs::srv::InspectFireExtinguisher>(
        "/inspect/fire_extinguisher");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal_name"),
      BT::OutputPort<bool>("present"),
      BT::OutputPort<double>("confidence"),
      BT::OutputPort<std::string>("image_path"),
      BT::OutputPort<std::string>("annotated_path")
    };
  }

  BT::NodeStatus onStart() override
  {
    RCLCPP_INFO(node_->get_logger(),
    "InspectFireExtinguisherBT started");
    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(),
        "Inspection service not available");
      return BT::NodeStatus::FAILURE;
    }

    auto request =
      std::make_shared<husky_msgs::srv::InspectFireExtinguisher::Request>();

    std::string goal_name;
    if (!getInput("goal_name", goal_name)) {
      goal_name = "Unknown";
    }

    request->goal_name = goal_name;

    auto future =
      client_->async_send_request(
        request,
        std::bind(
          &InspectFireExtinguisherBT::response_callback,
          this,
          std::placeholders::_1));

    sent_ = true;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!done_) {
      return BT::NodeStatus::RUNNING;
    }

    // Write to blackboard
    setOutput("present", result_present_);
    setOutput("confidence", result_confidence_);
    setOutput("image_path", result_image_path_);
    setOutput("annotated_path", result_annotated_path_);

    return result_present_
      ? BT::NodeStatus::SUCCESS
      : BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    sent_ = false;
    done_ = false;
  }

private:
  void response_callback(
    rclcpp::Client<husky_msgs::srv::InspectFireExtinguisher>::SharedFuture future)
  {
    auto response = future.get();

    result_present_ = response->present;
    result_confidence_ = response->confidence;
    result_image_path_ = response->image_path;
    result_annotated_path_ = response->annotated_path;

    done_ = true;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<husky_msgs::srv::InspectFireExtinguisher>::SharedPtr client_;

  bool sent_{false};
  bool done_{false};

  bool result_present_{false};
  double result_confidence_{0.0};
  std::string result_image_path_;
  std::string result_annotated_path_;
};
