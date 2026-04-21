#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "husky_msgs/srv/inspect_fire_extinguisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "husky_msgs/msg/mission_result.hpp"

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

    status_pub_ =
      node_->create_publisher<std_msgs::msg::String>(
        "/mission_status", 10);
    result_pub_ = node_->create_publisher<husky_msgs::msg::MissionResult>(
        "/mission_results", 10);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal_name"),
      BT::InputPort<uint64_t>("mission_id"),
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

    // 🔥 RESET STATE (VERY IMPORTANT)
    done_ = false;
    sent_ = false;
    result_present_ = false;

    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(),
        "Inspection service not available");
      publishStatus("ERROR");
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

    publishStatus("INSPECTING");

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!done_) {
      return BT::NodeStatus::RUNNING;
    }

    // Store result locally
    bool present = result_present_;

    setOutput("present", present);
    uint64_t mission_id = 0;
    getInput("mission_id", mission_id);

    std::string goal_name;
    getInput("goal_name", goal_name);

    auto msg = husky_msgs::msg::MissionResult();
    msg.mission_id = mission_id;
    msg.goal_name = goal_name;
    msg.present = present;
    msg.confidence = result_confidence_;
    msg.image_path = result_image_path_;
    msg.annotated_path = result_annotated_path_;

    // timestamp
    auto now = node_->get_clock()->now();
    msg.timestamp = std::to_string(now.seconds());

    result_pub_->publish(msg);
    setOutput("confidence", result_confidence_);
    setOutput("image_path", result_image_path_);
    setOutput("annotated_path", result_annotated_path_);

    // 🔥 Always continue mission
    publishStatus(present ? "COMPLETED" : "INSPECTED");

    RCLCPP_WARN(node_->get_logger(),
      "Inspection result: present=%s",
      present ? "true" : "false");

    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    done_ = false;
    sent_ = false;
  }

private:

  void publishStatus(const std::string& state)
  {
    std_msgs::msg::String msg;
    msg.data = state;
    status_pub_->publish(msg);
  }

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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<husky_msgs::msg::MissionResult>::SharedPtr result_pub_;

  bool sent_{false};
  bool done_{false};

  bool result_present_{false};
  double result_confidence_{0.0};
  std::string result_image_path_;
  std::string result_annotated_path_;
};
