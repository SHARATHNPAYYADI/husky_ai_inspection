#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>

class NextWaypointBT : public BT::SyncActionNode
{
public:
  NextWaypointBT(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
    {
    return {
        BT::InputPort<std::vector<std::string>>("goal_names"),
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses"),

        // BT::InputPort<int>("index"),
        // BT::OutputPort<int>("index"),
        BT::BidirectionalPort<int>("index"),

        BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::OutputPort<std::string>("goal_name")
        };
    }

    BT::NodeStatus tick() override
    {
    std::vector<std::string> names;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    int index = 0;

    getInput("goal_names", names);
    getInput("poses", poses);
    getInput("index", index);

    // 🔥 CHECK BEFORE increment
    if (index >= (int)poses.size()) {
        RCLCPP_WARN(rclcpp::get_logger("NextWaypoint"),
        "Mission complete");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_WARN(rclcpp::get_logger("NextWaypoint"),
        "Going to index: %d", index);

    // ✅ Set current goal FIRST
    setOutput("goal", poses[index]);
    setOutput("goal_name", names[index]);

    // ✅ THEN increment for next cycle
    index++;
    setOutput("index", index+1);

    return BT::NodeStatus::SUCCESS;
    }
};