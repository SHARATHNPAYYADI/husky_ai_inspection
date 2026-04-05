#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>

class HasMoreWaypoints : public BT::ConditionNode
{
public:
  HasMoreWaypoints(const std::string& name,
                   const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("index"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses")
    };
  }

  BT::NodeStatus tick() override
  {
    int index;
    std::vector<geometry_msgs::msg::PoseStamped> poses;

    if (!getInput("index", index) ||
        !getInput("poses", poses)) {
      return BT::NodeStatus::FAILURE;
    }

    if (index < (int)poses.size()) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};
