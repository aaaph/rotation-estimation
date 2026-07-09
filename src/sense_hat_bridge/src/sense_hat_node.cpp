#include "sense_hat_bridge/sense_hat_node.hpp"

namespace sense_hat_bridge {

SenseHatNode::SenseHatNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node(kDefaultNodeName, options) {
  RCLCPP_INFO(get_logger(), "%s started", kDefaultNodeName);
}

}  // namespace sense_hat_bridge
