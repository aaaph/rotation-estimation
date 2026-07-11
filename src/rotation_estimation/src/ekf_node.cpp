#include "rotation_estimation/ekf_node.hpp"

namespace rotation_estimation {

EkfNode::EkfNode(const rclcpp::NodeOptions& options) : rclcpp::Node(kDefaultNodeName, options) {
  RCLCPP_INFO(get_logger(), "%s started", kDefaultNodeName);
}

}  // namespace rotation_estimation
