#ifndef SENSE_HAT_BRIDGE_SENSE_HAT_NODE_HPP_
#define SENSE_HAT_BRIDGE_SENSE_HAT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace sense_hat_bridge {

class SenseHatNode final : public rclcpp::Node {
 public:
  static constexpr auto kDefaultNodeName = "sense_hat_node";

  explicit SenseHatNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_SENSE_HAT_NODE_HPP_
