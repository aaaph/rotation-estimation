#ifndef ROTATION_ESTIMATION_EKF_NODE_HPP_
#define ROTATION_ESTIMATION_EKF_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace rotation_estimation {

class EkfNode final : public rclcpp::Node {
 public:
  static constexpr auto kDefaultNodeName = "ekf_node";

  explicit EkfNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

}  // namespace rotation_estimation

#endif  // ROTATION_ESTIMATION_EKF_NODE_HPP_
