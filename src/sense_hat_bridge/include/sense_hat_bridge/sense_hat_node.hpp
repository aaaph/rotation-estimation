#ifndef SENSE_HAT_BRIDGE_SENSE_HAT_NODE_HPP_
#define SENSE_HAT_BRIDGE_SENSE_HAT_NODE_HPP_

#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include "sense_hat_bridge/imu_strategy.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace sense_hat_bridge {

class SenseHatNode final : public rclcpp::Node {
 public:
  static constexpr auto kDefaultNodeName = "sense_hat_imu_node";

  explicit SenseHatNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void publish_from_strategy();

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<ImuStrategy> imu_strategy_;
  bool drain_strategy_{false};
};

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_SENSE_HAT_NODE_HPP_
