#ifndef SENSE_HAT_BRIDGE_SENSE_HAT_NODE_HPP_
#define SENSE_HAT_BRIDGE_SENSE_HAT_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
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
  struct RuntimeConfig {
    bool mock{false};
    std::string mock_mode;
    double angular_velocity_z{0.0};
    double yaw_oscillation_frequency_hz{0.25};
    std::string host;
    int port{8765};
    std::string frame_id;
  };

  [[nodiscard]] RuntimeConfig currentRuntimeConfig() const;
  void configureStrategy(const RuntimeConfig& config);
  [[nodiscard]] bool isMockMode() const;
  static rcl_interfaces::msg::SetParametersResult applyMockParameterOverride(
      const rclcpp::Parameter& parameter, RuntimeConfig& config, bool& should_reconfigure);
  rcl_interfaces::msg::SetParametersResult handleParameterUpdate(
      const std::vector<rclcpp::Parameter>& parameters);

  std::mutex strategy_mutex_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<ImuStrategy> imu_strategy_;
  bool drain_strategy_{false};
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_SENSE_HAT_NODE_HPP_
