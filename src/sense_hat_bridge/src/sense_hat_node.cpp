#include "sense_hat_bridge/sense_hat_node.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

#include "rclcpp/qos.hpp"
#include "sense_hat_bridge/imu_strategy_factory.hpp"

namespace sense_hat_bridge {

SenseHatNode::SenseHatNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node(kDefaultNodeName, options) {
  declare_parameter("mock", false);
  declare_parameter("mock_mode", std::string{kMockModeYawRotation});
  declare_parameter("mock_rate_hz", 50.0);
  declare_parameter("mock_angular_velocity_z", 3.75);
  declare_parameter("host", "127.0.0.1");
  declare_parameter("port", 8765);
  declare_parameter("frame_id", "sensehat_link");
  declare_parameter("topic", "/sensehat/imu_raw");

  const bool mock = get_parameter("mock").as_bool();
  const auto host = get_parameter("host").as_string();
  const auto port = static_cast<int>(get_parameter("port").as_int());
  const auto frame_id = get_parameter("frame_id").as_string();
  const auto topic = get_parameter("topic").as_string();

  publisher_ = create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS{});

  std::string strategy_mode;
  double timer_period_s = 0.001;
  double angular_velocity_z = 0.0;
  std::string log_message;

  if (mock) {
    const auto rate_hz = get_parameter("mock_rate_hz").as_double();
    if (rate_hz <= 0.0) {
      throw std::invalid_argument{"mock_rate_hz must be greater than zero"};
    }

    strategy_mode = get_parameter("mock_mode").as_string();
    angular_velocity_z = get_parameter("mock_angular_velocity_z").as_double();
    timer_period_s = 1.0 / rate_hz;
    drain_strategy_ = false;
    log_message = "Publishing " + strategy_mode + " mock IMU on " + topic;
  } else {
    strategy_mode = std::string{kStrategyModeUdpReader};
    drain_strategy_ = true;
    log_message = "Listening UDP " + host + ":" + std::to_string(port) + ", publishing " + topic;
  }

  imu_strategy_ = create_imu_strategy(ImuStrategyConfig{
      .mode = strategy_mode,
      .frame_id = frame_id,
      .stamp_factory =
          [this]() { return static_cast<builtin_interfaces::msg::Time>(get_clock()->now()); },
      .host = host,
      .port = port,
      .angular_velocity_z = angular_velocity_z,
  });

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>{timer_period_s});
  timer_ = create_wall_timer(timer_period, [this]() { publish_from_strategy(); });
  RCLCPP_INFO(get_logger(), "%s", log_message.c_str());
}

void SenseHatNode::publish_from_strategy() {
  if (!imu_strategy_) {
    return;
  }

  while (true) {
    auto msg = imu_strategy_->get_imu_message();
    if (!msg) {
      return;
    }

    publisher_->publish(*msg);
    if (!drain_strategy_) {
      return;
    }
  }
}

}  // namespace sense_hat_bridge
