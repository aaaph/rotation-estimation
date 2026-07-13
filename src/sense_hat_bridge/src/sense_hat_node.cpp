#include "sense_hat_bridge/sense_hat_node.hpp"

#include <chrono>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"
#include "sense_hat_bridge/imu_strategy_factory.hpp"

namespace sense_hat_bridge {
namespace {

constexpr double kMockRateHz = 50.0;
constexpr double kUdpPollPeriodS = 0.001;

rcl_interfaces::msg::SetParametersResult SuccessfulParameterUpdate() {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

rcl_interfaces::msg::SetParametersResult FailedParameterUpdate(const std::string& reason) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = reason;
  return result;
}

}  // namespace

SenseHatNode::SenseHatNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node(kDefaultNodeName, options) {
  declare_parameter("mock", false);
  declare_parameter("mock_mode", std::string{kMockModeYawRotation});
  declare_parameter("mock_angular_velocity_z", 0.5);
  declare_parameter("mock_yaw_oscillation_frequency_hz", 0.25);
  declare_parameter("host", "127.0.0.1");
  declare_parameter("port", 8765);
  declare_parameter("frame_id", "sensehat_link");
  declare_parameter("topic", "/sensehat/imu_raw");

  const auto topic = get_parameter("topic").as_string();

  publisher_ = create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS{});

  configureStrategy(currentRuntimeConfig());

  parameter_callback_handle_ =
      add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
        return handleParameterUpdate(parameters);
      });
  RCLCPP_INFO(get_logger(), "%s started, publishing %s", kDefaultNodeName, topic.c_str());
}

SenseHatNode::RuntimeConfig SenseHatNode::currentRuntimeConfig() const {
  return RuntimeConfig{
      .mock = get_parameter("mock").as_bool(),
      .mock_mode = get_parameter("mock_mode").as_string(),
      .angular_velocity_z = get_parameter("mock_angular_velocity_z").as_double(),
      .yaw_oscillation_frequency_hz =
          get_parameter("mock_yaw_oscillation_frequency_hz").as_double(),
      .host = get_parameter("host").as_string(),
      .port = static_cast<int>(get_parameter("port").as_int()),
      .frame_id = get_parameter("frame_id").as_string(),
  };
}

void SenseHatNode::configureStrategy(const RuntimeConfig& config) {
  const std::string strategy_mode =
      config.mock ? config.mock_mode : std::string{kStrategyModeUdpReader};
  const double timer_period_s = config.mock ? 1.0 / kMockRateHz : kUdpPollPeriodS;

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>{timer_period_s});
  auto next_timer = create_wall_timer(timer_period, [this]() { publish_from_strategy(); });

  {
    std::scoped_lock lock{strategy_mutex_};
    if (timer_) {
      timer_->cancel();
    }
    auto old_snapshot_ = imu_strategy_ ? imu_strategy_->snapshot() : ImuStrategySnapshot{};
    auto next_strategy = create_imu_strategy(ImuStrategyConfig{
        .mode = strategy_mode,
        .frame_id = config.frame_id,
        .stamp_factory =
            [this]() { return static_cast<builtin_interfaces::msg::Time>(get_clock()->now()); },
        .host = config.host,
        .port = config.port,
        .angular_velocity_z = config.angular_velocity_z,
        .yaw_oscillation_frequency_hz = config.yaw_oscillation_frequency_hz,
        .old_snapshot = old_snapshot_,
    });
    imu_strategy_ = std::move(next_strategy);
    drain_strategy_ = !config.mock;
    timer_ = std::move(next_timer);
  }

  RCLCPP_INFO(get_logger(), "Configured %s strategy", strategy_mode.c_str());
}

rcl_interfaces::msg::SetParametersResult SenseHatNode::applyMockParameterOverride(
    const rclcpp::Parameter& parameter, RuntimeConfig& config, bool& should_reconfigure) {
  const auto& name = parameter.get_name();

  if (name == "mock_mode") {
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      return FailedParameterUpdate("mock_mode must be a string");
    }
    config.mock_mode = parameter.as_string();
    should_reconfigure = true;
    return SuccessfulParameterUpdate();
  }
  if (name == "mock_angular_velocity_z") {
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return FailedParameterUpdate("mock_angular_velocity_z must be a double");
    }
    config.angular_velocity_z = parameter.as_double();
    should_reconfigure = true;
    return SuccessfulParameterUpdate();
  }
  if (name == "mock_yaw_oscillation_frequency_hz") {
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return FailedParameterUpdate("mock_yaw_oscillation_frequency_hz must be a double");
    }
    if (parameter.as_double() <= 0.0) {
      return FailedParameterUpdate("mock_yaw_oscillation_frequency_hz must be greater than zero");
    }
    config.yaw_oscillation_frequency_hz = parameter.as_double();
    should_reconfigure = true;
    return SuccessfulParameterUpdate();
  }
  return FailedParameterUpdate(
      "only mock_mode, mock_angular_velocity_z, and mock_yaw_oscillation_frequency_hz can change "
      "at runtime");
}

rcl_interfaces::msg::SetParametersResult SenseHatNode::handleParameterUpdate(
    const std::vector<rclcpp::Parameter>& parameters) {
  if (!get_parameter("mock").as_bool()) {
    return FailedParameterUpdate("dynamic parameters are disabled in UDP mode");
  }

  auto config = currentRuntimeConfig();
  bool should_reconfigure = false;

  for (const auto& parameter : parameters) {
    const auto result = applyMockParameterOverride(parameter, config, should_reconfigure);
    if (!result.successful) {
      return result;
    }
  }

  if (!should_reconfigure) {
    return SuccessfulParameterUpdate();
  }
  if (config.mock_mode != kMockModeStationary && config.mock_mode != kMockModeYawRotation &&
      config.mock_mode != kMockModeYawOscillation) {
    return FailedParameterUpdate(
        "mock_mode must be one of: stationary, yaw_rotation, "
        "yaw_oscillation");
  }

  try {
    configureStrategy(config);
  } catch (const std::exception& error) {
    return FailedParameterUpdate(error.what());
  }
  return SuccessfulParameterUpdate();
}

void SenseHatNode::publish_from_strategy() {
  std::scoped_lock lock{strategy_mutex_};
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
