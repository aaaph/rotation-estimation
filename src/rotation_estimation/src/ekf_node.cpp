#include "rotation_estimation/ekf_node.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/qos.hpp"

namespace rotation_estimation {
namespace {

constexpr int64_t kNanosecondsPerSecond = 1'000'000'000LL;

int64_t toNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return (static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond) +
         static_cast<int64_t>(stamp.nanosec);
}

Eigen::Vector3d angularVelocityFrom(const sensor_msgs::msg::Imu& msg) {
  return {
      msg.angular_velocity.x,
      msg.angular_velocity.y,
      msg.angular_velocity.z,
  };
}

Eigen::Vector3d linearAccelerationFrom(const sensor_msgs::msg::Imu& msg) {
  return {
      msg.linear_acceleration.x,
      msg.linear_acceleration.y,
      msg.linear_acceleration.z,
  };
}

}  // namespace

EkfNode::EkfNode(const rclcpp::NodeOptions& options) : rclcpp::Node(kDefaultNodeName, options) {
  const auto mode_name =
      declare_parameter<std::string>("mode", std::string{toString(kDefaultMode)});
  const auto imu_topic = declare_parameter<std::string>("imu_topic", kDefaultImuTopic);
  const auto tf_topic = declare_parameter<std::string>("tf_topic", kDefaultTfTopic);
  const auto rotation_covariance_topic =
      declare_parameter<std::string>("rotation_covariance_topic", kDefaultRotationCovarianceTopic);
  const auto gyro_bias_covariance_topic =
      declare_parameter<std::string>("gyro_bias_covariance_topic", kDefaultGyroBiasCovarianceTopic);
  const auto gyro_bias_topic =
      declare_parameter<std::string>("gyro_bias_topic", kDefaultGyroBiasTopic);
  world_frame_id_ = declare_parameter<std::string>("world_frame_id", kDefaultWorldFrameId);

  const auto parsed_mode = modeFromString(mode_name);
  if (!parsed_mode.has_value()) {
    throw std::invalid_argument{"invalid ekf_node mode '" + mode_name +
                                "'; expected one of: accel, stationary"};
  }

  mode_ = *parsed_mode;
  tf_publisher_ = create_publisher<tf2_msgs::msg::TFMessage>(tf_topic, rclcpp::QoS{10});
  gyro_bias_publisher_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(gyro_bias_topic, rclcpp::QoS{10});
  rotation_covariance_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      rotation_covariance_topic, rclcpp::QoS{10});
  gyro_bias_covariance_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      gyro_bias_covariance_topic, rclcpp::QoS{10});

#if !defined(__clang_analyzer__)
  // rclcpp callback storage triggers clang static analyzer false positives in ROS headers.
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS{},
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) { handleImuMessage(*msg); });
#endif
  parameter_callback_handle_ =
      add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
        return handleParameterUpdate(parameters);
      });

  const std::string active_mode{toString(mode_)};
  RCLCPP_INFO(get_logger(), "%s started in %s mode; subscribing %s, publishing %s and %s",
              kDefaultNodeName, active_mode.c_str(), imu_topic.c_str(), tf_topic.c_str(),
              gyro_bias_topic.c_str());
}

std::optional<EkfNode::Mode> EkfNode::modeFromString(std::string_view mode) {
  if (mode == "accel" || mode == "accel_update") {
    return Mode::kAccel;
  }
  if (mode == "stationary") {
    return Mode::kStationary;
  }
  return std::nullopt;
}

std::string_view EkfNode::toString(Mode mode) {
  switch (mode) {
    case Mode::kAccel:
      return "accel";
    case Mode::kStationary:
      return "stationary";
  }
  return "unknown";
}

EkfNode::Mode EkfNode::mode() const {
  return mode_;
}

rcl_interfaces::msg::SetParametersResult EkfNode::handleParameterUpdate(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::optional<Mode> next_mode;
  for (const auto& parameter : parameters) {
    if (parameter.get_name() != "mode") {
      continue;
    }

    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      result.successful = false;
      result.reason = "mode must be a string";
      return result;
    }

    next_mode = modeFromString(parameter.as_string());
    if (!next_mode.has_value()) {
      result.successful = false;
      result.reason = "mode must be one of: accel, accel_update, stationary";
      return result;
    }
  }

  if (next_mode.has_value() && *next_mode != mode_) {
    mode_ = *next_mode;
    RCLCPP_INFO(get_logger(), "%s switched to %s mode", kDefaultNodeName,
                std::string{toString(mode_)}.c_str());
  }

  return result;
}

void EkfNode::handleImuMessage(const sensor_msgs::msg::Imu& msg) {
  const int64_t imu_stamp_ns = toNanoseconds(msg.header.stamp);
  if (!first_imu_stamp_ns_.has_value()) {
    first_imu_stamp_ns_ = imu_stamp_ns;
  }

  const int64_t filter_stamp_ns = imu_stamp_ns - *first_imu_stamp_ns_;
  const Eigen::Vector3d gyro = angularVelocityFrom(msg);
  const Eigen::Vector3d accel = linearAccelerationFrom(msg);

  filter_.predict(gyro, filter_stamp_ns);
  filter_.updateByAccel(accel);
  if (mode_ == Mode::kStationary) {
    filter_.updateByStationary(gyro);
  }

  const auto& state = filter_.state();
  const auto covariance_diagonal = filter_.covariance_diagonal();

  tf2_msgs::msg::TFMessage tf_msg;
  auto& transform_msg = tf_msg.transforms.emplace_back();
  transform_msg.header = msg.header;
  transform_msg.header.frame_id = world_frame_id_;
  transform_msg.child_frame_id = msg.header.frame_id;
  transform_msg.transform.translation.x = 0.0;
  transform_msg.transform.translation.y = 0.0;
  transform_msg.transform.translation.z = 0.0;
  transform_msg.transform.rotation.x = state.q.x();
  transform_msg.transform.rotation.y = state.q.y();
  transform_msg.transform.rotation.z = state.q.z();
  transform_msg.transform.rotation.w = state.q.w();
  tf_publisher_->publish(tf_msg);

  geometry_msgs::msg::Vector3Stamped gyro_bias_msg;
  gyro_bias_msg.header = msg.header;
  gyro_bias_msg.vector.x = state.gyro_bias.x();
  gyro_bias_msg.vector.y = state.gyro_bias.y();
  gyro_bias_msg.vector.z = state.gyro_bias.z();
  gyro_bias_publisher_->publish(gyro_bias_msg);

  geometry_msgs::msg::Vector3Stamped rotation_covariance_msg;
  rotation_covariance_msg.header = msg.header;
  rotation_covariance_msg.vector.x = std::sqrt(covariance_diagonal(0)) * 180.0 / std::numbers::pi;
  rotation_covariance_msg.vector.y = std::sqrt(covariance_diagonal(1)) * 180.0 / std::numbers::pi;
  rotation_covariance_msg.vector.z =
      std::min(180.0, std::sqrt(covariance_diagonal(2)) * 180.0 / std::numbers::pi);
  rotation_covariance_publisher_->publish(rotation_covariance_msg);

  geometry_msgs::msg::Vector3Stamped gyro_bias_covariance_msg;
  gyro_bias_covariance_msg.header = msg.header;
  gyro_bias_covariance_msg.vector.x = covariance_diagonal(3);
  gyro_bias_covariance_msg.vector.y = covariance_diagonal(4);
  gyro_bias_covariance_msg.vector.z = covariance_diagonal(5);
  gyro_bias_covariance_publisher_->publish(gyro_bias_covariance_msg);
}

}  // namespace rotation_estimation
