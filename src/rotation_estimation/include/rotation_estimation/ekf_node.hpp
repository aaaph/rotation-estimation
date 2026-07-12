#ifndef ROTATION_ESTIMATION_EKF_NODE_HPP_
#define ROTATION_ESTIMATION_EKF_NODE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rotation_estimation/esekf.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace rotation_estimation {

class EkfNode final : public rclcpp::Node {
 public:
  enum class Mode : std::uint8_t {
    kAccel,
    kStationary,
  };

  static constexpr auto kDefaultNodeName = "ekf_node";
  static constexpr Mode kDefaultMode = Mode::kAccel;
  static constexpr auto kDefaultImuTopic = "/sensehat/imu_raw";
  static constexpr auto kDefaultTfTopic = "/tf";
  static constexpr auto kDefaultGyroBiasTopic = "/rotation_estimation/gyro_bias";
  static constexpr auto kDefaultWorldFrameId = "world";
  static constexpr auto kDefaultRotationCovarianceTopic =
      "/rotation_estimation/debug/rotation_covariance";
  static constexpr auto kDefaultGyroBiasCovarianceTopic =
      "/rotation_estimation/debug/gyro_bias_covariance";

  explicit EkfNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  [[nodiscard]] static std::optional<Mode> modeFromString(std::string_view mode);
  [[nodiscard]] static std::string_view toString(Mode mode);
  [[nodiscard]] Mode mode() const;

 private:
  void handleImuMessage(const sensor_msgs::msg::Imu& msg);

  Mode mode_ = kDefaultMode;
  ESEKF filter_;
  std::string world_frame_id_ = kDefaultWorldFrameId;
  std::optional<int64_t> first_imu_stamp_ns_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_bias_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rotation_covariance_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_bias_covariance_publisher_;
};

}  // namespace rotation_estimation

#endif  // ROTATION_ESTIMATION_EKF_NODE_HPP_
