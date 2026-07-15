#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <string_view>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rotation_estimation/ekf_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace {

constexpr auto kTestImuTopic = "/test/rotation_estimation/imu_raw";
constexpr auto kTestTfTopic = "/test/rotation_estimation/tf";
constexpr auto kTestFrameId = "test_imu_link";
constexpr double kGravityMetersPerSecondSquared = 9.80665;
constexpr double kQuaternionTolerance = 1e-9;

template <typename Predicate>
bool spin_until(rclcpp::executors::SingleThreadedExecutor& executor, Predicate predicate,
                std::chrono::nanoseconds timeout = std::chrono::seconds{3}) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds{10});
  }
  return predicate();
}

class EkfNodeRosTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

}  // namespace

TEST(EkfNodeTest, UsesDefaultNodeName) {
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultNodeName}, "ekf_node");
}

TEST(EkfNodeTest, UsesDefaultTopics) {
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultImuTopic}, "/sensehat/imu_raw");
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultTfTopic}, "/tf");
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultGyroBiasTopic},
            "/rotation_estimation/gyro_bias");
}

TEST(EkfNodeTest, UsesDefaultWorldFrame) {
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultWorldFrameId}, "world");
}

TEST(EkfNodeTest, UsesAccelAsDefaultMode) {
  EXPECT_EQ(rotation_estimation::EkfNode::kDefaultMode, rotation_estimation::EkfNode::Mode::kAccel);
}

TEST(EkfNodeTest, ConvertsModeToString) {
  EXPECT_EQ(rotation_estimation::EkfNode::toString(rotation_estimation::EkfNode::Mode::kAccel),
            "accel");
  EXPECT_EQ(rotation_estimation::EkfNode::toString(rotation_estimation::EkfNode::Mode::kStationary),
            "stationary");
}

TEST(EkfNodeTest, ParsesModeFromString) {
  EXPECT_EQ(rotation_estimation::EkfNode::modeFromString("accel"),
            rotation_estimation::EkfNode::Mode::kAccel);
  EXPECT_EQ(rotation_estimation::EkfNode::modeFromString("accel_update"),
            rotation_estimation::EkfNode::Mode::kAccel);
  EXPECT_EQ(rotation_estimation::EkfNode::modeFromString("stationary"),
            rotation_estimation::EkfNode::Mode::kStationary);
  EXPECT_EQ(rotation_estimation::EkfNode::modeFromString("unknown"), std::nullopt);
}

TEST_F(EkfNodeRosTest, UpdatesModeAtRuntime) {
  auto node = std::make_shared<rotation_estimation::EkfNode>();
  ASSERT_EQ(node->mode(), rotation_estimation::EkfNode::Mode::kAccel);

  const auto stationary_result = node->set_parameter(rclcpp::Parameter{"mode", "stationary"});
  ASSERT_TRUE(stationary_result.successful) << stationary_result.reason;
  EXPECT_EQ(node->mode(), rotation_estimation::EkfNode::Mode::kStationary);

  const auto accel_result = node->set_parameter(rclcpp::Parameter{"mode", "accel"});
  ASSERT_TRUE(accel_result.successful) << accel_result.reason;
  EXPECT_EQ(node->mode(), rotation_estimation::EkfNode::Mode::kAccel);
}

TEST_F(EkfNodeRosTest, KeepsIdentityOrientationForGravityAlignedStationaryImu) {
  auto options = rclcpp::NodeOptions{};
  options.parameter_overrides({
      rclcpp::Parameter{"mode", "stationary"},
      rclcpp::Parameter{"imu_topic", std::string{kTestImuTopic}},
      rclcpp::Parameter{"tf_topic", std::string{kTestTfTopic}},
  });
  auto node = std::make_shared<rotation_estimation::EkfNode>(options);
  auto helper_node = std::make_shared<rclcpp::Node>("ekf_node_test_helper");

  auto imu_publisher =
      helper_node->create_publisher<sensor_msgs::msg::Imu>(kTestImuTopic, rclcpp::SensorDataQoS{});
  bool received_tf = false;
  tf2_msgs::msg::TFMessage captured_tf;
  auto tf_subscription = helper_node->create_subscription<tf2_msgs::msg::TFMessage>(
      kTestTfTopic, rclcpp::QoS{10},
      [&received_tf, &captured_tf](const tf2_msgs::msg::TFMessage::ConstSharedPtr& msg) {
        captured_tf = *msg;
        received_tf = true;
      });
  ASSERT_NE(tf_subscription, nullptr);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);
  ASSERT_TRUE(spin_until(
      executor, [&imu_publisher]() { return imu_publisher->get_subscription_count() > 0; }));

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp.sec = 1;
  imu_msg.header.frame_id = kTestFrameId;
  imu_msg.linear_acceleration.z = kGravityMetersPerSecondSquared;
  imu_publisher->publish(imu_msg);

  ASSERT_TRUE(spin_until(executor, [&received_tf]() { return received_tf; }));
  ASSERT_EQ(captured_tf.transforms.size(), 1U);

  const auto& quaternion = captured_tf.transforms.front().transform.rotation;
  const double quaternion_norm =
      std::sqrt((quaternion.x * quaternion.x) + (quaternion.y * quaternion.y) +
                (quaternion.z * quaternion.z) + (quaternion.w * quaternion.w));
  EXPECT_NEAR(quaternion.x, 0.0, kQuaternionTolerance);
  EXPECT_NEAR(quaternion.y, 0.0, kQuaternionTolerance);
  EXPECT_NEAR(quaternion.z, 0.0, kQuaternionTolerance);
  EXPECT_NEAR(quaternion.w, 1.0, kQuaternionTolerance);
  EXPECT_NEAR(quaternion_norm, 1.0, kQuaternionTolerance);
}
