#include <gtest/gtest.h>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "sense_hat_bridge/imu_message.hpp"
#include "sense_hat_bridge/imu_strategy_factory.hpp"
#include "sense_hat_bridge/stationary_imu_mock.hpp"
#include "sense_hat_bridge/yaw_rotation_imu_mock.hpp"

namespace {

builtin_interfaces::msg::Time fixed_stamp() {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1;
  stamp.nanosec = 2;
  return stamp;
}

sense_hat_bridge::ImuStrategyConfig config_for(std::string mode) {
  return sense_hat_bridge::ImuStrategyConfig{
      .mode = std::move(mode),
      .frame_id = "sensehat_link",
      .stamp_factory = fixed_stamp,
      .angular_velocity_z = 1.2,
  };
}

sensor_msgs::msg::Imu require_message(std::optional<sensor_msgs::msg::Imu> msg) {
  if (!msg.has_value()) {
    throw std::runtime_error{"Expected IMU message"};
  }
  return *msg;
}

}  // namespace

TEST(StationaryImuMockTest, CreatesStationaryImuMessage) {
  sense_hat_bridge::StationaryImuMock mock{"sensehat_link", fixed_stamp};

  const auto msg = require_message(mock.get_imu_message());

  EXPECT_EQ(msg.header.frame_id, "sensehat_link");
  EXPECT_EQ(msg.header.stamp.sec, 1);
  EXPECT_EQ(msg.header.stamp.nanosec, 2U);
  EXPECT_DOUBLE_EQ(msg.orientation_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(msg.angular_velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.angular_velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(msg.angular_velocity.z, 0.0);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.y, 0.0);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.z, sense_hat_bridge::kGravityMetersPerSecondSquared);
}

TEST(YawRotationImuMockTest, CreatesYawRotationImuMessage) {
  sense_hat_bridge::YawRotationImuMock mock{"sensehat_link", fixed_stamp, 1.2};

  const auto msg = require_message(mock.get_imu_message());

  EXPECT_EQ(msg.header.frame_id, "sensehat_link");
  EXPECT_EQ(msg.header.stamp.sec, 1);
  EXPECT_EQ(msg.header.stamp.nanosec, 2U);
  EXPECT_DOUBLE_EQ(msg.orientation_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(msg.angular_velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.angular_velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(msg.angular_velocity.z, 1.2);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.y, 0.0);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.z, sense_hat_bridge::kGravityMetersPerSecondSquared);
}

TEST(ImuStrategyFactoryTest, CreatesStationaryMock) {
  const auto strategy = sense_hat_bridge::create_imu_strategy(
      config_for(std::string{sense_hat_bridge::kMockModeStationary}));

  EXPECT_NE(dynamic_cast<sense_hat_bridge::StationaryImuMock*>(strategy.get()), nullptr);
}

TEST(ImuStrategyFactoryTest, CreatesYawRotationMock) {
  const auto strategy = sense_hat_bridge::create_imu_strategy(
      config_for(std::string{sense_hat_bridge::kMockModeYawRotation}));

  ASSERT_NE(dynamic_cast<sense_hat_bridge::YawRotationImuMock*>(strategy.get()), nullptr);
  const auto msg = require_message(strategy->get_imu_message());
  EXPECT_DOUBLE_EQ(msg.angular_velocity.z, 1.2);
}

TEST(ImuStrategyFactoryTest, RejectsUnknownMode) {
  EXPECT_THROW(sense_hat_bridge::create_imu_strategy(config_for("unknown")), std::invalid_argument);
}
