#include <gtest/gtest.h>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "sense_hat_bridge/imu_message.hpp"
#include "sense_hat_bridge/imu_strategy_factory.hpp"
#include "sense_hat_bridge/stationary_imu_mock.hpp"
#include "sense_hat_bridge/yaw_oscillation_imu_mock.hpp"
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
      .old_snapshot = {},
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
  sense_hat_bridge::YawRotationImuMock mock{
      "sensehat_link", fixed_stamp,
      sense_hat_bridge::YawRotationImuMock::YawRotationRampParams{
          .start_angular_velocity_z = 1.2, .end_angular_velocity_z = 1.2, .duration_s = 1.0}};

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

TEST(YawRotationImuMockTest, RampsYawRateOverOneSecondAtFixedMockRate) {
  sense_hat_bridge::YawRotationImuMock mock{
      "sensehat_link", fixed_stamp,
      sense_hat_bridge::YawRotationImuMock::YawRotationRampParams{
          .start_angular_velocity_z = 0.0, .end_angular_velocity_z = 1.0, .duration_s = 1.0}};

  const auto first_msg = require_message(mock.get_imu_message());
  EXPECT_DOUBLE_EQ(first_msg.angular_velocity.z, 0.0);

  for (int i = 0; i < 24; ++i) {
    static_cast<void>(require_message(mock.get_imu_message()));
  }
  const auto mid_msg = require_message(mock.get_imu_message());
  EXPECT_GT(mid_msg.angular_velocity.z, 0.5);
  EXPECT_LT(mid_msg.angular_velocity.z, 1.0);

  for (int i = 0; i < 24; ++i) {
    static_cast<void>(require_message(mock.get_imu_message()));
  }
  const auto end_msg = require_message(mock.get_imu_message());
  EXPECT_DOUBLE_EQ(end_msg.angular_velocity.z, 1.0);
}

TEST(YawRotationImuMockTest, SnapshotStartsAtConfiguredStartYawRate) {
  const sense_hat_bridge::YawRotationImuMock mock{
      "sensehat_link", fixed_stamp,
      sense_hat_bridge::YawRotationImuMock::YawRotationRampParams{
          .start_angular_velocity_z = 0.7, .end_angular_velocity_z = 1.2, .duration_s = 1.0}};

  EXPECT_DOUBLE_EQ(mock.snapshot().angular_velocity_z, 0.7);
}

TEST(YawOscillationImuMockTest, RampsToZeroBeforeStartingSinusoid) {
  sense_hat_bridge::YawOscillationImuMock mock{
      "sensehat_link", fixed_stamp,
      sense_hat_bridge::YawOscillationImuMock::Params{.start_angular_velocity_z = 1.0,
                                                      .amplitude_angular_velocity_z = 2.0,
                                                      .ramp_duration_s = 1.0,
                                                      .frequency_hz = 0.25}};

  const auto first_msg = require_message(mock.get_imu_message());
  EXPECT_DOUBLE_EQ(first_msg.angular_velocity.z, 1.0);

  for (int i = 0; i < 49; ++i) {
    static_cast<void>(require_message(mock.get_imu_message()));
  }
  const auto ramp_end_msg = require_message(mock.get_imu_message());
  EXPECT_NEAR(ramp_end_msg.angular_velocity.z, 0.0, 1e-12);

  const auto oscillation_start_msg = require_message(mock.get_imu_message());
  EXPECT_NEAR(oscillation_start_msg.angular_velocity.z, 0.0, 1e-12);

  for (int i = 0; i < 49; ++i) {
    static_cast<void>(require_message(mock.get_imu_message()));
  }
  const auto quarter_period_msg = require_message(mock.get_imu_message());
  EXPECT_NEAR(quarter_period_msg.angular_velocity.z, 2.0, 1e-12);
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
  EXPECT_DOUBLE_EQ(msg.angular_velocity.z, 0.0);

  sensor_msgs::msg::Imu ramped_msg;
  for (int i = 0; i < 50; ++i) {
    ramped_msg = require_message(strategy->get_imu_message());
  }
  EXPECT_DOUBLE_EQ(ramped_msg.angular_velocity.z, 1.2);
}

TEST(ImuStrategyFactoryTest, CreatesYawRotationMockFromOldSnapshot) {
  auto config = config_for(std::string{sense_hat_bridge::kMockModeYawRotation});
  config.old_snapshot = sense_hat_bridge::ImuStrategySnapshot{.angular_velocity_z = 0.7};

  const auto strategy = sense_hat_bridge::create_imu_strategy(config);

  ASSERT_NE(dynamic_cast<sense_hat_bridge::YawRotationImuMock*>(strategy.get()), nullptr);
  const auto msg = require_message(strategy->get_imu_message());
  EXPECT_DOUBLE_EQ(msg.angular_velocity.z, 0.7);

  sensor_msgs::msg::Imu ramped_msg;
  for (int i = 0; i < 50; ++i) {
    ramped_msg = require_message(strategy->get_imu_message());
  }
  EXPECT_DOUBLE_EQ(ramped_msg.angular_velocity.z, 1.2);
}

TEST(ImuStrategyFactoryTest, CreatesYawOscillationMockFromOldSnapshot) {
  auto config = config_for(std::string{sense_hat_bridge::kMockModeYawOscillation});
  config.old_snapshot = sense_hat_bridge::ImuStrategySnapshot{.angular_velocity_z = 0.7};

  const auto strategy = sense_hat_bridge::create_imu_strategy(config);

  ASSERT_NE(dynamic_cast<sense_hat_bridge::YawOscillationImuMock*>(strategy.get()), nullptr);
  const auto first_msg = require_message(strategy->get_imu_message());
  EXPECT_DOUBLE_EQ(first_msg.angular_velocity.z, 0.7);

  sensor_msgs::msg::Imu ramp_end_msg;
  for (int i = 0; i < 50; ++i) {
    ramp_end_msg = require_message(strategy->get_imu_message());
  }
  EXPECT_NEAR(ramp_end_msg.angular_velocity.z, 0.0, 1e-12);

  const auto oscillation_start_msg = require_message(strategy->get_imu_message());
  EXPECT_NEAR(oscillation_start_msg.angular_velocity.z, 0.0, 1e-12);
}

TEST(ImuStrategyFactoryTest, PassesYawOscillationFrequencyParameter) {
  auto config = config_for(std::string{sense_hat_bridge::kMockModeYawOscillation});
  config.old_snapshot = sense_hat_bridge::ImuStrategySnapshot{.angular_velocity_z = 0.7};
  config.yaw_oscillation_frequency_hz = 0.5;

  const auto strategy = sense_hat_bridge::create_imu_strategy(config);

  ASSERT_NE(dynamic_cast<sense_hat_bridge::YawOscillationImuMock*>(strategy.get()), nullptr);
  const auto first_msg = require_message(strategy->get_imu_message());
  EXPECT_DOUBLE_EQ(first_msg.angular_velocity.z, 0.7);

  sensor_msgs::msg::Imu ramp_end_msg;
  for (int i = 0; i < 50; ++i) {
    ramp_end_msg = require_message(strategy->get_imu_message());
  }
  EXPECT_NEAR(ramp_end_msg.angular_velocity.z, 0.0, 1e-12);

  const auto oscillation_start_msg = require_message(strategy->get_imu_message());
  EXPECT_NEAR(oscillation_start_msg.angular_velocity.z, 0.0, 1e-12);

  for (int i = 0; i < 24; ++i) {
    static_cast<void>(require_message(strategy->get_imu_message()));
  }
  const auto quarter_period_msg = require_message(strategy->get_imu_message());
  EXPECT_NEAR(quarter_period_msg.angular_velocity.z, 1.2, 1e-12);
}

TEST(ImuStrategyFactoryTest, RejectsUnknownMode) {
  EXPECT_THROW(sense_hat_bridge::create_imu_strategy(config_for("unknown")), std::invalid_argument);
}
