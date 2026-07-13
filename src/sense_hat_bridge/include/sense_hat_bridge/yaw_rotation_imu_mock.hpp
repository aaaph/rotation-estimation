#ifndef SENSE_HAT_BRIDGE_YAW_ROTATION_IMU_MOCK_HPP_
#define SENSE_HAT_BRIDGE_YAW_ROTATION_IMU_MOCK_HPP_

#include <string>

#include "sense_hat_bridge/imu_strategy.hpp"

namespace sense_hat_bridge {

class YawRotationImuMock final : public ImuStrategy {
 public:
  struct YawRotationRampParams {
    double start_angular_velocity_z{0.0};
    double end_angular_velocity_z{0.25};
    double duration_s{2.0};
  };

  [[nodiscard]] ImuStrategySnapshot snapshot() const override {
    return {.angular_velocity_z = current_angular_velocity_z_};
  };

  static constexpr auto kDefaultRampDuration = 1.0;
  static constexpr auto kDefaultStartAngularVelocityZ = 0.0;
  static constexpr auto kDefaultEndAngularVelocityZ = 0.25;
  static constexpr auto kMockRateHz = 50.0;

  explicit YawRotationImuMock(
      std::string frame_id = "", StampFactory stamp_factory = {},
      YawRotationRampParams yaw_rotation_ramp_params = YawRotationRampParams{
          .start_angular_velocity_z = kDefaultStartAngularVelocityZ,
          .end_angular_velocity_z = kDefaultEndAngularVelocityZ,
          .duration_s = kDefaultRampDuration});

  std::optional<sensor_msgs::msg::Imu> get_imu_message() override;

 private:
  std::string frame_id_;
  StampFactory stamp_factory_;
  YawRotationRampParams yaw_rotation_ramp_params_;
  int ramp_step_ = 0;
  double current_angular_velocity_z_ = 0.0;
};

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_YAW_ROTATION_IMU_MOCK_HPP_
