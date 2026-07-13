#include "sense_hat_bridge/yaw_rotation_imu_mock.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "sense_hat_bridge/imu_message.hpp"

namespace sense_hat_bridge {

namespace {

builtin_interfaces::msg::Time default_stamp() {
  return builtin_interfaces::msg::Time{};
}

}  // namespace

YawRotationImuMock::YawRotationImuMock(std::string frame_id, StampFactory stamp_factory,
                                       YawRotationRampParams yaw_rotation_ramp_params)
    : frame_id_(std::move(frame_id)),
      stamp_factory_(stamp_factory ? std::move(stamp_factory) : StampFactory{default_stamp}),
      yaw_rotation_ramp_params_(yaw_rotation_ramp_params),
      current_angular_velocity_z_(yaw_rotation_ramp_params.start_angular_velocity_z) {}

std::optional<sensor_msgs::msg::Imu> YawRotationImuMock::get_imu_message() {
  auto msg = new_imu_message(stamp_factory_(), frame_id_);
  const double progress = std::clamp(
      static_cast<double>(ramp_step_) / (yaw_rotation_ramp_params_.duration_s * kMockRateHz), 0.0,
      1.0);
  const double shape = 4.0;
  const double alpha =
      progress >= 1.0 ? 1.0 : (1.0 - std::exp(-shape * progress)) / (1.0 - std::exp(-shape));
  const double angular_velocity_z = yaw_rotation_ramp_params_.start_angular_velocity_z +
                                    ((yaw_rotation_ramp_params_.end_angular_velocity_z -
                                      yaw_rotation_ramp_params_.start_angular_velocity_z) *
                                     alpha);
  current_angular_velocity_z_ = angular_velocity_z;
  msg.angular_velocity.z = angular_velocity_z;
  msg.angular_velocity.x = 0.0;
  msg.angular_velocity.y = 0.0;
  msg.linear_acceleration.x = 0.0;
  msg.linear_acceleration.y = 0.0;
  msg.linear_acceleration.z = kGravityMetersPerSecondSquared;
  ramp_step_++;
  return msg;
}

}  // namespace sense_hat_bridge
