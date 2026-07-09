#include "sense_hat_bridge/yaw_rotation_imu_mock.hpp"

#include <utility>

#include "sense_hat_bridge/imu_message.hpp"

namespace sense_hat_bridge {

namespace {

builtin_interfaces::msg::Time default_stamp() {
  return builtin_interfaces::msg::Time{};
}

}  // namespace

YawRotationImuMock::YawRotationImuMock(std::string frame_id, StampFactory stamp_factory,
                                       double angular_velocity_z)
    : frame_id_(std::move(frame_id)),
      stamp_factory_(stamp_factory ? std::move(stamp_factory) : StampFactory{default_stamp}),
      angular_velocity_z_(angular_velocity_z) {}

std::optional<sensor_msgs::msg::Imu> YawRotationImuMock::get_imu_message() {
  auto msg = new_imu_message(stamp_factory_(), frame_id_);
  msg.angular_velocity.x = 0.0;
  msg.angular_velocity.y = 0.0;
  msg.angular_velocity.z = angular_velocity_z_;
  msg.linear_acceleration.x = 0.0;
  msg.linear_acceleration.y = 0.0;
  msg.linear_acceleration.z = kGravityMetersPerSecondSquared;
  return msg;
}

}  // namespace sense_hat_bridge
