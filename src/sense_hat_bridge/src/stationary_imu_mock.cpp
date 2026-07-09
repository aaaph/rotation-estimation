#include "sense_hat_bridge/stationary_imu_mock.hpp"

#include <utility>

namespace sense_hat_bridge {

namespace {

builtin_interfaces::msg::Time default_stamp() {
  return builtin_interfaces::msg::Time{};
}

}  // namespace

StationaryImuMock::StationaryImuMock(std::string frame_id, StampFactory stamp_factory,
                                     double gravity)
    : frame_id_(std::move(frame_id)),
      stamp_factory_(stamp_factory ? std::move(stamp_factory) : StampFactory{default_stamp}),
      gravity_(gravity) {}

std::optional<sensor_msgs::msg::Imu> StationaryImuMock::get_imu_message() {
  auto msg = new_imu_message(stamp_factory_(), frame_id_);
  msg.angular_velocity.x = 0.0;
  msg.angular_velocity.y = 0.0;
  msg.angular_velocity.z = 0.0;
  msg.linear_acceleration.x = 0.0;
  msg.linear_acceleration.y = 0.0;
  msg.linear_acceleration.z = gravity_;
  return msg;
}

}  // namespace sense_hat_bridge
