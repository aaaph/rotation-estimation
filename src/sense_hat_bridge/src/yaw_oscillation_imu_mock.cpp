#include "sense_hat_bridge/yaw_oscillation_imu_mock.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include "sense_hat_bridge/imu_message.hpp"

namespace sense_hat_bridge {
namespace {

builtin_interfaces::msg::Time default_stamp() {
  return builtin_interfaces::msg::Time{};
}

double exponentialEaseOut(double progress) {
  constexpr auto kShape = 4.0;
  if (progress >= 1.0) {
    return 1.0;
  }
  return (1.0 - std::exp(-kShape * progress)) / (1.0 - std::exp(-kShape));
}

int sampleCount(double duration_s) {
  if (duration_s <= 0.0) {
    return 0;
  }
  return static_cast<int>(std::ceil(duration_s * YawOscillationImuMock::kMockRateHz));
}

}  // namespace

YawOscillationImuMock::YawOscillationImuMock(std::string frame_id, StampFactory stamp_factory,
                                             Params params)
    : frame_id_(std::move(frame_id)),
      stamp_factory_(stamp_factory ? std::move(stamp_factory) : StampFactory{default_stamp}),
      params_(params),
      current_angular_velocity_z_(params.start_angular_velocity_z) {}

std::optional<sensor_msgs::msg::Imu> YawOscillationImuMock::get_imu_message() {
  auto msg = new_imu_message(stamp_factory_(), frame_id_);
  const auto ramp_samples = sampleCount(params_.ramp_duration_s);
  double angular_velocity_z = 0.0;

  if (sample_index_ <= ramp_samples) {
    const double progress =
        ramp_samples == 0
            ? 1.0
            : std::clamp(static_cast<double>(sample_index_) / static_cast<double>(ramp_samples),
                         0.0, 1.0);
    const double alpha = exponentialEaseOut(progress);
    angular_velocity_z = params_.start_angular_velocity_z * (1.0 - alpha);
  } else {
    const auto oscillation_sample_index = sample_index_ - ramp_samples - 1;
    const double t = static_cast<double>(oscillation_sample_index) / kMockRateHz;
    angular_velocity_z = params_.amplitude_angular_velocity_z *
                         std::sin(2.0 * std::numbers::pi * params_.frequency_hz * t);
  }

  current_angular_velocity_z_ = angular_velocity_z;
  msg.angular_velocity.x = 0.0;
  msg.angular_velocity.y = 0.0;
  msg.angular_velocity.z = angular_velocity_z;
  msg.linear_acceleration.x = 0.0;
  msg.linear_acceleration.y = 0.0;
  msg.linear_acceleration.z = kGravityMetersPerSecondSquared;
  sample_index_++;
  return msg;
}

}  // namespace sense_hat_bridge
