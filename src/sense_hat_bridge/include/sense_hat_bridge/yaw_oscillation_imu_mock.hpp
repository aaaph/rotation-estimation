#ifndef SENSE_HAT_BRIDGE_YAW_OSCILLATION_IMU_MOCK_HPP_
#define SENSE_HAT_BRIDGE_YAW_OSCILLATION_IMU_MOCK_HPP_

#include <string>

#include "sense_hat_bridge/imu_strategy.hpp"

namespace sense_hat_bridge {

class YawOscillationImuMock final : public ImuStrategy {
 public:
  struct Params {
    double start_angular_velocity_z{0.0};
    double amplitude_angular_velocity_z{0.25};
    double ramp_duration_s{1.0};
    double frequency_hz{0.25};
  };

  static constexpr auto kDefaultStartAngularVelocityZ = 0.0;
  static constexpr auto kDefaultAmplitudeAngularVelocityZ = 0.25;
  static constexpr auto kDefaultRampDuration = 1.0;
  static constexpr auto kDefaultFrequencyHz = 0.25;
  static constexpr auto kMockRateHz = 50.0;

  explicit YawOscillationImuMock(
      std::string frame_id = "", StampFactory stamp_factory = {},
      Params params = Params{.start_angular_velocity_z = kDefaultStartAngularVelocityZ,
                             .amplitude_angular_velocity_z = kDefaultAmplitudeAngularVelocityZ,
                             .ramp_duration_s = kDefaultRampDuration,
                             .frequency_hz = kDefaultFrequencyHz});

  [[nodiscard]] ImuStrategySnapshot snapshot() const override {
    return {.angular_velocity_z = current_angular_velocity_z_};
  }

  std::optional<sensor_msgs::msg::Imu> get_imu_message() override;

 private:
  std::string frame_id_;
  StampFactory stamp_factory_;
  Params params_;
  int sample_index_ = 0;
  double current_angular_velocity_z_ = 0.0;
};

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_YAW_OSCILLATION_IMU_MOCK_HPP_
