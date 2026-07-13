#include "sense_hat_bridge/imu_strategy_factory.hpp"

#include <stdexcept>
#include <string>

#include "sense_hat_bridge/stationary_imu_mock.hpp"
#include "sense_hat_bridge/udp_reader_imu_strategy.hpp"
#include "sense_hat_bridge/yaw_oscillation_imu_mock.hpp"
#include "sense_hat_bridge/yaw_rotation_imu_mock.hpp"

namespace sense_hat_bridge {

std::unique_ptr<ImuStrategy> create_imu_strategy(const ImuStrategyConfig& config) {
  if (config.mode == kStrategyModeUdpReader) {
    return std::make_unique<UdpReaderImuStrategy>(config.host, config.port, config.frame_id,
                                                  config.stamp_factory);
  }

  if (config.mode == kMockModeStationary) {
    return std::make_unique<StationaryImuMock>(config.frame_id, config.stamp_factory);
  }

  if (config.mode == kMockModeYawRotation) {
    return std::make_unique<YawRotationImuMock>(
        config.frame_id, config.stamp_factory,
        YawRotationImuMock::YawRotationRampParams{
            .start_angular_velocity_z = config.old_snapshot.angular_velocity_z,
            .end_angular_velocity_z = config.angular_velocity_z,
            .duration_s = 1.0});
  }

  if (config.mode == kMockModeYawOscillation) {
    return std::make_unique<YawOscillationImuMock>(
        config.frame_id, config.stamp_factory,
        YawOscillationImuMock::Params{
            .start_angular_velocity_z = config.old_snapshot.angular_velocity_z,
            .amplitude_angular_velocity_z = config.angular_velocity_z,
            .ramp_duration_s = YawOscillationImuMock::kDefaultRampDuration,
            .frequency_hz = config.yaw_oscillation_frequency_hz});
  }

  throw std::invalid_argument{"Unsupported IMU strategy mode '" + config.mode +
                              "'. Use one of: stationary, yaw_rotation, yaw_oscillation, "
                              "udp_reader."};
}

}  // namespace sense_hat_bridge
