#include "sense_hat_bridge/imu_strategy_factory.hpp"

#include <stdexcept>
#include <string>

#include "sense_hat_bridge/stationary_imu_mock.hpp"
#include "sense_hat_bridge/udp_reader_imu_strategy.hpp"
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
    return std::make_unique<YawRotationImuMock>(config.frame_id, config.stamp_factory,
                                                config.angular_velocity_z);
  }

  throw std::invalid_argument{"Unsupported IMU strategy mode '" + config.mode +
                              "'. Use one of: stationary, yaw_rotation, udp_reader."};
}

}  // namespace sense_hat_bridge
