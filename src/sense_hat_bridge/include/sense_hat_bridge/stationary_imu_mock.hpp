#ifndef SENSE_HAT_BRIDGE_STATIONARY_IMU_MOCK_HPP_
#define SENSE_HAT_BRIDGE_STATIONARY_IMU_MOCK_HPP_

#include <string>

#include "sense_hat_bridge/imu_message.hpp"
#include "sense_hat_bridge/imu_strategy.hpp"

namespace sense_hat_bridge {

class StationaryImuMock final : public ImuStrategy {
 public:
  explicit StationaryImuMock(std::string frame_id = "", StampFactory stamp_factory = {},
                             double gravity = kGravityMetersPerSecondSquared);

  std::optional<sensor_msgs::msg::Imu> get_imu_message() override;

 private:
  std::string frame_id_;
  StampFactory stamp_factory_;
  double gravity_;
};

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_STATIONARY_IMU_MOCK_HPP_
