#ifndef SENSE_HAT_BRIDGE_YAW_ROTATION_IMU_MOCK_HPP_
#define SENSE_HAT_BRIDGE_YAW_ROTATION_IMU_MOCK_HPP_

#include <string>

#include "sense_hat_bridge/imu_strategy.hpp"

namespace sense_hat_bridge {

class YawRotationImuMock final : public ImuStrategy {
 public:
  explicit YawRotationImuMock(std::string frame_id = "", StampFactory stamp_factory = {},
                              double angular_velocity_z = 0.25);

  std::optional<sensor_msgs::msg::Imu> get_imu_message() override;

 private:
  std::string frame_id_;
  StampFactory stamp_factory_;
  double angular_velocity_z_;
};

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_YAW_ROTATION_IMU_MOCK_HPP_
