#ifndef SENSE_HAT_BRIDGE_IMU_MESSAGE_HPP_
#define SENSE_HAT_BRIDGE_IMU_MESSAGE_HPP_

#include <string_view>

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace sense_hat_bridge {

inline constexpr double kGravityMetersPerSecondSquared = 9.80665;

sensor_msgs::msg::Imu new_imu_message(const builtin_interfaces::msg::Time& stamp,
                                      std::string_view frame_id);

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_IMU_MESSAGE_HPP_
