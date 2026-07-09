#include "sense_hat_bridge/imu_message.hpp"

namespace sense_hat_bridge {

sensor_msgs::msg::Imu new_imu_message(const builtin_interfaces::msg::Time& stamp,
                                      std::string_view frame_id) {
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = std::string{frame_id};
  msg.orientation_covariance[0] = -1.0;
  return msg;
}

}  // namespace sense_hat_bridge
