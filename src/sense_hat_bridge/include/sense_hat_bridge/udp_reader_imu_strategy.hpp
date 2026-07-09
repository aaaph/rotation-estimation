#ifndef SENSE_HAT_BRIDGE_UDP_READER_IMU_STRATEGY_HPP_
#define SENSE_HAT_BRIDGE_UDP_READER_IMU_STRATEGY_HPP_

#include <string>

#include "sense_hat_bridge/imu_strategy.hpp"

namespace sense_hat_bridge {

class UdpReaderImuStrategy final : public ImuStrategy {
 public:
  UdpReaderImuStrategy(const std::string& host, int port, std::string frame_id,
                       StampFactory stamp_factory);
  ~UdpReaderImuStrategy() override;

  UdpReaderImuStrategy(const UdpReaderImuStrategy&) = delete;
  UdpReaderImuStrategy& operator=(const UdpReaderImuStrategy&) = delete;
  UdpReaderImuStrategy(UdpReaderImuStrategy&&) = delete;
  UdpReaderImuStrategy& operator=(UdpReaderImuStrategy&&) = delete;

  std::optional<sensor_msgs::msg::Imu> get_imu_message() override;
  void close() override;

 private:
  std::string frame_id_;
  StampFactory stamp_factory_;
  int socket_fd_{-1};
};

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_UDP_READER_IMU_STRATEGY_HPP_
