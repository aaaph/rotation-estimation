#ifndef SENSE_HAT_BRIDGE_IMU_STRATEGY_FACTORY_HPP_
#define SENSE_HAT_BRIDGE_IMU_STRATEGY_FACTORY_HPP_

#include <memory>
#include <string>
#include <string_view>

#include "sense_hat_bridge/imu_strategy.hpp"

namespace sense_hat_bridge {

inline constexpr std::string_view kStrategyModeUdpReader = "udp_reader";
inline constexpr std::string_view kMockModeStationary = "stationary";
inline constexpr std::string_view kMockModeYawRotation = "yaw_rotation";
inline constexpr std::string_view kMockModeYawOscillation = "yaw_oscillation";

struct ImuStrategyConfig {
  std::string mode;
  std::string frame_id;
  StampFactory stamp_factory;
  std::string host{"127.0.0.1"};
  int port{8765};
  double angular_velocity_z{0.25};
  double yaw_oscillation_frequency_hz{0.25};
  ImuStrategySnapshot old_snapshot{};
};

std::unique_ptr<ImuStrategy> create_imu_strategy(const ImuStrategyConfig& config);

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_IMU_STRATEGY_FACTORY_HPP_
