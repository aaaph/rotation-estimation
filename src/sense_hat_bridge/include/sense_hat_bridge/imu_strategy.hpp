#ifndef SENSE_HAT_BRIDGE_IMU_STRATEGY_HPP_
#define SENSE_HAT_BRIDGE_IMU_STRATEGY_HPP_

#include <functional>
#include <optional>

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace sense_hat_bridge {

using StampFactory = std::function<builtin_interfaces::msg::Time()>;

struct ImuStrategySnapshot {
  double angular_velocity_z{0.0};
};
class ImuStrategy {
 public:
  ImuStrategy() = default;
  ImuStrategy(const ImuStrategy&) = delete;
  ImuStrategy& operator=(const ImuStrategy&) = delete;
  ImuStrategy(ImuStrategy&&) = delete;
  ImuStrategy& operator=(ImuStrategy&&) = delete;
  virtual ~ImuStrategy() = default;

  [[nodiscard]] virtual ImuStrategySnapshot snapshot() const {
    return {};
  }

  virtual std::optional<sensor_msgs::msg::Imu> get_imu_message() = 0;
  virtual void close() {}
};

}  // namespace sense_hat_bridge

#endif  // SENSE_HAT_BRIDGE_IMU_STRATEGY_HPP_
