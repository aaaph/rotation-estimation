#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/service.hpp>

#include "rotation_estimation_bringup/srv/set_mock_strategy.hpp"
#include "rotation_estimation_bringup/srv/set_system_mode.hpp"

namespace rotation_estimation_bringup {

using SetMockStrategy = rotation_estimation_bringup::srv::SetMockStrategy;
using SetSystemMode = rotation_estimation_bringup::srv::SetSystemMode;

enum class SystemMode : std::uint8_t {
  kStationary,
  kMobile,
};

class ManagerNode : public rclcpp::Node {
 public:
  explicit ManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  SystemMode system_mode_ = SystemMode::kStationary;
  bool mock_imu_enabled_ = false;
  std::string mobile_mock_strategy_ = "yaw_rotation";

  void handleSetSystemMode(const SetSystemMode::Request& request,
                           SetSystemMode::Response& response);
  void handleSetMockStrategy(const SetMockStrategy::Request& request,
                             SetMockStrategy::Response& response);
  [[nodiscard]] std::optional<std::string> setEkfMode(SystemMode mode);
  [[nodiscard]] std::optional<std::string> setSenseHatMode(SystemMode mode);
  [[nodiscard]] std::optional<std::string> setSenseHatMockStrategy(std::string_view strategy);

  rclcpp::Service<SetSystemMode>::SharedPtr set_system_mode_service_;
  rclcpp::Service<SetMockStrategy>::SharedPtr set_mock_strategy_service_;

  rclcpp::CallbackGroup::SharedPtr parameters_callback_group_;
  rclcpp::AsyncParametersClient::SharedPtr ekf_parameters_client_;
  rclcpp::AsyncParametersClient::SharedPtr sense_hat_parameters_client_;
};
}  // namespace rotation_estimation_bringup
