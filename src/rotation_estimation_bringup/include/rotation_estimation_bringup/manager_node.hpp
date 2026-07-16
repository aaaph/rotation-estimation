#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include "rotation_estimation_bringup/srv/set_system_mode.hpp"

namespace rotation_estimation_bringup {

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
  void handleSetSystemMode(const SetSystemMode::Request& request,
                           SetSystemMode::Response& response);

  rclcpp::Service<SetSystemMode>::SharedPtr set_system_mode_service_;
};
}  // namespace rotation_estimation_bringup
