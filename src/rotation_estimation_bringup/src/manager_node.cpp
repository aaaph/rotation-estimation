#include "rotation_estimation_bringup/manager_node.hpp"

#include <optional>
#include <string_view>

#include <rclcpp/create_service.hpp>

#include "rclcpp/node_options.hpp"

namespace rotation_estimation_bringup {
namespace {
std::optional<SystemMode> systemModeFromString(std::string_view mode) {
  if (mode == "stationary") {
    return SystemMode::kStationary;
  }
  if (mode == "mobile") {
    return SystemMode::kMobile;
  }
  return std::nullopt;
}
}  // namespace

ManagerNode::ManagerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("manager_node", options) {
  declare_parameter("system_mode", "stationary");
  auto system_mode_arg = get_parameter("system_mode").as_string();
  if (system_mode_arg == "stationary") {
    system_mode_ = SystemMode::kStationary;
  } else if (system_mode_arg == "mobile") {
    system_mode_ = SystemMode::kMobile;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid system mode: %s", system_mode_arg.c_str());
    throw std::invalid_argument("Invalid system mode: " + system_mode_arg);
  }

  // rclcpp requires service callback shared pointers to be passed by value.
  // NOLINTBEGIN(performance-unnecessary-value-param)
  set_system_mode_service_ = create_service<SetSystemMode>(
      "~/set_system_mode", [this](SetSystemMode::Request::SharedPtr request,
                                  SetSystemMode::Response::SharedPtr response) {
        this->handleSetSystemMode(*request, *response);
      });
  // NOLINTEND(performance-unnecessary-value-param)
  RCLCPP_INFO(this->get_logger(), "Manager node initialized");
}

void ManagerNode::handleSetSystemMode(const SetSystemMode::Request& request,
                                      SetSystemMode::Response& response) {
  const auto requested_mode = systemModeFromString(request.mode);
  if (!requested_mode.has_value()) {
    response.success = false;
    response.active_mode = system_mode_ == SystemMode::kStationary ? "stationary" : "mobile";
    response.message = "Invalid system mode: " + request.mode;
    return;
  }
  if (*requested_mode == system_mode_) {
    response.success = true;
    response.active_mode = requested_mode == SystemMode::kStationary ? "stationary" : "mobile";
    response.message = "No change";
    return;
  }
  system_mode_ = *requested_mode;
  response.success = true;
  response.active_mode = requested_mode == SystemMode::kStationary ? "stationary" : "mobile";
  response.message = "Done";
}
}  // namespace rotation_estimation_bringup
