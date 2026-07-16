#include "rotation_estimation_bringup/manager_node.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include <rclcpp/create_service.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>

#include "rclcpp/node_options.hpp"

namespace rotation_estimation_bringup {
namespace {
using namespace std::chrono_literals;

constexpr auto kParameterServiceTimeout = 2s;
constexpr auto kParameterRequestTimeout = 2s;
constexpr std::string_view kDefaultMobileMockStrategy = "yaw_rotation";

std::optional<SystemMode> systemModeFromString(std::string_view mode) {
  if (mode == "stationary") {
    return SystemMode::kStationary;
  }
  if (mode == "mobile") {
    return SystemMode::kMobile;
  }
  return std::nullopt;
}

std::string_view systemModeToString(SystemMode mode) {
  return mode == SystemMode::kStationary ? "stationary" : "mobile";
}

bool isMobileMockStrategy(std::string_view strategy) {
  return strategy == "yaw_rotation" || strategy == "yaw_oscillation";
}

std::optional<std::string> setRemoteParameter(rclcpp::AsyncParametersClient& client,
                                              const rclcpp::Parameter& parameter) {
  if (!client.wait_for_service(kParameterServiceTimeout)) {
    return "parameter services are unavailable";
  }

  try {
    auto future = client.set_parameters({parameter});
    if (future.wait_for(kParameterRequestTimeout) != std::future_status::ready) {
      return "parameter request timed out";
    }

    const auto& results = future.get();
    if (results.size() != 1) {
      return "parameter service returned an unexpected result count";
    }
    if (!results.front().successful) {
      return results.front().reason;
    }
  } catch (const std::exception& error) {
    return error.what();
  }

  return std::nullopt;
}
}  // namespace

ManagerNode::ManagerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("manager_node", options) {
  declare_parameter("system_mode", "stationary");
  mock_imu_enabled_ = declare_parameter<bool>("mock_imu", false);
  mobile_mock_strategy_ = declare_parameter<std::string>("mobile_mock_strategy",
                                                         std::string{kDefaultMobileMockStrategy});

  const auto system_mode_arg = get_parameter("system_mode").as_string();
  if (system_mode_arg == "stationary") {
    system_mode_ = SystemMode::kStationary;
  } else if (system_mode_arg == "mobile") {
    system_mode_ = SystemMode::kMobile;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid system mode: %s", system_mode_arg.c_str());
    throw std::invalid_argument("Invalid system mode: " + system_mode_arg);
  }
  if (!isMobileMockStrategy(mobile_mock_strategy_)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid mobile mock strategy: %s",
                 mobile_mock_strategy_.c_str());
    throw std::invalid_argument("Invalid mobile mock strategy: " + mobile_mock_strategy_);
  }

  parameters_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  ekf_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "/ekf_node", rclcpp::ParametersQoS(), parameters_callback_group_);

  sense_hat_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "/sense_hat_imu_node", rclcpp::ParametersQoS(), parameters_callback_group_);

  // rclcpp requires service callback shared pointers to be passed by value.
  // NOLINTBEGIN(performance-unnecessary-value-param)
  set_system_mode_service_ = create_service<SetSystemMode>(
      "~/set_system_mode", [this](SetSystemMode::Request::SharedPtr request,
                                  SetSystemMode::Response::SharedPtr response) {
        this->handleSetSystemMode(*request, *response);
      });

  set_mock_strategy_service_ = create_service<SetMockStrategy>(
      "~/set_mock_strategy", [this](SetMockStrategy::Request::SharedPtr request,
                                    SetMockStrategy::Response::SharedPtr response) {
        this->handleSetMockStrategy(*request, *response);
      });
  // NOLINTEND(performance-unnecessary-value-param)

  RCLCPP_INFO(this->get_logger(), "Manager node initialized");
}

std::optional<std::string> ManagerNode::setEkfMode(SystemMode mode) {
  const std::string_view ekf_mode = mode == SystemMode::kStationary ? "stationary" : "accel";
  auto error =
      setRemoteParameter(*ekf_parameters_client_, rclcpp::Parameter{"mode", std::string{ekf_mode}});
  if (error.has_value()) {
    return "ekf_node: " + *error;
  }
  return std::nullopt;
}

std::optional<std::string> ManagerNode::setSenseHatMode(SystemMode mode) {
  const std::string_view sense_hat_mode = mode == SystemMode::kStationary
                                              ? std::string_view{"stationary"}
                                              : std::string_view{mobile_mock_strategy_};
  return setSenseHatMockStrategy(sense_hat_mode);
}

std::optional<std::string> ManagerNode::setSenseHatMockStrategy(std::string_view strategy) {
  auto error = setRemoteParameter(*sense_hat_parameters_client_,
                                  rclcpp::Parameter{"mock_mode", std::string{strategy}});
  if (error.has_value()) {
    return "sense_hat_imu_node: " + *error;
  }
  return std::nullopt;
}

void ManagerNode::handleSetSystemMode(const SetSystemMode::Request& request,
                                      SetSystemMode::Response& response) {
  const auto requested_mode = systemModeFromString(request.mode);
  if (!requested_mode.has_value()) {
    response.success = false;
    response.active_mode = systemModeToString(system_mode_);
    response.message = "Invalid system mode: " + request.mode;
    return;
  }
  if (*requested_mode == system_mode_) {
    response.success = true;
    response.active_mode = systemModeToString(system_mode_);
    response.message = "No change";
    return;
  }

  const auto previous_mode = system_mode_;

  if (!mock_imu_enabled_) {
    auto error = setEkfMode(*requested_mode);
    if (error.has_value()) {
      response.success = false;
      response.active_mode = systemModeToString(system_mode_);
      response.message = "Failed to change system mode: " + *error;
      return;
    }

    system_mode_ = *requested_mode;
    response.success = true;
    response.active_mode = systemModeToString(system_mode_);
    response.message = "Done";
    return;
  }

  const bool switching_to_mobile = *requested_mode == SystemMode::kMobile;

  auto first_error =
      switching_to_mobile ? setEkfMode(*requested_mode) : setSenseHatMode(*requested_mode);
  if (first_error.has_value()) {
    response.success = false;
    response.active_mode = systemModeToString(system_mode_);
    response.message = "Failed to change system mode: " + *first_error;
    return;
  }

  auto second_error =
      switching_to_mobile ? setSenseHatMode(*requested_mode) : setEkfMode(*requested_mode);
  if (second_error.has_value()) {
    auto rollback_error =
        switching_to_mobile ? setEkfMode(previous_mode) : setSenseHatMode(previous_mode);

    response.success = false;
    response.active_mode = systemModeToString(system_mode_);
    response.message = "Failed to change system mode: " + *second_error;
    if (rollback_error.has_value()) {
      response.message += "; rollback failed: " + *rollback_error;
    }
    return;
  }

  system_mode_ = *requested_mode;
  response.success = true;
  response.active_mode = systemModeToString(system_mode_);
  response.message = "Done";
}

void ManagerNode::handleSetMockStrategy(const SetMockStrategy::Request& request,
                                        SetMockStrategy::Response& response) {
  response.active_strategy = mobile_mock_strategy_;

  if (!mock_imu_enabled_) {
    response.success = false;
    response.message = "Mock strategy cannot be changed while UDP input is active";
    return;
  }
  if (!isMobileMockStrategy(request.strategy)) {
    response.success = false;
    response.message = "Invalid mock strategy: " + request.strategy;
    return;
  }
  if (request.strategy == mobile_mock_strategy_) {
    response.success = true;
    response.message = "No change";
    return;
  }

  if (system_mode_ == SystemMode::kMobile) {
    auto error = setSenseHatMockStrategy(request.strategy);
    if (error.has_value()) {
      response.success = false;
      response.message = "Failed to change mock strategy: " + *error;
      return;
    }
  }

  mobile_mock_strategy_ = request.strategy;
  (void)set_parameter(rclcpp::Parameter{"mobile_mock_strategy", mobile_mock_strategy_});
  response.success = true;
  response.active_strategy = mobile_mock_strategy_;
  response.message =
      system_mode_ == SystemMode::kStationary ? "Saved for the next mobile mode" : "Done";
}
}  // namespace rotation_estimation_bringup
