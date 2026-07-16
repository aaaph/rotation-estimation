#include <chrono>
#include <future>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/client.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rotation_estimation_bringup/manager_node.hpp"

namespace {

using SetMockStrategy = rotation_estimation_bringup::srv::SetMockStrategy;
using SetSystemMode = rotation_estimation_bringup::srv::SetSystemMode;
using namespace std::chrono_literals;

constexpr auto kSystemModeServiceName = "/manager_node/set_system_mode";
constexpr auto kMockStrategyServiceName = "/manager_node/set_mock_strategy";
constexpr auto kServiceTimeout = 3s;

class ManagerNodeRosTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class ManagerNodeServiceTest : public ManagerNodeRosTest {
 protected:
  [[nodiscard]] virtual bool mockImuEnabled() const {
    return true;
  }

  void SetUp() override {
    auto manager_options = rclcpp::NodeOptions{};
    manager_options.parameter_overrides({
        rclcpp::Parameter{"mock_imu", mockImuEnabled()},
        rclcpp::Parameter{"mobile_mock_strategy", "yaw_rotation"},
    });
    manager_node_ = std::make_shared<rotation_estimation_bringup::ManagerNode>(manager_options);
    ekf_node_ = std::make_shared<rclcpp::Node>("ekf_node");
    ekf_node_->declare_parameter("mode", "stationary");
    sense_hat_node_ = std::make_shared<rclcpp::Node>("sense_hat_imu_node");
    sense_hat_node_->declare_parameter("mock_mode", "stationary");
    client_node_ = std::make_shared<rclcpp::Node>("manager_node_test_client");
    system_mode_client_ = client_node_->create_client<SetSystemMode>(kSystemModeServiceName);
    mock_strategy_client_ = client_node_->create_client<SetMockStrategy>(kMockStrategyServiceName);

    executor_.add_node(manager_node_);
    executor_.add_node(ekf_node_);
    executor_.add_node(sense_hat_node_);
    executor_.add_node(client_node_);
    executor_thread_ = std::thread([this]() { executor_.spin(); });
    ASSERT_TRUE(system_mode_client_->wait_for_service(kServiceTimeout));
    ASSERT_TRUE(mock_strategy_client_->wait_for_service(kServiceTimeout));
  }

  void TearDown() override {
    executor_.cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    executor_.remove_node(client_node_);
    executor_.remove_node(sense_hat_node_);
    executor_.remove_node(ekf_node_);
    executor_.remove_node(manager_node_);
    mock_strategy_client_.reset();
    system_mode_client_.reset();
    client_node_.reset();
    sense_hat_node_.reset();
    ekf_node_.reset();
    manager_node_.reset();
  }

  SetSystemMode::Response::SharedPtr callSetSystemMode(const std::string& mode) const {
    auto request = std::make_shared<SetSystemMode::Request>();
    request->mode = mode;
    auto future = system_mode_client_->async_send_request(request);

    if (future.wait_for(kServiceTimeout) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  SetMockStrategy::Response::SharedPtr callSetMockStrategy(const std::string& strategy) const {
    auto request = std::make_shared<SetMockStrategy::Request>();
    request->strategy = strategy;
    auto future = mock_strategy_client_->async_send_request(request);

    if (future.wait_for(kServiceTimeout) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

 public:
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions{}, 2};
  std::thread executor_thread_;
  std::shared_ptr<rotation_estimation_bringup::ManagerNode> manager_node_;
  rclcpp::Node::SharedPtr ekf_node_;
  rclcpp::Node::SharedPtr sense_hat_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<SetSystemMode>::SharedPtr system_mode_client_;
  rclcpp::Client<SetMockStrategy>::SharedPtr mock_strategy_client_;
};

class ManagerNodeProductionServiceTest : public ManagerNodeServiceTest {
 protected:
  [[nodiscard]] bool mockImuEnabled() const override {
    return false;
  }
};

TEST_F(ManagerNodeServiceTest, SwitchesToMobileMode) {
  const auto response = callSetSystemMode("mobile");

  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->active_mode, "mobile");
  EXPECT_EQ(ekf_node_->get_parameter("mode").as_string(), "accel");
  EXPECT_EQ(sense_hat_node_->get_parameter("mock_mode").as_string(), "yaw_rotation");
}

TEST_F(ManagerNodeServiceTest, SwitchesBackToStationaryMode) {
  const auto mobile_response = callSetSystemMode("mobile");
  ASSERT_NE(mobile_response, nullptr);
  ASSERT_TRUE(mobile_response->success);

  const auto stationary_response = callSetSystemMode("stationary");

  ASSERT_NE(stationary_response, nullptr);
  EXPECT_TRUE(stationary_response->success);
  EXPECT_EQ(stationary_response->active_mode, "stationary");
  EXPECT_EQ(ekf_node_->get_parameter("mode").as_string(), "stationary");
  EXPECT_EQ(sense_hat_node_->get_parameter("mock_mode").as_string(), "stationary");
}

TEST_F(ManagerNodeServiceTest, RepeatedModeIsIdempotent) {
  const auto first_response = callSetSystemMode("mobile");
  ASSERT_NE(first_response, nullptr);
  ASSERT_TRUE(first_response->success);

  const auto repeated_response = callSetSystemMode("mobile");
  ASSERT_NE(repeated_response, nullptr);
  EXPECT_TRUE(repeated_response->success);
  EXPECT_EQ(repeated_response->active_mode, "mobile");
  EXPECT_EQ(repeated_response->message, "No change");
}

TEST_F(ManagerNodeServiceTest, SavesMockStrategyWhileStationary) {
  const auto strategy_response = callSetMockStrategy("yaw_oscillation");

  ASSERT_NE(strategy_response, nullptr);
  ASSERT_TRUE(strategy_response->success);
  EXPECT_EQ(strategy_response->active_strategy, "yaw_oscillation");
  EXPECT_EQ(strategy_response->message, "Saved for the next mobile mode");
  EXPECT_EQ(sense_hat_node_->get_parameter("mock_mode").as_string(), "stationary");

  const auto mobile_response = callSetSystemMode("mobile");
  ASSERT_NE(mobile_response, nullptr);
  EXPECT_TRUE(mobile_response->success);
  EXPECT_EQ(sense_hat_node_->get_parameter("mock_mode").as_string(), "yaw_oscillation");
}

TEST_F(ManagerNodeServiceTest, AppliesMockStrategyWhileMobile) {
  const auto mobile_response = callSetSystemMode("mobile");
  ASSERT_NE(mobile_response, nullptr);
  ASSERT_TRUE(mobile_response->success);

  const auto strategy_response = callSetMockStrategy("yaw_oscillation");

  ASSERT_NE(strategy_response, nullptr);
  EXPECT_TRUE(strategy_response->success);
  EXPECT_EQ(strategy_response->active_strategy, "yaw_oscillation");
  EXPECT_EQ(sense_hat_node_->get_parameter("mock_mode").as_string(), "yaw_oscillation");
  EXPECT_EQ(ekf_node_->get_parameter("mode").as_string(), "accel");
}

TEST_F(ManagerNodeServiceTest, RejectsInvalidMockStrategy) {
  const auto response = callSetMockStrategy("invalid");

  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->active_strategy, "yaw_rotation");
  EXPECT_FALSE(response->message.empty());
}

TEST_F(ManagerNodeProductionServiceTest, RejectsMockStrategyAndSwitchesOnlyEkf) {
  const auto strategy_response = callSetMockStrategy("yaw_oscillation");

  ASSERT_NE(strategy_response, nullptr);
  EXPECT_FALSE(strategy_response->success);
  EXPECT_EQ(strategy_response->active_strategy, "yaw_rotation");

  const auto mobile_response = callSetSystemMode("mobile");
  ASSERT_NE(mobile_response, nullptr);
  EXPECT_TRUE(mobile_response->success);
  EXPECT_EQ(ekf_node_->get_parameter("mode").as_string(), "accel");
  EXPECT_EQ(sense_hat_node_->get_parameter("mock_mode").as_string(), "stationary");
}

TEST_F(ManagerNodeServiceTest, RejectsInvalidModeAndKeepsCurrentMode) {
  const auto response = callSetSystemMode("invalid");

  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->active_mode, "stationary");
  EXPECT_FALSE(response->message.empty());
}

TEST_F(ManagerNodeRosTest, RejectsInvalidStartupMode) {
  auto options = rclcpp::NodeOptions{};
  options.parameter_overrides({rclcpp::Parameter{"system_mode", "invalid"}});

  EXPECT_THROW(
      { auto invalid_node = std::make_shared<rotation_estimation_bringup::ManagerNode>(options); },
      std::invalid_argument);
}

TEST_F(ManagerNodeRosTest, RejectsInvalidStartupMockStrategy) {
  auto options = rclcpp::NodeOptions{};
  options.parameter_overrides({rclcpp::Parameter{"mobile_mock_strategy", "invalid"}});

  EXPECT_THROW(
      { auto invalid_node = std::make_shared<rotation_estimation_bringup::ManagerNode>(options); },
      std::invalid_argument);
}

}  // namespace
