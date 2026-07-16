#include <chrono>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "rclcpp/client.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rotation_estimation_bringup/manager_node.hpp"

namespace {

using SetSystemMode = rotation_estimation_bringup::srv::SetSystemMode;
using namespace std::chrono_literals;

constexpr auto kServiceName = "/manager_node/set_system_mode";
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
  void SetUp() override {
    manager_node_ = std::make_shared<rotation_estimation_bringup::ManagerNode>();
    client_node_ = std::make_shared<rclcpp::Node>("manager_node_test_client");
    client_ = client_node_->create_client<SetSystemMode>(kServiceName);

    executor_.add_node(manager_node_);
    executor_.add_node(client_node_);
    ASSERT_TRUE(client_->wait_for_service(kServiceTimeout));
  }

  void TearDown() override {
    executor_.remove_node(client_node_);
    executor_.remove_node(manager_node_);
    client_.reset();
    client_node_.reset();
    manager_node_.reset();
  }

  SetSystemMode::Response::SharedPtr callSetSystemMode(const std::string& mode) {
    auto request = std::make_shared<SetSystemMode::Request>();
    request->mode = mode;
    auto future = client_->async_send_request(request);

    if (executor_.spin_until_future_complete(future, kServiceTimeout) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

 private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<rotation_estimation_bringup::ManagerNode> manager_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<SetSystemMode>::SharedPtr client_;
};

TEST_F(ManagerNodeServiceTest, SwitchesToMobileMode) {
  const auto response = callSetSystemMode("mobile");

  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->active_mode, "mobile");
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

}  // namespace
