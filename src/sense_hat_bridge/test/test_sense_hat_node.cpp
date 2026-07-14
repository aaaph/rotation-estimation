#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <memory>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "sense_hat_bridge/imu_message.hpp"
#include "sense_hat_bridge/imu_strategy_factory.hpp"
#include "sense_hat_bridge/sense_hat_node.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace {

constexpr auto kTestTopic = "/test/sense_hat/imu_raw";
constexpr auto kTestFrameId = "test_sensehat_link";
constexpr auto kUdpHost = "127.0.0.1";
constexpr double kTargetAngularVelocityZ = 1.2;
constexpr double kValueTolerance = 1e-12;

class UdpSocket final {
 public:
  UdpSocket() : socket_fd_(::socket(AF_INET, SOCK_DGRAM, 0)) {
    if (socket_fd_ < 0) {
      throw std::system_error{errno, std::generic_category(), "Failed to create UDP socket"};
    }
  }

  ~UdpSocket() {
    if (socket_fd_ >= 0) {
      static_cast<void>(::close(socket_fd_));
    }
  }

  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;
  UdpSocket(UdpSocket&&) = delete;
  UdpSocket& operator=(UdpSocket&&) = delete;

  [[nodiscard]] int fd() const {
    return socket_fd_;
  }

 private:
  int socket_fd_{-1};
};

struct CapturedImu {
  bool received{false};
  sensor_msgs::msg::Imu msg;
};

template <typename Predicate>
bool spin_until(rclcpp::executors::SingleThreadedExecutor& executor, Predicate predicate,
                std::chrono::nanoseconds timeout = std::chrono::seconds{3}) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds{10});
  }
  return predicate();
}

rclcpp::NodeOptions node_options_with(const std::vector<rclcpp::Parameter>& overrides) {
  auto node_options = rclcpp::NodeOptions{};
  node_options.parameter_overrides(overrides);
  return node_options;
}

rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscribe_to_imu(
    const std::shared_ptr<rclcpp::Node>& helper_node, std::string_view topic,
    CapturedImu& captured_imu) {
  return helper_node->create_subscription<sensor_msgs::msg::Imu>(
      std::string{topic}, rclcpp::SensorDataQoS{},
      [&captured_imu](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        captured_imu.msg = *msg;
        captured_imu.received = true;
      });
}

std::shared_ptr<sense_hat_bridge::SenseHatNode> make_sense_hat_node(
    const std::vector<rclcpp::Parameter>& overrides) {
  return std::make_shared<sense_hat_bridge::SenseHatNode>(node_options_with(overrides));
}

int available_udp_port() {
  UdpSocket socket;
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = 0;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  if (::bind(socket.fd(), reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    throw std::system_error{errno, std::generic_category(), "Failed to bind UDP probe socket"};
  }

  auto address_size = static_cast<socklen_t>(sizeof(address));
  if (::getsockname(socket.fd(), reinterpret_cast<sockaddr*>(&address), &address_size) != 0) {
    throw std::system_error{errno, std::generic_category(), "Failed to read UDP probe port"};
  }

  return static_cast<int>(ntohs(address.sin_port));
}

void send_udp_packet(int port, std::string_view packet) {
  UdpSocket socket;
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, kUdpHost, &address.sin_addr) != 1) {
    throw std::runtime_error{"Invalid UDP test host"};
  }

  const auto byte_count = ::sendto(socket.fd(), packet.data(), packet.size(), 0,
                                   reinterpret_cast<const sockaddr*>(&address), sizeof(address));
  if (byte_count < 0) {
    throw std::system_error{errno, std::generic_category(), "Failed to send UDP packet"};
  }
  if (static_cast<std::size_t>(byte_count) != packet.size()) {
    throw std::runtime_error{"Failed to send complete UDP packet"};
  }
}

class SenseHatNodeTest : public ::testing::Test {
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

}  // namespace

TEST_F(SenseHatNodeTest, UsesDefaultNodeName) {
  EXPECT_EQ(std::string_view{sense_hat_bridge::SenseHatNode::kDefaultNodeName},
            "sense_hat_imu_node");
}

TEST_F(SenseHatNodeTest, PublishesStationaryMockImuMessage) {
  auto node = make_sense_hat_node(std::vector<rclcpp::Parameter>{
      rclcpp::Parameter{"mock", true},
      rclcpp::Parameter{"mock_mode", std::string{sense_hat_bridge::kMockModeStationary}},
      rclcpp::Parameter{"topic", std::string{kTestTopic}},
      rclcpp::Parameter{"frame_id", std::string{kTestFrameId}},
  });

  auto helper_node = std::make_shared<rclcpp::Node>("sense_hat_node_test_helper");
  auto captured_imu = CapturedImu();
  const auto subscriber = subscribe_to_imu(helper_node, kTestTopic, captured_imu);
  ASSERT_NE(subscriber, nullptr);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  ASSERT_TRUE(spin_until(executor, [&captured_imu]() { return captured_imu.received; }));
  EXPECT_EQ(captured_imu.msg.header.frame_id, kTestFrameId);
  EXPECT_DOUBLE_EQ(captured_imu.msg.orientation_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(captured_imu.msg.angular_velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(captured_imu.msg.angular_velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(captured_imu.msg.angular_velocity.z, 0.0);
  EXPECT_DOUBLE_EQ(captured_imu.msg.linear_acceleration.x, 0.0);
  EXPECT_DOUBLE_EQ(captured_imu.msg.linear_acceleration.y, 0.0);
  EXPECT_DOUBLE_EQ(captured_imu.msg.linear_acceleration.z,
                   sense_hat_bridge::kGravityMetersPerSecondSquared);
}

TEST_F(SenseHatNodeTest, ReconfiguresMockModeAtRuntime) {
  auto node = make_sense_hat_node(std::vector<rclcpp::Parameter>{
      rclcpp::Parameter{"mock", true},
      rclcpp::Parameter{"mock_mode", std::string{sense_hat_bridge::kMockModeStationary}},
      rclcpp::Parameter{"mock_angular_velocity_z", kTargetAngularVelocityZ},
      rclcpp::Parameter{"topic", std::string{kTestTopic}},
      rclcpp::Parameter{"frame_id", std::string{kTestFrameId}},
  });

  auto helper_node = std::make_shared<rclcpp::Node>("sense_hat_node_reconfigure_test_helper");
  auto captured_imu = CapturedImu();
  const auto subscriber = subscribe_to_imu(helper_node, kTestTopic, captured_imu);
  ASSERT_NE(subscriber, nullptr);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  ASSERT_TRUE(spin_until(executor, [&captured_imu]() { return captured_imu.received; }));
  EXPECT_DOUBLE_EQ(captured_imu.msg.angular_velocity.z, 0.0);

  captured_imu.received = false;
  const auto result = node->set_parameters_atomically(std::vector<rclcpp::Parameter>{
      rclcpp::Parameter{"mock_mode", std::string{sense_hat_bridge::kMockModeYawRotation}},
      rclcpp::Parameter{"mock_angular_velocity_z", kTargetAngularVelocityZ},
  });
  ASSERT_TRUE(result.successful) << result.reason;

  ASSERT_TRUE(spin_until(executor, [&captured_imu]() {
    return captured_imu.received && std::abs(captured_imu.msg.angular_velocity.z -
                                             kTargetAngularVelocityZ) <= kValueTolerance;
  }));
  EXPECT_EQ(captured_imu.msg.header.frame_id, kTestFrameId);
  EXPECT_DOUBLE_EQ(captured_imu.msg.linear_acceleration.z,
                   sense_hat_bridge::kGravityMetersPerSecondSquared);
}

TEST_F(SenseHatNodeTest, RejectsDynamicParametersInUdpMode) {
  const auto node = make_sense_hat_node(std::vector<rclcpp::Parameter>{
      rclcpp::Parameter{"mock", false},
      rclcpp::Parameter{"host", std::string{kUdpHost}},
      rclcpp::Parameter{"port", available_udp_port()},
      rclcpp::Parameter{"topic", std::string{kTestTopic}},
      rclcpp::Parameter{"frame_id", std::string{kTestFrameId}},
  });

  const auto result = node->set_parameters_atomically(std::vector<rclcpp::Parameter>{
      rclcpp::Parameter{"mock_angular_velocity_z", kTargetAngularVelocityZ},
  });

  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("dynamic parameters are disabled"), std::string::npos);
}

TEST_F(SenseHatNodeTest, PublishesUdpImuPacket) {
  constexpr auto kUdpTopic = "/test/sense_hat/udp_imu_raw";
  constexpr auto kUdpPacket =
      R"({"gyro":{"x":0.1,"y":-0.2,"z":0.3},"accel":{"x":0.25,"y":-0.5,"z":1.0}})";
  const auto port = available_udp_port();
  auto node = make_sense_hat_node(std::vector<rclcpp::Parameter>{
      rclcpp::Parameter{"mock", false},
      rclcpp::Parameter{"host", std::string{kUdpHost}},
      rclcpp::Parameter{"port", port},
      rclcpp::Parameter{"topic", std::string{kUdpTopic}},
      rclcpp::Parameter{"frame_id", std::string{kTestFrameId}},
  });

  auto helper_node = std::make_shared<rclcpp::Node>("sense_hat_node_udp_test_helper");
  auto captured_imu = CapturedImu();
  const auto subscriber = subscribe_to_imu(helper_node, kUdpTopic, captured_imu);
  ASSERT_NE(subscriber, nullptr);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  ASSERT_TRUE(spin_until(executor, [&captured_imu, port]() {
    send_udp_packet(port, kUdpPacket);
    return captured_imu.received;
  }));

  EXPECT_EQ(captured_imu.msg.header.frame_id, kTestFrameId);
  EXPECT_DOUBLE_EQ(captured_imu.msg.orientation_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(captured_imu.msg.angular_velocity.x, 0.1);
  EXPECT_DOUBLE_EQ(captured_imu.msg.angular_velocity.y, -0.2);
  EXPECT_DOUBLE_EQ(captured_imu.msg.angular_velocity.z, 0.3);
  EXPECT_NEAR(captured_imu.msg.linear_acceleration.x,
              0.25 * sense_hat_bridge::kGravityMetersPerSecondSquared, kValueTolerance);
  EXPECT_NEAR(captured_imu.msg.linear_acceleration.y,
              -0.5 * sense_hat_bridge::kGravityMetersPerSecondSquared, kValueTolerance);
  EXPECT_DOUBLE_EQ(captured_imu.msg.linear_acceleration.z,
                   sense_hat_bridge::kGravityMetersPerSecondSquared);
}
