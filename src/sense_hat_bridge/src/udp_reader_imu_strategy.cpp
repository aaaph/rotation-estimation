#include "sense_hat_bridge/udp_reader_imu_strategy.hpp"

#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

#include "sense_hat_bridge/imu_message.hpp"

namespace sense_hat_bridge {

namespace {

constexpr std::size_t kMaxDatagramBytes = 4096;

std::string quoted(std::string_view value) {
  return "\"" + std::string{value} + "\"";
}

std::string_view object_body(std::string_view json, std::string_view object_name) {
  const auto name_pos = json.find(quoted(object_name));
  if (name_pos == std::string_view::npos) {
    throw std::runtime_error{"Missing JSON object: " + std::string{object_name}};
  }

  const auto begin = json.find('{', name_pos);
  if (begin == std::string_view::npos) {
    throw std::runtime_error{"Missing JSON object body: " + std::string{object_name}};
  }

  const auto end = json.find('}', begin);
  if (end == std::string_view::npos) {
    throw std::runtime_error{"Unterminated JSON object: " + std::string{object_name}};
  }

  return json.substr(begin, end - begin + 1);
}

double number_field(std::string_view json, std::string_view field_name) {
  const auto name_pos = json.find(quoted(field_name));
  if (name_pos == std::string_view::npos) {
    throw std::runtime_error{"Missing JSON field: " + std::string{field_name}};
  }

  const auto value_begin = json.find(':', name_pos);
  if (value_begin == std::string_view::npos) {
    throw std::runtime_error{"Missing JSON value: " + std::string{field_name}};
  }

  std::string value{json.substr(value_begin + 1)};
  char* parsed_end = nullptr;
  const double parsed = std::strtod(value.c_str(), &parsed_end);
  if (parsed_end == value.c_str()) {
    throw std::runtime_error{"Invalid JSON number: " + std::string{field_name}};
  }
  return parsed;
}

sensor_msgs::msg::Imu imu_message_from_packet(std::string_view packet,
                                              const builtin_interfaces::msg::Time& stamp,
                                              std::string_view frame_id) {
  const auto gyro = object_body(packet, "gyro");
  const auto accel = object_body(packet, "accel");

  auto msg = new_imu_message(stamp, frame_id);
  msg.angular_velocity.x = number_field(gyro, "x");
  msg.angular_velocity.y = number_field(gyro, "y");
  msg.angular_velocity.z = number_field(gyro, "z");
  msg.linear_acceleration.x = number_field(accel, "x") * kGravityMetersPerSecondSquared;
  msg.linear_acceleration.y = number_field(accel, "y") * kGravityMetersPerSecondSquared;
  msg.linear_acceleration.z = number_field(accel, "z") * kGravityMetersPerSecondSquared;
  return msg;
}

}  // namespace

UdpReaderImuStrategy::UdpReaderImuStrategy(const std::string& host, int port, std::string frame_id,
                                           StampFactory stamp_factory)
    : frame_id_(std::move(frame_id)),
      stamp_factory_(std::move(stamp_factory)),
      socket_fd_(::socket(AF_INET, SOCK_DGRAM, 0)) {
  if (socket_fd_ < 0) {
    throw std::runtime_error{"Failed to create UDP socket: " + std::string{std::strerror(errno)}};
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
    close();
    throw std::runtime_error{"Invalid UDP host: " + host};
  }

  if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    close();
    throw std::runtime_error{"Failed to bind UDP socket: " + std::string{std::strerror(errno)}};
  }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): POSIX fcntl is a C vararg API.
  const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): POSIX fcntl is a C vararg API.
  if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) != 0) {
    close();
    throw std::runtime_error{"Failed to set UDP socket non-blocking"};
  }
}

UdpReaderImuStrategy::~UdpReaderImuStrategy() {
  close();
}

std::optional<sensor_msgs::msg::Imu> UdpReaderImuStrategy::get_imu_message() {
  std::array<char, kMaxDatagramBytes> buffer{};
  const auto byte_count = ::recvfrom(socket_fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);

  if (byte_count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return std::nullopt;
    }
    throw std::runtime_error{"Failed to receive UDP datagram: " +
                             std::string{std::strerror(errno)}};
  }

  return imu_message_from_packet(
      std::string_view{buffer.data(), static_cast<std::size_t>(byte_count)}, stamp_factory_(),
      frame_id_);
}

void UdpReaderImuStrategy::close() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

}  // namespace sense_hat_bridge
