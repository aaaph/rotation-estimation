#pragma once

#include <Eigen/Core>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace rotation_estimation {

class GyroBiasSlidingWindow {
 public:
  static constexpr auto kDefaultDuration = std::chrono::seconds{10};

  explicit GyroBiasSlidingWindow(std::chrono::nanoseconds duration = kDefaultDuration);

  void add(int64_t timestamp_ns, const Eigen::Vector3d& gyro_bias);
  void clear();

  [[nodiscard]] std::optional<Eigen::Vector3d> mean() const;
  [[nodiscard]] std::size_t size() const;

 private:
  struct Sample {
    int64_t timestamp_ns;
    Eigen::Vector3d gyro_bias;
  };

  std::chrono::nanoseconds duration_;
  std::deque<Sample> samples_;
  Eigen::Vector3d sum_ = Eigen::Vector3d::Zero();
};

}  // namespace rotation_estimation
