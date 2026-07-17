#include "rotation_estimation/gyro_bias_sliding_window.hpp"

#include <stdexcept>

namespace rotation_estimation {

GyroBiasSlidingWindow::GyroBiasSlidingWindow(std::chrono::nanoseconds duration)
    : duration_(duration) {
  if (duration_ <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument{"gyro bias window duration must be positive"};
  }
}

void GyroBiasSlidingWindow::add(int64_t timestamp_ns, const Eigen::Vector3d& gyro_bias) {
  if (!samples_.empty() && timestamp_ns < samples_.back().timestamp_ns) {
    clear();
  }

  samples_.push_back(Sample{.timestamp_ns = timestamp_ns, .gyro_bias = gyro_bias});
  sum_ += gyro_bias;

  const int64_t oldest_timestamp_ns = timestamp_ns - duration_.count();
  while (!samples_.empty() && samples_.front().timestamp_ns < oldest_timestamp_ns) {
    sum_ -= samples_.front().gyro_bias;
    samples_.pop_front();
  }
}

void GyroBiasSlidingWindow::clear() {
  samples_.clear();
  sum_.setZero();
}

std::optional<Eigen::Vector3d> GyroBiasSlidingWindow::mean() const {
  if (samples_.empty()) {
    return std::nullopt;
  }
  return sum_ / static_cast<double>(samples_.size());
}

std::size_t GyroBiasSlidingWindow::size() const {
  return samples_.size();
}

}  // namespace rotation_estimation
