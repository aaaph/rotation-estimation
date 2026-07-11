#include "rotation_estimation/esekf.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace rotation_estimation {
namespace {

Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

Eigen::Matrix3d ExpSO3Matrix(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const auto K = Skew(phi);
  if (theta < 1e-12) {
    return I + K;
  }
  return I + (std::sin(theta) / theta) * K + ((1.0 - std::cos(theta)) / (theta * theta)) * K * K;
}

}  // namespace

ESEKF::ESEKF(ESEKFConfig options, int64_t timestamp_ns)
    : options_(options), timestamp_ns_(timestamp_ns) {
  state_.covariance.setIdentity();
  state_.q.normalize();
}

const ESEKFState& ESEKF::state() const {
  return state_;
}

int64_t ESEKF::timestamp_ns() const {
  return timestamp_ns_;
}

void ESEKF::reset(const ESEKFState& state) {
  state_ = state;
  state_.q.normalize();
}

void ESEKF::predict(const Eigen::Vector3d& gyro, int64_t timestamp_ns_next) {
  const auto dt_ns = (timestamp_ns_next - timestamp_ns_);
  if (dt_ns <= 0) {
    return;
  }
  const double dt = static_cast<double>(dt_ns) * 1e-9;

  timestamp_ns_ = timestamp_ns_next;

  Eigen::Vector3d gyro_hat = gyro - state_.gyro_bias;
  const Eigen::Quaterniond dq = Eigen::Quaterniond(ExpSO3Matrix(gyro_hat * dt));
  state_.q = (state_.q * dq).normalized();
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
void ESEKF::update(const Eigen::Vector3d& accel) {
  (void)accel;
}
}  // namespace rotation_estimation
