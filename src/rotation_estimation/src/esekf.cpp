#include "rotation_estimation/esekf.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace rotation_estimation {
namespace {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

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

  Matrix6d Fc = Matrix6d::Zero();
  Fc.block<3, 3>(0, 0) = -Skew(gyro_hat);
  Fc.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

  Matrix6d Gc = Matrix6d::Zero();
  Gc.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
  Gc.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  Matrix6d Qc = Matrix6d::Zero();
  Qc.block<3, 3>(0, 0) = std::pow(options_.gyro_noise_std, 2) * Eigen::Matrix3d::Identity();
  Qc.block<3, 3>(3, 3) =
      std::pow(options_.gyro_bias_random_walk_std, 2) * Eigen::Matrix3d::Identity();

  const Matrix6d F = Matrix6d::Identity() + Fc * dt;
  const Matrix6d Qd = Gc * Qc * Gc.transpose() * dt;

  const Eigen::Quaterniond dq = Eigen::Quaterniond(ExpSO3Matrix(gyro_hat * dt));

  state_.q = (state_.q * dq).normalized();
  state_.covariance = F * state_.covariance * F.transpose() + Qd;
  state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
void ESEKF::update(const Eigen::Vector3d& accel) {
  (void)accel;
}
}  // namespace rotation_estimation
