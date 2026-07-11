#include "rotation_estimation/esekf.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace rotation_estimation {
namespace {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3x6d = Eigen::Matrix<double, 3, 6>;
using Matrix6x3d = Eigen::Matrix<double, 6, 3>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

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

void Symmetrize(Matrix6d& covariance) {
  covariance = (covariance + covariance.transpose()) / 2.0;
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
  Symmetrize(state_.covariance);
}

void ESEKF::updateByAccel(const Eigen::Vector3d& accel) {
  const double accel_norm = accel.norm();
  if (options_.g <= 0.0 || accel_norm <= 1e-9) {
    return;
  }
  if (std::abs(accel_norm - options_.g) > options_.accel_norm_gate) {
    return;
  }

  const Eigen::Vector3d expected_accel =
      options_.g * (state_.q.inverse() * Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d residual = accel - expected_accel;

  Matrix3x6d H = Matrix3x6d::Zero();
  H.block<3, 3>(0, 0) = Skew(expected_accel);

  const Eigen::Matrix3d R =
      options_.accel_noise_std * options_.accel_noise_std * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d S = H * state_.covariance * H.transpose() + R;
  const Eigen::LDLT<Eigen::Matrix3d> solver{S};
  if (solver.info() != Eigen::Success) {
    return;
  }

  const Matrix6x3d PHT = state_.covariance * H.transpose();
  const Matrix6x3d K = solver.solve(PHT.transpose()).transpose();
  const Vector6d correction = K * residual;

  const Eigen::Quaterniond dq = Eigen::Quaterniond(ExpSO3Matrix(correction.head<3>()));
  state_.q = (state_.q * dq).normalized();
  state_.gyro_bias += correction.tail<3>();

  const Matrix6d I = Matrix6d::Identity();
  const Matrix6d IKH = I - K * H;
  state_.covariance = IKH * state_.covariance * IKH.transpose() + K * R * K.transpose();
  Symmetrize(state_.covariance);
}

void ESEKF::updateByStationary(const Eigen::Vector3d& gyro) {
  Matrix3x6d H = Matrix3x6d::Zero();
  H.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
  const Eigen::Vector3d residual = Eigen::Vector3d::Zero() - (gyro - state_.gyro_bias);

  const Eigen::Matrix3d R = options_.stationary_gyro_noise_std *
                            options_.stationary_gyro_noise_std * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d S = H * state_.covariance * H.transpose() + R;

  const Eigen::LDLT<Eigen::Matrix3d> solver{S};
  if (solver.info() != Eigen::Success) {
    return;
  }

  const Matrix6x3d PHT = state_.covariance * H.transpose();
  const Matrix6x3d K = solver.solve(PHT.transpose()).transpose();

  const Vector6d correction = K * residual;

  const Eigen::Quaterniond dq = Eigen::Quaterniond(ExpSO3Matrix(correction.head<3>()));
  state_.q = (state_.q * dq).normalized();
  state_.gyro_bias += correction.tail<3>();

  const Matrix6d I = Matrix6d::Identity();
  const Matrix6d IKH = I - K * H;
  state_.covariance = IKH * state_.covariance * IKH.transpose() + K * R * K.transpose();
  Symmetrize(state_.covariance);
}
}  // namespace rotation_estimation
