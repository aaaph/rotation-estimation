#pragma once
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <cstdint>
namespace rotation_estimation {

struct ESEKFConfig {
  double gyro_noise_std = 0.02;
  double gyro_bias_random_walk_std = 0.001;
  double accel_noise_std = 0.2;
  double g = 9.80665;
  double accel_norm_gate = 1.0;
};

struct ESEKFState {
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
};

class ESEKF {
 public:
  explicit ESEKF(ESEKFConfig options = {}, int64_t timestamp_ns = 0LL);
  [[nodiscard]] const ESEKFState& state() const;
  [[nodiscard]] int64_t timestamp_ns() const;
  void reset(const ESEKFState& state);
  void predict(const Eigen::Vector3d& gyro, int64_t timestamp_ns_next);
  void update(const Eigen::Vector3d& accel);

 private:
  ESEKFConfig options_;
  ESEKFState state_;
  int64_t timestamp_ns_ = 0LL;
};

}  // namespace rotation_estimation
