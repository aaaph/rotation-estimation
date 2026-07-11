#include <Eigen/Geometry>
#include <cstdint>
#include <gtest/gtest.h>
#include <numbers>

#include "rotation_estimation/esekf.hpp"

namespace {

constexpr int64_t kOneSecondNs = 1'000'000'000LL;
constexpr double kHalfPi = std::numbers::pi / 2.0;

TEST(ESEKFTest, PredictWithZeroGyroKeepsIdentityOrientation) {
  rotation_estimation::ESEKF filter{};

  filter.predict(Eigen::Vector3d::Zero(), kOneSecondNs);

  EXPECT_NEAR(filter.state().q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
  EXPECT_EQ(filter.timestamp_ns(), kOneSecondNs);
}

TEST(ESEKFTest, PredictIntegratesYawRate) {
  rotation_estimation::ESEKF filter{};
  const Eigen::Vector3d gyro_rad_per_s{0.0, 0.0, kHalfPi};

  filter.predict(gyro_rad_per_s, kOneSecondNs);

  const Eigen::Quaterniond expected{Eigen::AngleAxisd{kHalfPi, Eigen::Vector3d::UnitZ()}};
  EXPECT_NEAR(filter.state().q.angularDistance(expected), 0.0, 1e-12);
}

TEST(ESEKFTest, PredictUsesGyroBias) {
  rotation_estimation::ESEKFState state{};
  state.gyro_bias = Eigen::Vector3d{0.0, 0.0, kHalfPi};

  rotation_estimation::ESEKF filter{};
  filter.reset(state);
  filter.predict(Eigen::Vector3d{0.0, 0.0, kHalfPi}, kOneSecondNs);

  EXPECT_NEAR(filter.state().q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
}

TEST(ESEKFTest, PredictKeepsCovarianceSymmetric) {
  rotation_estimation::ESEKF filter{};

  filter.predict(Eigen::Vector3d{0.1, -0.2, 0.3}, kOneSecondNs);

  const auto asymmetry = filter.state().covariance - filter.state().covariance.transpose();
  EXPECT_NEAR(asymmetry.norm(), 0.0, 1e-12);
}

TEST(ESEKFTest, PredictAddsGyroNoiseToOrientationCovariance) {
  rotation_estimation::ESEKFConfig config{};
  config.gyro_noise_std = 0.3;
  config.gyro_bias_random_walk_std = 0.0;

  rotation_estimation::ESEKFState state{};
  state.covariance.setZero();

  rotation_estimation::ESEKF filter{config};
  filter.reset(state);
  filter.predict(Eigen::Vector3d::Zero(), kOneSecondNs);

  const double expected_variance = config.gyro_noise_std * config.gyro_noise_std;
  for (Eigen::Index i = 0; i < 3; ++i) {
    EXPECT_NEAR(filter.state().covariance(i, i), expected_variance, 1e-12);
    EXPECT_NEAR(filter.state().covariance(i + 3, i + 3), 0.0, 1e-12);
  }
}

TEST(ESEKFTest, PredictAddsGyroBiasRandomWalkToBiasCovariance) {
  rotation_estimation::ESEKFConfig config{};
  config.gyro_noise_std = 0.0;
  config.gyro_bias_random_walk_std = 0.04;

  rotation_estimation::ESEKFState state{};
  state.covariance.setZero();

  rotation_estimation::ESEKF filter{config};
  filter.reset(state);
  filter.predict(Eigen::Vector3d::Zero(), kOneSecondNs);

  const double expected_variance =
      config.gyro_bias_random_walk_std * config.gyro_bias_random_walk_std;
  for (Eigen::Index i = 0; i < 3; ++i) {
    EXPECT_NEAR(filter.state().covariance(i, i), 0.0, 1e-12);
    EXPECT_NEAR(filter.state().covariance(i + 3, i + 3), expected_variance, 1e-12);
  }
}

}  // namespace
