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

TEST(ESEKFTest, CovarianceDiagonalReturnsStateCovarianceDiagonal) {
  rotation_estimation::ESEKFState state{};
  state.covariance.diagonal() << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  rotation_estimation::ESEKF filter{};
  filter.reset(state);

  EXPECT_TRUE(filter.covariance_diagonal().isApprox(state.covariance.diagonal()));
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

TEST(ESEKFTest, UpdateWithGravityAlignedAccelerationKeepsIdentityOrientation) {
  rotation_estimation::ESEKF filter{};

  filter.updateByAccel(Eigen::Vector3d{0.0, 0.0, 9.80665});

  EXPECT_NEAR(filter.state().q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
}

TEST(ESEKFTest, UpdateTiltsOrientationTowardAccelerometer) {
  rotation_estimation::ESEKFConfig config{};
  rotation_estimation::ESEKF filter{config};
  const double tilt_rad = 0.2;
  const Eigen::Quaterniond expected{Eigen::AngleAxisd{tilt_rad, Eigen::Vector3d::UnitX()}};
  const Eigen::Vector3d accel = config.g * (expected.inverse() * Eigen::Vector3d::UnitZ());

  filter.updateByAccel(accel);

  EXPECT_LT(filter.state().q.angularDistance(expected),
            Eigen::Quaterniond::Identity().angularDistance(expected));
}

TEST(ESEKFTest, UpdateRejectsAccelerationOutsideNormGate) {
  rotation_estimation::ESEKFConfig config{};
  config.accel_norm_gate = 0.01;

  rotation_estimation::ESEKF filter{config};
  filter.updateByAccel(Eigen::Vector3d{0.0, 0.0, 2.0 * config.g});

  EXPECT_NEAR(filter.state().q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
}

TEST(ESEKFTest, UpdateReducesObservableOrientationCovariance) {
  rotation_estimation::ESEKF filter{};

  filter.updateByAccel(Eigen::Vector3d{0.0, 0.0, 9.80665});

  EXPECT_LT(filter.state().covariance(0, 0), 1.0);
  EXPECT_LT(filter.state().covariance(1, 1), 1.0);
  EXPECT_NEAR(filter.state().covariance(2, 2), 1.0, 1e-12);
}

TEST(ESEKFTest, UpdateKeepsCovarianceSymmetric) {
  rotation_estimation::ESEKF filter{};

  filter.updateByAccel(Eigen::Vector3d{0.0, 0.1, 9.80665});

  const auto asymmetry = filter.state().covariance - filter.state().covariance.transpose();
  EXPECT_NEAR(asymmetry.norm(), 0.0, 1e-12);
}

TEST(ESEKFTest, UpdateByStationaryMovesGyroBiasTowardMeasuredGyro) {
  rotation_estimation::ESEKFConfig config{};
  config.stationary_gyro_noise_std = 0.1;
  const Eigen::Vector3d initial_bias{0.01, -0.02, 0.03};
  const Eigen::Vector3d gyro{0.11, -0.22, 0.33};

  rotation_estimation::ESEKFState state{};
  state.gyro_bias = initial_bias;

  rotation_estimation::ESEKF filter{config};
  filter.reset(state);
  filter.updateByStationary(gyro);

  const double gain =
      1.0 / (1.0 + config.stationary_gyro_noise_std * config.stationary_gyro_noise_std);
  const Eigen::Vector3d expected_bias = initial_bias + gain * (gyro - initial_bias);
  EXPECT_NEAR((filter.state().gyro_bias - expected_bias).norm(), 0.0, 1e-12);
}

TEST(ESEKFTest, UpdateByStationaryDoesNotChangeOrientationWithUncorrelatedCovariance) {
  rotation_estimation::ESEKF filter{};

  filter.updateByStationary(Eigen::Vector3d{0.1, -0.2, 0.3});

  EXPECT_NEAR(filter.state().q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
}

TEST(ESEKFTest, UpdateByStationaryReducesGyroBiasCovariance) {
  rotation_estimation::ESEKF filter{};

  filter.updateByStationary(Eigen::Vector3d::Zero());

  for (Eigen::Index i = 0; i < 3; ++i) {
    EXPECT_NEAR(filter.state().covariance(i, i), 1.0, 1e-12);
    EXPECT_LT(filter.state().covariance(i + 3, i + 3), 1.0);
  }
}

TEST(ESEKFTest, UpdateByStationaryKeepsCovarianceSymmetric) {
  rotation_estimation::ESEKFState state{};
  state.covariance(0, 3) = 0.1;
  state.covariance(3, 0) = 0.1;
  state.covariance(1, 4) = -0.2;
  state.covariance(4, 1) = -0.2;

  rotation_estimation::ESEKF filter{};
  filter.reset(state);
  filter.updateByStationary(Eigen::Vector3d{0.1, -0.2, 0.3});

  const auto asymmetry = filter.state().covariance - filter.state().covariance.transpose();
  EXPECT_NEAR(asymmetry.norm(), 0.0, 1e-12);
}

}  // namespace
