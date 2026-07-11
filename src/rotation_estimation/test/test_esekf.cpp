#include <Eigen/Geometry>
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
  EXPECT_DOUBLE_EQ(filter.timestamp_ns(), kOneSecondNs);
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

}  // namespace
