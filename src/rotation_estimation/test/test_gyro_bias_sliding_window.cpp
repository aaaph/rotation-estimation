#include <Eigen/Core>
#include <chrono>
#include <cstdint>
#include <gtest/gtest.h>
#include <stdexcept>

#include "rotation_estimation/gyro_bias_sliding_window.hpp"

namespace {

using namespace std::chrono_literals;

constexpr int64_t kOneSecondNs = 1'000'000'000LL;

TEST(GyroBiasSlidingWindowTest, ReturnsMeanOfSamplesInsideLastTenSeconds) {
  rotation_estimation::GyroBiasSlidingWindow window;

  window.add(0, Eigen::Vector3d{1.0, 2.0, 3.0});
  window.add(5 * kOneSecondNs, Eigen::Vector3d{2.0, 3.0, 4.0});
  window.add(10 * kOneSecondNs, Eigen::Vector3d{3.0, 4.0, 5.0});

  ASSERT_EQ(window.size(), 3U);
  ASSERT_TRUE(window.mean().has_value());
  EXPECT_TRUE(window.mean()->isApprox(Eigen::Vector3d{2.0, 3.0, 4.0}));

  window.add((10 * kOneSecondNs) + 1, Eigen::Vector3d{4.0, 5.0, 6.0});

  ASSERT_EQ(window.size(), 3U);
  ASSERT_TRUE(window.mean().has_value());
  EXPECT_TRUE(window.mean()->isApprox(Eigen::Vector3d{3.0, 4.0, 5.0}));
}

TEST(GyroBiasSlidingWindowTest, ClearsSamplesWhenTimestampMovesBackward) {
  rotation_estimation::GyroBiasSlidingWindow window{10s};
  window.add(2 * kOneSecondNs, Eigen::Vector3d::Ones());

  window.add(kOneSecondNs, 3.0 * Eigen::Vector3d::Ones());

  ASSERT_EQ(window.size(), 1U);
  ASSERT_TRUE(window.mean().has_value());
  EXPECT_TRUE(window.mean()->isApprox(3.0 * Eigen::Vector3d::Ones()));
}

TEST(GyroBiasSlidingWindowTest, EmptyWindowHasNoMean) {
  rotation_estimation::GyroBiasSlidingWindow window;

  EXPECT_FALSE(window.mean().has_value());

  window.add(0, Eigen::Vector3d::Ones());
  window.clear();

  EXPECT_EQ(window.size(), 0U);
  EXPECT_FALSE(window.mean().has_value());
}

TEST(GyroBiasSlidingWindowTest, RejectsNonPositiveDuration) {
  EXPECT_THROW(rotation_estimation::GyroBiasSlidingWindow{0ns}, std::invalid_argument);
}

}  // namespace
