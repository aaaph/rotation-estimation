#include <gtest/gtest.h>
#include <string_view>

#include "rotation_estimation/ekf_node.hpp"

TEST(EkfNodeTest, UsesDefaultNodeName) {
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultNodeName}, "ekf_node");
}

TEST(EkfNodeTest, UsesDefaultTopics) {
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultImuTopic}, "/sensehat/imu_raw");
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultTfTopic}, "/tf");
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultGyroBiasTopic},
            "/rotation_estimation/gyro_bias");
}

TEST(EkfNodeTest, UsesDefaultWorldFrame) {
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultWorldFrameId}, "world");
}

TEST(EkfNodeTest, UsesAccelAsDefaultMode) {
  EXPECT_EQ(rotation_estimation::EkfNode::kDefaultMode, rotation_estimation::EkfNode::Mode::kAccel);
}

TEST(EkfNodeTest, ConvertsModeToString) {
  EXPECT_EQ(rotation_estimation::EkfNode::toString(rotation_estimation::EkfNode::Mode::kAccel),
            "accel");
  EXPECT_EQ(rotation_estimation::EkfNode::toString(rotation_estimation::EkfNode::Mode::kStationary),
            "stationary");
}

TEST(EkfNodeTest, ParsesModeFromString) {
  EXPECT_EQ(rotation_estimation::EkfNode::modeFromString("accel"),
            rotation_estimation::EkfNode::Mode::kAccel);
  EXPECT_EQ(rotation_estimation::EkfNode::modeFromString("accel_update"),
            rotation_estimation::EkfNode::Mode::kAccel);
  EXPECT_EQ(rotation_estimation::EkfNode::modeFromString("stationary"),
            rotation_estimation::EkfNode::Mode::kStationary);
  EXPECT_EQ(rotation_estimation::EkfNode::modeFromString("unknown"), std::nullopt);
}
