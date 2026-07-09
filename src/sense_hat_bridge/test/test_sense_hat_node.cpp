#include <gtest/gtest.h>
#include <string_view>

#include "sense_hat_bridge/sense_hat_node.hpp"

TEST(SenseHatNodeTest, UsesDefaultNodeName) {
  EXPECT_EQ(std::string_view{sense_hat_bridge::SenseHatNode::kDefaultNodeName},
            "sensehat_udp_imu_node");
}
