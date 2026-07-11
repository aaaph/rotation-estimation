#include <gtest/gtest.h>
#include <string_view>

#include "rotation_estimation/ekf_node.hpp"

TEST(EkfNodeTest, UsesDefaultNodeName) {
  EXPECT_EQ(std::string_view{rotation_estimation::EkfNode::kDefaultNodeName}, "ekf_node");
}
