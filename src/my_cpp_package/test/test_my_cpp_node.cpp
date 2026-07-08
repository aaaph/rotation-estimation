#include <gtest/gtest.h>

#include "my_cpp_package/my_cpp_node.hpp"

TEST(MyCppNodeTest, AddsIntegers) {
  EXPECT_EQ(my_cpp_package::add(2, 3), 5);
  EXPECT_EQ(my_cpp_package::add(-2, 3), 1);
}
