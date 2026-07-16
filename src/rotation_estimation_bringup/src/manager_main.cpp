#include <memory>

#include "rclcpp/utilities.hpp"
#include "rotation_estimation_bringup/manager_node.hpp"

int main(int argc, char** argv) {  // NOLINT(bugprone-exception-escape)
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rotation_estimation_bringup::ManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 2};
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
