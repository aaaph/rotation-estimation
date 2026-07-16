#include "memory"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rotation_estimation_bringup/manager_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rotation_estimation_bringup::ManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
