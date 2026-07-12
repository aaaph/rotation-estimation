#include <memory>

#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
#include "rotation_estimation/ekf_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rotation_estimation::EkfNode>());
  rclcpp::shutdown();

  return 0;
}
