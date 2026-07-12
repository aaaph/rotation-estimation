#include <memory>

#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
#include "sense_hat_bridge/sense_hat_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sense_hat_bridge::SenseHatNode>());
  rclcpp::shutdown();

  return 0;
}
