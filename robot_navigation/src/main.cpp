#include "../include/robot_navigation/navigation_manager.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_navigation::NavigationManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}