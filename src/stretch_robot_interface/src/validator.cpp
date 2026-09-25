#include "stretch_robot_interface/interface_node.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stretch_robot_interface::InterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
