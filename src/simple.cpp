#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cstdlib>  // For system function

int main(int argc, char * argv []) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_node");

  const char* command_on = "blink1-tool --on";
  const char* command_off = "blink1-tool --off";

  int result = system(command_on);
  if (result != 0) {
    std::cerr << "Command failed to execute." << std::endl;
  }

  rclcpp::spin(node);

  rclcpp::shutdown();

  result = system(command_off);
  if (result != 0) {
    std::cerr << "Command failed to execute." << std::endl;
  }

  return 0;
}