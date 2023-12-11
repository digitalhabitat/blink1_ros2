// Copyright 2024 Michael Miller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdlib>  // For system function

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_node");

  const char * command_on = "blink1-tool --on";
  const char * command_off = "blink1-tool --off";

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
