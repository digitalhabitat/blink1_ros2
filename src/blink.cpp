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
#include <sys/wait.h> // waitpid()

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class blink1InterfaceNode : public rclcpp::Node
{
public:
  blink1InterfaceNode()
  : Node("blink1_interface_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>
    ("string_topic", 10, std::bind(&blink1InterfaceNode::callback, this, _1));
  }

  void launchTinyServer()
  {
    std::string filepath = "/workspaces/bot2_ros2_workspace/src2/blink1-tool/blink1-tiny-server";
    std::string filename = "blink1-tiny-server";
    
    pid_t pid = fork();

    if (pid == -1)
    {
      std::perror("Error forking process");
      std::exit(EXIT_FAILURE);
    }
    else if (pid == 0)
    {
      // Child process
      execlp(filepath.c_str(), filename.c_str(), "--port", "8123", "-no-html", nullptr);
      // If execlp fails
      std::perror("Error executing child process");
      std::string errmsg = "Executing child process: " + filename + "failed. Shutting"; 
      std::exit(EXIT_FAILURE);
      RCLCPP_ERROR(get_logger(), "Operation failed. Shutting down.");
      rclcpp::shutdown();  // Terminate the node
    }
  }

private:
  void callback(const std_msgs::msg::String & cmd) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'",cmd.data.c_str());
    std::string uri = "curl localhost:8123/blink1/" + cmd.data;
    std::system(uri.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<blink1InterfaceNode>();
  node->launchTinyServer();
 
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
