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

class blink1_node : public rclcpp::Node
{
public:
  blink1_node()
  : Node("blink1_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>
    ("string_topic", 10, std::bind(&blink1_node::callback, this, _1));
    on_shutdown(std::bind(&blink1_node::ShutdownCallback, this));
    curl_wrap("fadeToRGB?rgb=000000&ledn=0");
    curl_wrap("pattern/play?pattern=0,ffbf00,0.4,2,000000,0.4,2");
  }

private:
  void curl_wrap(const std::string & str) const
  {
    // TODO: use C++ Requests: Curl for People (cpr) instead of system call
    std::string uri = "curl \"localhost:8123/blink1/" + str +"\"";
    RCLCPP_INFO(this->get_logger(), uri.c_str());
    std::system(uri.c_str());
  }

  void ShutdownCallback() const
  {
     curl_wrap("blink?rgb=FF00FF");
  }

  void callback(const std_msgs::msg::String & cmd) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'",cmd.data.c_str());
    curl_wrap(cmd.data);
    
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  bool serverOn_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<blink1_node>();
 
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
