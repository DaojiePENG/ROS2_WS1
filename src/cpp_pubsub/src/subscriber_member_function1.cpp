// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using std::placeholders::_1;

class MinimalSubscriber1 : public rclcpp::Node
{
public:
  MinimalSubscriber1()
  : Node("minimal_subscriber")
  {
    auto newCallback = std::bind(&MinimalSubscriber1::topic_callback, this, _1); // 将MinimalSubscriber1与其成员函数绑定，传入参数就是msg了
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(
      "topic", 10, newCallback); // 每一次发布就被激发一次订阅更新。
  }

private:
  void topic_callback(const tutorial_interfaces::msg::Num & msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber1>());
  rclcpp::shutdown();
  return 0;
}
