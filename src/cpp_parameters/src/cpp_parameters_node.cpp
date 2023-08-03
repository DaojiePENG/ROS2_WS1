#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "this is my parameter...";

    this->declare_parameter("my_parameter", "world", param_desc);
    this->declare_parameter("my_parameter1", "PDJ", param_desc);


    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback()
  {
    std::string my_param = this->get_parameter("my_parameter").as_string();
    std::string my_param1 = this->get_parameter("my_parameter1").as_string();


    RCLCPP_INFO(this->get_logger(), "Hello %s! Hi %s!", my_param.c_str(), my_param1.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world"),
    rclcpp::Parameter("my_parameter1", "PDJ")}; // 这样写的用意应该是可以同时设置一组参数。是的，这样写OK。不过好像用终端一次只能改一个变量，用launch一次可以改多个。

    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}