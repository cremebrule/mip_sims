#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/* This is a simple publisher that is used to publish numerical (float) values */

class numPublisher : public rclcpp::Node
{
public:
  numPublisher()
  : Node("number_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("control_sig");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&numPublisher::timer_callback, this));
  }
  
private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32();
    message.data = (float)(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<numPublisher>());
  rclcpp::shutdown();
  return 0;
}
