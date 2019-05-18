/*
A simple subscriber node that is meant to be paired with the dummy_publisher.cpp node. This
node reads values published from the dummy_publisher and prints out the timestamp upon receiving, using the first value received from dummy_publisher as the "zero" timestamp. */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#define PUB_PERIOD 500		// ms

using std::placeholders::_1;

class numSubscriber : public rclcpp::Node
{
public:
  numSubscriber()
  : Node("number_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "control_sig", std::bind(&numSubscriber::topic_callback, this, _1));
  }
  
private:
  rclcpp::Time clock;
  int started = 0;
  int received = 0;
  uint64_t ns_period = PUB_PERIOD * 1000000;
  uint64_t timer_offset = 0;
  int64_t timing_lag = 0;
  
  void topic_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    
    //RCLCPP_INFO(this->get_logger(), "I heard: '%f', time: %lu", msg->data, now().nanoseconds());
    if(!started) 
    {
      timer_offset = now().nanoseconds();
      RCLCPP_INFO(this->get_logger(), "Started receiving data at time: %lu", timer_offset);
      started++;
      received++;
    }
    else
    {
      timing_lag = now().nanoseconds() - timer_offset - (uint64_t)(ns_period * received);
      RCLCPP_INFO(this->get_logger(), "Periodic Lag: %li", timing_lag);
      received++;
    }
  }
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<numSubscriber>());
  rclcpp::shutdown();
  return 0;
}
