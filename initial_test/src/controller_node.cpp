/*
A simple controller node that is meant to be paired with the plant.cpp node. This
node reads plant output values published from the plant node and computes the corresponding control signal, while also sending the lag to the plant as well. */

#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "test_messages/msg/timestamp_msg.hpp"
#include "ringbuf.h"

#define PUB_PERIOD 500		// ms
#define N_INPUT 4		// number of values to be used for inputs ringbuffer
#define N_ERR 3			// number of values to be used for error ringbuffer

using std::placeholders::_1;

class mipController : public rclcpp::Node
{
public:
  mipController()
  : Node("controller_node")
  {

    // Initialize RMW
    auto msg_strategy = std::make_shared<rclcpp::strategies::message_pool_memory_strategy::\
      MessagePoolMemoryStrategy<test_messages::msg::TimestampMsg, 1>>();
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile.depth = 500;
    
    subscription_ = this->create_subscription<test_messages::msg::TimestampMsg>(
    "plant_sig", std::bind(&mipController::controller_callback, this, _1),\
      qos_profile, nullptr, false, msg_strategy);
    publisher_ = this->create_publisher<test_messages::msg::TimestampMsg>("control_sig",\
      qos_profile);
    //time_sub_ = this->create_subscription<std_msgs::msg::UInt64>(
    //"timestamp", std::bind(&mipController::lag_callback, this, _1));
  }
  
private:
  rclcpp::Time clock;
  int started = 0;
  int received = 0;
  uint64_t ns_period = PUB_PERIOD * 1000000;
  uint64_t timer_offset = 0;
  uint64_t earliest_tstamp = 0;
  int64_t timing_lag = 0;
  int in_idx = 0;
  int err_idx = 0;
  float inputs[N_INPUT] = {0};
  float errors[N_ERR] = {0};
  float setpoint = 0.0;
  Ringbuf in_ring = Ringbuf(N_INPUT);
  Ringbuf err_ring = Ringbuf(N_ERR);
  
  float clamp(float max, float min, float val)
  {
    if(val > max) return max;
    else if(val < min) return min;
    else return val;
  }
  
  float compute_controller(float y)
  {
    // Compute the error
    errors[err_idx] = setpoint + y;
    
    // Compute the controller signal from plant output
    inputs[in_idx] = \
      1.303 * inputs[(in_idx + N_INPUT - 1) % N_INPUT] \
      - 0.3029 * inputs[(in_idx + N_INPUT - 2) % N_INPUT] \
      + 0.00001404 * inputs[(in_idx + N_INPUT - 3) % N_INPUT] \
      - 4.151 * errors[err_idx] \
      + 6.159 * errors[(err_idx + N_ERR - 1) % N_ERR] \
      - 2.108 * errors[(err_idx + N_ERR - 2) % N_ERR];
      
    // Saturate the output
    float out = clamp(5.0, -5.0, inputs[in_idx]);
    
    // Update indices for ringbuffers
    in_idx = in_ring.update_idx();
    err_idx = err_ring.update_idx();
    
    // Return controller signal value
    return out;
  }
  
  /*
  void lag_callback(const std_msgs::msg::UInt64::SharedPtr time)
  {
    earliest_tstamp = time->data;
    timing_lag = (int64_t)(now().nanoseconds() - time->data);
    RCLCPP_INFO(this->get_logger(), "Receiving Lag: %lu", timing_lag);
  }
  */
  
  void controller_callback(const test_messages::msg::TimestampMsg::SharedPtr msg)
  {
    if(!started) 
    {
      timer_offset = now().nanoseconds();
      RCLCPP_INFO(this->get_logger(), "Started receiving data at time: %lu", timer_offset);
      started++;
      received++;
    }
    else if(msg->last_msg)
    {
      RCLCPP_INFO(this->get_logger(), "Test finished.");
      rclcpp::shutdown();
    }
    else
    {
      earliest_tstamp = msg->ntime;
      //RCLCPP_INFO(this->get_logger(), "Pre-lag: %lu", now().nanoseconds() - earliest_tstamp);
      //timing_lag = now().nanoseconds() - timer_offset - (uint64_t)(ns_period * received);
      //RCLCPP_INFO(this->get_logger(), "Periodic Lag: %li", timing_lag);
      received++;
      
      // Calculate controller signal
      auto control_signal = test_messages::msg::TimestampMsg();
      control_signal.signal = compute_controller(msg->signal);
      publisher_->publish(control_signal);
      //RCLCPP_INFO(this->get_logger(), "Total lag: %lu", now().nanoseconds() - earliest_tstamp);
      //RCLCPP_INFO(this->get_logger(), "Sent control signal: %f", control_signal.signal);
    }
  }
  rclcpp::Subscription<test_messages::msg::TimestampMsg>::SharedPtr subscription_;
  //rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr time_sub_;
  rclcpp::Publisher<test_messages::msg::TimestampMsg>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mipController>());
  rclcpp::shutdown();
  return 0;
}
