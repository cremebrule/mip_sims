#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "test_messages/msg/timestamp_msg.hpp"
#include "ringbuf.h"

#define N_IN 3
#define N_ANG 4
#define ANG_FALL 1.3		// radians
#define FILEPATH_ "test2.txt"
#define TOKEN 100		// parser token for reading data file in matlab

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This is a simple plant node that takes a controller input and returns a (simulated) plant output based on a mobile inverted pendulum (MIP) */

// Declare function prototypes
void start_protocol(void);
void exit_protocol(void);

// Define global variables
std::ofstream myfile;
int freq = 5;
int timeout_sec = 0;
float noise = 0;
uint64_t min_lag = 10000000;
uint64_t max_lag = 0;
uint64_t time_offset = 0;
uint64_t timeout;
float avg_lag = 0;
int success = 0;
bool shutdown_run = 0;
//rclcpp::Time clock;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Subscription<test_messages::msg::TimestampMsg>::SharedPtr subscription_;
rclcpp::Publisher<test_messages::msg::TimestampMsg>::SharedPtr publisher_;

class mipPlant : public rclcpp::Node
{
public:
  mipPlant()
  : Node("plant_node"), count_(0)
  {
    // Get command-line parameters
    get_parameter("frequency", freq);
    get_parameter("noise", noise);
    get_parameter("timeout", timeout_sec);
    timeout = (uint64_t)freq * 1000000000 * timeout_sec;
    std::chrono::duration<int64_t, std::milli> ms_freq(freq);
    
    // Initialize RMW
    auto msg_strategy = std::make_shared<rclcpp::strategies::message_pool_memory_strategy::\
      MessagePoolMemoryStrategy<test_messages::msg::TimestampMsg, 1>>();
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile.depth = 500;
    
    // Initialize nodes
    publisher_ = this->create_publisher<test_messages::msg::TimestampMsg>("plant_sig",\
      qos_profile);
    subscription_ = this->create_subscription<test_messages::msg::TimestampMsg>(
      "control_sig", std::bind(&mipPlant::plant_callback, this, _1),\
      qos_profile, nullptr, false, msg_strategy);
    //time_pub_ = this->create_publisher<std_msgs::msg::UInt64>("timestamp");
    timer_ = this->create_wall_timer(
      ms_freq, std::bind(&mipPlant::timer_callback, this));
    srand(static_cast <unsigned> (time(0)));
    time_offset = now().nanoseconds();
  }
  
private:
  int started = 0;
  int in_idx = 0;
  int ang_idx = 0;
  int n_since_last_input = 0;
  int n = 0;
  int count_ = 0;
  uint64_t time_sent = 0;
  uint64_t current_lag = 0;
  double angles[N_ANG] = {0};
  double inputs[N_IN] = {0};
  float input;
  float b[3] = {2.916, -2.832, 0.9155};
  float a[3] = {0.001715, -0.00004973, -0.001665};
  Ringbuf in_ring = Ringbuf(N_IN);
  Ringbuf ang_ring = Ringbuf(N_ANG);
  
  float compute_plant(float input)
  {
    // Add input
    inputs[in_idx] = input;
    
    // Compute the plant output angle from the given input
    angles[ang_idx] = \
      + b[0] * angles[(ang_idx + N_ANG - 1) % N_ANG] \
      + b[1] * angles[(ang_idx + N_ANG - 2) % N_ANG] \
      + b[2] * angles[(ang_idx + N_ANG - 3) % N_ANG] \
      + a[0] * inputs[in_idx] \
      + a[1] * inputs[(in_idx + N_IN - 1) % N_IN] \
      + a[2] * inputs[(in_idx + N_IN - 2) % N_IN];
      
    // Save output
    float angle = (float)angles[ang_idx];
    if(angle > ANG_FALL || angle <-ANG_FALL)
    {
      started = 0;
      success--;
      exit_protocol();
    }
    
    // Update ring buffers
    in_idx = in_ring.update_idx();
    ang_idx = ang_ring.update_idx();
    
    // Return angle value
    return angle;
  }
  
  void plant_callback(const test_messages::msg::TimestampMsg::SharedPtr msg)
  {
    if(!started)
    {
      //count_ = 0;
      started++;
    }
    else
    {
      n++;
      input = msg->signal;
      current_lag = now().nanoseconds() - time_sent;
      if(current_lag < min_lag) min_lag = current_lag;
      else if(current_lag > max_lag) max_lag = current_lag;
      avg_lag = static_cast<float>((n - 1) * avg_lag + current_lag ) / static_cast<float>(n);
      //RCLCPP_INFO(this->get_logger(), "Input received: %f", input);
      n_since_last_input = 0;
    }
  }
  
  void timer_callback()
  {
    if(!started)
    {
      if(count_ < 9) count_++;
      else
      {
        auto message = test_messages::msg::TimestampMsg();
        message.signal = 0.01;
        time_sent = now().nanoseconds();
        //RCLCPP_INFO(this->get_logger(), "Publishing initializer (%f) value", message.signal);
        publisher_->publish(message);
        count_ = 0;
      }
    }
    else 
    {
      if(count_ < 9)			// 10x as fast compared to controller (1000Hz)
      {
        //float test = compute_plant(input);
	compute_plant(input);        
	//RCLCPP_INFO(this->get_logger(), "Published @ %i: %f", count_, test);
        count_++;
      }
      else
      {
        auto message = test_messages::msg::TimestampMsg();
        float r;
        if(noise)
        {
          r = -noise/2 + static_cast <float> (rand()) / \
	    static_cast <float> (RAND_MAX/noise);
        }
        else r = 0.0;
        float noise_free_sig = compute_plant(input);
        message.signal = noise_free_sig + r;
        time_sent = now().nanoseconds();
        
        // Check to see if total time elapsed is more than 5 seconds
        if(time_sent - time_offset > timeout)
        {
          RCLCPP_INFO(this->get_logger(), "Timeout (ns): %lu", timeout);
          RCLCPP_INFO(this->get_logger(), "Time (ns): %lu", time_sent - time_offset);
          success++;
          exit_protocol();
        }
        
        message.ntime = time_sent;
        publisher_->publish(message);
        //RCLCPP_INFO(this->get_logger(), "Published: '%f'", message.signal);
        myfile << time_sent - time_offset << "," << noise_free_sig << \
      		"," << 9 - n_since_last_input << "\n";        
        count_ = 0;
      }
      n_since_last_input++;
    }
  }
};

void start_protocol(void)
{
  myfile.open(FILEPATH_);
  myfile << "Timestamp,Angle,SampleLag\n";
  
  // Get parameters from parameter server
  //auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  //freq = parameters_client->get_parameter("frequency");
  //noise = parameters_client->get_parameter("noise");
}

void exit_protocol(void)
{
  // Protocol for shutting down node (checks to make sure it's only run once)
  if(shutdown_run) return;
  else shutdown_run = true;
  
  // Close down file and output infomatics before shutting down
  myfile << TOKEN << "," << TOKEN << \
      "," << success << "\n";
  myfile << min_lag << "," << max_lag << \
      "," << avg_lag << "\n";
  myfile << freq << "," << noise << \
      "," << 0 << "\n";
  
  myfile.close();
  if(success > 0) std::cout << "Test concluded; exiting" << std::endl;
  else if(success < 0) std::cout << "MIP fell; exiting" << std::endl;
  else std::cout << "Test interrupted; exiting" << std::endl;
  std::cout << "Test Statistics:" << std::endl;
  std::cout << "Min latency: " << (float)min_lag/1e6 << " ms" << std::endl;
  std::cout << "Max latency: " << (float)max_lag/1e6 << " ms" << std::endl;
  std::cout << "Avg latency: " << (float)avg_lag/1e6 << " ms" << std::endl;
  std::cout << "Frequency used: " << freq << " ms" << std::endl;
  std::cout << "Noise is set to: " << noise << " rad" << std::endl;
  auto final_msg = test_messages::msg::TimestampMsg();
  final_msg.last_msg = true;
  publisher_->publish(final_msg);
  rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
  start_protocol();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mipPlant>());
  exit_protocol();
  return 0;
}
