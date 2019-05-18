#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "ringbuf.h"
#include <frtn01/TimestampMsg.h>

/* Simple publisher that is meant to be paired with the corresponding "plant" node that publishes control values to the "control_sig" topic and receives output values from the "plant_sig" topic
*/

#define PUB_PERIOD 500		// ms
#define N_INPUT 4		// number of values to be used for inputs ringbuffer
#define N_ERR 3			// number of values to be used for error ringbuffer

// Define global variables
ros::Publisher control_pub;
ros::Subscriber control_sub;
int started = 0;
int received = 0;
uint64_t ns_period = PUB_PERIOD * 1000000;
double timer_offset = 0;
double earliest_tstamp = 0;
double timing_lag = 0;
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

void controller_callback(const frtn01::TimestampMsg::ConstPtr& msg)
{
  if(!started) 
  {
    timer_offset = ros::Time::now().toSec();
    ROS_INFO("Started receiving data at time: %lf", timer_offset);
    started++;
    received++;
  }
  else if(msg->last_msg)
  {
    ROS_INFO("Test finished.");
    ros::shutdown();
  }
  else
  {
    earliest_tstamp = msg->time;
    //RCLCPP_INFO(this->get_logger(), "Pre-lag: %lu", now().nanoseconds() - earliest_tstamp);
    //timing_lag = now().nanoseconds() - timer_offset - (uint64_t)(ns_period * received);
    //RCLCPP_INFO(this->get_logger(), "Periodic Lag: %li", timing_lag);
    received++;
      
    // Calculate controller signal
    frtn01::TimestampMsg control_signal;
    control_signal.signal = compute_controller(msg->signal);
    control_pub.publish(control_signal);
    //ROS_INFO("Total lag: %lf", ros::Time::now().toSec() - earliest_tstamp);
    //ROS_INFO("Sent control signal: %f", \
      control_signal.signal);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mip_controller");
  ros::NodeHandle n;
  control_pub = n.advertise<frtn01::TimestampMsg>("control_sig", 500);
  control_sub = n.subscribe("plant_sig", 500, controller_callback);
  
  /*
  ros::Rate pub_rate(PUBRATE);
  int count = 0;
  while(ros::ok())
  {
    std_msgs::Float32 msg;
    msg.data = (float)count;
    ROS_INFO("Sending: %f", msg.data);
    control_pub.publish(msg);
    ros::spinOnce();
    pub_rate.sleep();
    ++count;
  }
  */
  
  // Spin when not doing anything
  ros::spin();

  ros::shutdown();
  
  return 0;
}
