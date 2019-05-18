#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "ringbuf.h"
#include <frtn01/TimestampMsg.h>
#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>

/* A simple node that receives values from the controller.cpp node and returns values to it */

#define FREQ 0.002		// frequency (seconds)
#define TIMEOUT FREQ*5000	// test timeout (seconds)
#define TOKEN 100		// parse token for reading in matlab

#define N_IN 3
#define N_ANG 4
#define ANG_FALL 1.3		// radians
//#define FILEPATH_ "~/catkin_ws/src/frtn01/testing.txt"
#define FILEPATH_ "test.txt"

// Declare global variables
ros::Publisher plant_pub;
ros::Subscriber plant_sub;
std::ofstream myfile;
int started = 0;
int in_idx = 0;
int ang_idx = 0;
int count_ = 0;
int n_since_last_input = 0;
float freq;
float timeout_;
float noise = 0;
double angles[N_ANG] = {0};
double inputs[N_IN] = {0};
double time_offset;
double time_sent;
double min_lag = 1.0;
double max_lag = 0.0;
double avg_lag = 0.0;
double current_lag = 0.0;
int n = 0;
int success = 0;
float input;
float b[3] = {2.916, -2.832, 0.9155};
float a[3] = {0.001715, -0.00004973, -0.001665};
Ringbuf in_ring = Ringbuf(N_IN);
Ringbuf ang_ring = Ringbuf(N_ANG);

void exit_protocol(void)
{
  // Protocol for shutting down node
  
  // Close down file and output infomatics before shutting down
  myfile << TOKEN << "," << TOKEN << \
      "," << success << "\n";
  myfile << min_lag << "," << max_lag << \
      "," << avg_lag << "\n";
  myfile << freq << "," << noise << \
      "," << 0 << "\n";
  
  myfile.close();
  if(success > 0) ROS_INFO("Test concluded; exiting");
  else if(success < 0) ROS_INFO("MIP fell; exiting");
  else ROS_INFO("Test interrupted; exiting");
  ROS_INFO("Test Statistics:");
  ROS_INFO("Min latency: %lf", min_lag);
  ROS_INFO("Max latency: %lf", max_lag);
  ROS_INFO("Avg latency: %lf", avg_lag);
  frtn01::TimestampMsg final_msg;
  final_msg.last_msg = true;
  plant_pub.publish(final_msg);
  ros::shutdown();
}

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

void timer_callback(const ros::TimerEvent&)
{
  if(!started)
  {
    if(count_ < 9) count_++;
    else
    {
      frtn01::TimestampMsg message;
      message.signal = 0.01;
      //ROS_INFO("Publishing initializer (%f) value", message.signal);
      plant_pub.publish(message);
      count_ = 0;
    }
  }
  else 
  {
    if(count_ < 9)			// 10x as fast compared to controller (1000Hz)
    {
      compute_plant(input);
      //RCLCPP_INFO(this->get_logger(), "Count @ %i", count_);
      count_++;
    }
    else
    {
      frtn01::TimestampMsg message;
      float r;
      if(noise)
      {
        r = -noise/2 + static_cast <float> (rand()) / \
	  static_cast <float> (RAND_MAX/noise);
      }
      else r = 0.0;
      float noise_free_sig = compute_plant(input);
      message.signal = noise_free_sig + r;
      time_sent = ros::Time::now().toSec();
      
      // Check to see if total time elapsed is more than 5 seconds
      if(time_sent - time_offset > timeout_)
      {
        success++;
        exit_protocol();
      }
      
      message.time = time_sent;
      plant_pub.publish(message);
      //ROS_INFO("Published: '%f'", message.signal);
      myfile << message.time - time_offset << "," << noise_free_sig << \
      		"," << 9 - n_since_last_input << "\n";
      count_ = 0;
    }
    n_since_last_input++;
  }
}

void plant_callback(const frtn01::TimestampMsg::ConstPtr& msg)
{
  if(!started) started++;
  else
  {
    n++;
    input = msg->signal;
    current_lag = ros::Time::now().toSec() - time_sent;
    if(current_lag < min_lag) min_lag = current_lag;
    else if(current_lag > max_lag) max_lag = current_lag;
    avg_lag = ( (n - 1) * avg_lag + current_lag ) / n;
    //ROS_INFO("Total round lag: '%lf'", ros::Time::now().toSec() - time_sent); 
    n_since_last_input = 0;
  }
}

int main(int argc, char **argv)
{
  // Initialize node characteristics
  ros::init(argc, argv, "mip_plant");
  
  myfile.open(FILEPATH_);
  myfile << "Timestamp,Angle,SampleLag\n";
  ros::NodeHandle n;

  // Read Command-Line Arguments
  if(n.getParam("/frequency", freq) && n.getParam("/noise", noise)) \
    ROS_INFO("Param return success: %f AND %f", freq, noise);
  else ROS_INFO("Failed Param return.");
  timeout_ = freq * 5000;

  time_offset = ros::Time::now().toSec();
  plant_sub = n.subscribe("control_sig", 500, plant_callback);
  plant_pub = n.advertise<frtn01::TimestampMsg>("plant_sig", 500);
  ros::Timer plant_timer = n.createTimer(ros::Duration(freq), timer_callback);
  
  // Spin when not doing anything
  ros::spin();
  
  exit_protocol();
  
  return 0;
}
