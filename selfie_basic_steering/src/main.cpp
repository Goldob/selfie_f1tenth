#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "include/process.hpp"
#include "std_msgs/Float64.h"
Process process;
SteerData data_to_send;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& message);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_steering");
  ros::NodeHandle node;
  ros::Publisher steering_publisher = node.advertise<std_msgs::Float64>("steering_state", 100);
  ros::Subscriber lidar_subscriber = node.subscribe("scan", 1, lidarCallback);
  //msg sent to PID:
  std_msgs::Float64 state_msg;
  
  while (ros::ok())
  {
    // Process data
    process.polar_to_cartesian();
    process.simplify_data();
    process.split_poins_equally();
    process.search_points();
    process.calc_mid();
    process.calc_offsets();
    process.calc_slopes();
    
    // Publish data
    process.pack_data(data_to_send);
    //TODO: here add the variable that you want to give to PID:
    //need to be Float type, one value, now doesnt work
    //steering_publisher.publish(data_to_send);
    state_msg.data = 10.f;
    steering_publisher.publish(state_msg);

     // Next step
    ros::spinOnce();
  }

  return 0;
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& message)
{
    // Copy data
    process.angle_min = -message->angle_max;
    process.raw_data.clear();
    for(uint32_t i = 0; i<message->ranges.size(); i++){
        process.raw_data.push_back (message->ranges[i]);
    }
}

