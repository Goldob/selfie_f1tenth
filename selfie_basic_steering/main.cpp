#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <include/process.hpp>

Process process;
SteerData data_to_send;

void lidarCallback(LaserScan message);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_steering");
  ros::NodeHandle node;

  ros::Publisher steering_publisher = node.advertise();
  ros::Subscriber lidar_subscriber = node.subscribe("scan", 1, lidarCallback);

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
    steering_publisher.publish(data_to_send);

     // Next step
    ros::spinOnce();
  }

  return 0;
}

void lidarCallback(LaserScan message)
{
    // Copy data
    process.angle_min = -message->angle_max;

    for(uint32_t i = 0; i < message->ranges.size(); i++)
        process.raw_data = message->ranges[i];
}

