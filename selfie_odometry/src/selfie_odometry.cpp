#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"

class odom{
public:
   float velocity;
   float quaternion_x; 
   float quaternion_y;
   float quaternion_z; 
   float quaternion_w;
   float ang_vel_x; 
   float ang_vel_y;
   float ang_vel_z;
   float lin_acc_x; 
   float lin_acc_y;
   float lin_acc_z;

}odom_data;

void chatterCallback_imu(const sensor_msgs::Imu::ConstPtr& msg){
  //ROS_INFO("Imu_orientation: [%f]", msg->orientation.z);
  odom_data.quaternion_x = msg->orientation.x;
}

void chatterCallback_velo(const std_msgs::Float32::ConstPtr& msg){
  //ROS_INFO("Velo: [%f]", msg->data);
  odom_data.velocity = msg->data;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "selfie_odometry");

  ros::NodeHandle n;

  ros::Subscriber imu_sub = n.subscribe("data_raw", 1000, chatterCallback_imu);
  ros::Subscriber velo_sub = n.subscribe("float32", 1000, chatterCallback_velo);
  while(true){
    ROS_INFO("Velo: [%f]", odom_data.velocity);
    ros::spinOnce();
    loop_rate.sleep();
  }
  

  return 0;
}
// %EndTag(FULLTEXT)%

