#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

float speed = 0;

double roll = 0;
double pitch = 0;
double yaw = 0;

float x = 0;
float y = 0;

float vx = 0;
float vy = 0;
float vyaw = 0;

float currentDistance = 0;
float lastDistance = 0;
float deltaDistance = 0;
float dt = 0;
float dx = 0;
float dy = 0;

ros::Time current_time, last_time;

ros::Publisher odom_pub;
geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

void distanceCallback(const std_msgs::Float32 &msg){
  current_time = ros::Time::now();
  lastDistance = currentDistance;
  currentDistance = msg.data / 1000;
  deltaDistance = currentDistance - lastDistance;
  dt = (current_time - last_time).toSec();
  speed = deltaDistance / dt;
  vx = speed * cos(yaw);
  vy = speed * sin(yaw);
  
  dx = deltaDistance * cos(yaw);
  dy = deltaDistance * sin(yaw);

  x += dx;
  y += dy;

  //publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "base_frame";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "rear_axis_frame";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vyaw;
  
  //publish the message
  odom_pub.publish(odom);

  last_time = current_time;
}

void imuCallback(const sensor_msgs::Imu &msg){

  vyaw = msg.angular_velocity.z;

  tf::Quaternion q(
  msg.orientation.x,
  msg.orientation.y,
  msg.orientation.z,
  msg.orientation.w);

  tf::Matrix3x3 m(q);
  m.getRPY(yaw,pitch,roll);
  //quaternion created from yaw
  odom_quat = tf::createQuaternionMsgFromYaw(yaw);
}

int main(int argc, char** argv){

  ros::init(argc, argv, "selfie_odometry");
  
  ros::NodeHandle n;
  
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub_distance=n.subscribe("distance", 50, distanceCallback);
  ros::Subscriber sub_imu=n.subscribe("imu", 50, imuCallback);

  tf::TransformBroadcaster odom_broadcaster;

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate loop_rate(10);

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages

    //publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "base_frame";
    odom_trans.child_frame_id = "rear_axis_frame";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    loop_rate.sleep();
  }
}
