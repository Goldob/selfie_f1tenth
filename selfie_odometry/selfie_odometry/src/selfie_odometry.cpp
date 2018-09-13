#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

float speed=0.0;

double roll,pitch,yaw=0.0;

double x = 0.0;
double y = 0.0;
//double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

void speedCallback(const std_msgs::Float32 &msg){
  speed=msg.data;
}

void imuCallback(const sensor_msgs::Imu &msg){
  vth=msg.angular_velocity.z;

  tf::Quaternion q(
  msg.orientation.x,
  msg.orientation.y,
  msg.orientation.z,
  msg.orientation.w);

  tf::Matrix3x3 m(q);
  m.getRPY(roll,pitch,yaw);

  vx=speed*cos(yaw);
  vy=speed*sin(yaw);
}


int main(int argc, char** argv){


  ros::init(argc, argv, "selfie_odometry");
  ros::NodeHandle n;
  
  ros::Subscriber sub_speed=n.subscribe("speed", 50, speedCallback);
  ros::Subscriber sub_imu=n.subscribe("imu", 50, imuCallback);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = vx * dt;
    double delta_y = vy * dt;
    //double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    //th += delta_th;

    //quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

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
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
  }
}
