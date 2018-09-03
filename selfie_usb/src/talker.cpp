#include "ros/ros.h"
#include "std_msgs/String.h"
#include "usb.hpp"
#include <sstream>

USB_STM Usb;

/**
  * This tutorial demonstrates simple sending of messages over the ROS system.
  */
int main(int argc, char **argv){
/**
 * The ros::init() function needs to see argc and argv so that it can perform
 * any ROS arguments and name remapping that were provided at the command line.
 * For programmatic remappings you can use a different version of init() which takes
 * remappings directly, but for most command-line programs, passing argc and argv is
 * the easiest way to do it.  The third argument to init() is the name of the node.
 *
 * You must call one of the versions of ros::init() before using any other
 * part of the ROS system.
 */
    ros::init(argc, argv, "talker");
  
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
   ros::NodeHandle n;
 
   /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
 
   ros::Rate loop_rate(10);
  
   /**
    * A count of how many messages we have sent. This is used to create
    * a unique string for each message.
   */
   int count = 0;


   //usb communication
   data_container to_send;
   float imu_angle;
   uint32_t velocity = 1;
   uint32_t angle; 
   uint32_t angle_to_send = 0;
   std::vector<uint32_t>usb_from_vision(5);
   for(int i=0;i<5;i++){
            usb_from_vision[i]=i;
   }


   //usb receive data
   Usb.init();//init
  
   while (ros::ok()){
      /**
      * This is a message object. You stuff it with data, and then publish it.
      */
      
      //Usb.data_pack(velocity,angle_to_send,usb_from_vision,&to_send);
      //Usb.send_buf(to_send);
      Usb.read_buffer(4,imu_angle);
      //angle_to_send = Usb.test(angle_to_send);

      std_msgs::String msg;
  
      std::stringstream ss;
      //ss << "send"<<angle_to_send;
      ss << "receive: "<<Usb.imu_angle_from_stm;
      msg.data = ss.str();
  
      ROS_INFO("%s", msg.data.c_str());
  
      /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor above.
      */
      chatter_pub.publish(msg);
  
      ros::spinOnce();
  
      loop_rate.sleep();
      ++count;
    }
    return 0;
}
  

