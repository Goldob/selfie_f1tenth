<<<<<<< HEAD
### More detailed info about node selfie_stm32_bridge:
This node is responsible for communication with STM32 through USB. STM32 sends orientation, linear acceleration and angular velocity from IMU, velocity of robot from encoders and also sends control flags.
This variables are sent to sensor_msg/Imu.h and std_msg/Float32.h.
STM32 gets from this node variables from ackermann_msgs/AckermannDriveStamped.

Uses: imu_publisher, velo_publisher, ackerman_subscriber

Commands:
sudo chmod 777 ./dev/ttyACM0
source ./devel/setup.bash
rosrun selfie_stm32_bridge selfie_stm32_bridge
=======
# STM32 Bridge

`selfie_stm32_bridge` package provides a node with the same name responsible for communication with on-board STM32 microcontroller that handles IMU, encoders and vehicle control. The node communicates with ROS using standarized message types, as described below.

## `selfie_stm32_bridge`

### Subscribed topics

`drive` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
Steering commands to be applied.

### Published topics

`imu` ([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))
Data stream from IMU.

`speed` ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))
Linear velocity magnitude at the center of rear axle, as calculated from encoder data (in m/s).
>>>>>>> master
