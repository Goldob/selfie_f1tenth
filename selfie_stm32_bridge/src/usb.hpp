#pragma once
#ifndef USB_HPP
#define USB_HPP

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <string>
#include <vector>
#include <stdint.h>
#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

struct data_container
{
  const uint8_t start = 0xFF;
  const uint8_t code = 0xAA;
  uint8_t length = 27;
  const uint8_t stop = 0xFE;
};

class Ackermann_control
{
public:
  data_container commands;
  float steering_angle;
  float steering_angle_velocity;
  float speed;
  float acceleration;
  float jerk;
  uint8_t flag1;
  uint8_t flag2;
  uint8_t flag3;
};

class USB_STM
{
private:
  int fd; //file descriptor
public:
  Ackermann_control control;
  int init(int speed = B115200);
  void usb_read_buffer(int buf_size, uint32_t& timestamp, float& velocity, float& quaternion_x, float& quaternion_y, float& quaternion_z, float& quaternion_w, float& ang_vel_x, float& ang_vel_y, float& ang_vel_z, float& lin_acc_x, float& lin_acc_y, float& lin_acc_z, uint8_t& taranis_3_pos, uint8_t& taranis_reset_gear, uint8_t& stm_reset);
  void usb_send_buffer(uint32_t timestamp_ms, float steering_angle, float steering_angle_velocity, float speed, float acceleration, float jerk, uint8_t flag1, uint8_t flag2, uint8_t flag3);

};

#endif // USB_HPP
