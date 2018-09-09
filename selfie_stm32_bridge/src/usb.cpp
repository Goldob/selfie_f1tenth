#include "usb.hpp"


int USB_STM::init(int speed)
{
  char port[] = "/dev/serial/by-id/usb-KNR_Selfie_F7_00000000001A-if00";
  fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
    std::cout << "Could not open serial communication on port: " << port << std::endl;
  else
  {
    std::cout << "Opened serial communication on port: " << port << std::endl;
    std::cout << "File descriptor: " << fd << std::endl;
  }

  if (fd < 0)
  {
    std::cout << "Could not open any USB port" << std::endl;
    return -1;
  }

  // Get attributes of transmission
  struct termios tty;
  if (tcgetattr(fd, &tty) < 0)
  {
    std::cout << "Error while getting attributes!" << std::endl;
    return -2;
  }

  // Set input and output speed
  cfsetospeed(&tty, B921600);
  cfsetispeed(&tty, B921600);

  tty.c_cflag |= (CLOCAL | CREAD);    // program will not become owner of port
  tty.c_cflag &= ~CSIZE;              // bit mask for data bits
  tty.c_cflag |= CS8;                 // 8 bit data lenght
  tty.c_cflag |= PARENB;              // enable parity
  tty.c_cflag &= ~PARODD;             // even parity
  tty.c_cflag &= ~CSTOPB;             // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;            // no hardware flowcontrol

  // non-canonical mode
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  // fetch bytes asap
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

  // Set new parameters of transmission
  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    std::cout << "Error while setting attributes!" << std::endl;
    return -3;
  }
  return 1;
}

void USB_STM::usb_read_buffer(int buf_size, uint32_t& timestamp, float& velocity, float& quaternion_x, float& quaternion_y, float& quaternion_z, float& quaternion_w, float& ang_vel_x, float& ang_vel_y, float& ang_vel_z, float& lin_acc_x, float& lin_acc_y, float& lin_acc_z, uint8_t& taranis_3_pos, uint8_t& taranis_reset_gear, uint8_t& stm_reset)
{

  struct UsbFrame_s
  {
    uint8_t startbyte;
    uint8_t code;
    uint8_t length;

    uint32_t timecode;
    float velocity;
    float quaternion[4];
    float rates[3];
    float acc[3];

    uint8_t taranis_C;
    uint8_t taranis_F;
    uint8_t vision_res;
    uint8_t endByte;
  } __attribute__((__packed__));
  union UsbFrame_u
  {
    unsigned char buffer[512];
    struct UsbFrame_s frame;
  } Data;

  int read_state = read(fd, &Data.buffer[0], 512) ;

  if (read_state == 55 && Data.frame.startbyte == 0xff
      && Data.frame.code == 0x40 && Data.frame.length == 51
      && Data.frame.endByte == 0xfe)
  {
    //timestamp
    timestamp = Data.frame.timecode;

    //car velocity
    velocity = Data.frame.velocity;

    //imu data
    quaternion_x = Data.frame.quaternion[0];
    quaternion_y = Data.frame.quaternion[1];
    quaternion_z = Data.frame.quaternion[2];
    quaternion_w = Data.frame.quaternion[3];
    ang_vel_x = Data.frame.rates[0];
    ang_vel_y = Data.frame.rates[1];
    ang_vel_z = Data.frame.rates[2];
    lin_acc_x = Data.frame.acc[0];
    lin_acc_y = Data.frame.acc[1];
    lin_acc_z = Data.frame.acc[2];

    taranis_3_pos = Data.frame.taranis_C;
    taranis_reset_gear = Data.frame.taranis_F;
    stm_reset = Data.frame.vision_res;
    ROS_INFO("Bytes: %d Time: %u Gz: %.3f VeloZ: %.3f", read_state, timestamp, lin_acc_z, ang_vel_z);
  }
}

void USB_STM::usb_send_buffer(uint32_t timestamp_ms, float steering_angle, float steering_angle_velocity, float speed, float acceleration, float jerk, uint8_t flag1, uint8_t flag2, uint8_t flag3)
{

  struct UsbFrame_s
  {
    uint8_t startbyte;
    uint8_t code;
    uint8_t length;
    uint32_t timestamp_ms;
    float steering_angle;
    float steering_angle_velocity;
    float speed;
    float acceleration;
    float jerk;
    uint8_t flag1;
    uint8_t flag2;
    uint8_t flag3;
    uint8_t endbyte;
  } __attribute__((__packed__));
  union UsbFrame_u
  {
    unsigned char bytes[31];
    struct UsbFrame_s frame;
  } Data;
  Data.frame.startbyte = control.commands.start;
  Data.frame.code = control.commands.code;
  Data.frame.length = control.commands.length;
  Data.frame.timestamp_ms = timestamp_ms;
  Data.frame.steering_angle = steering_angle;
  Data.frame.steering_angle_velocity = steering_angle_velocity;
  Data.frame.speed = speed;
  Data.frame.acceleration = acceleration;
  Data.frame.jerk = jerk;
  Data.frame.flag1 = flag1;
  Data.frame.flag2 = flag2;
  Data.frame.flag3 = flag3;
  Data.frame.endbyte = control.commands.stop;
  write(fd, &Data.bytes, 31);
}





