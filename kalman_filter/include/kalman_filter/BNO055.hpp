#ifndef _BNO055_HPP_
#define _BNO055_HPP_

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <thread>

#include <linux/i2c-dev.h>
#include <smbus_functions.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "BNO055_Registers.hpp"

// order of this struct is designed to match the I2C registers
// so all data can be read in one fell swoop
typedef struct {
  int16_t raw_linear_acceleration_x;
  int16_t raw_linear_acceleration_y;
  int16_t raw_linear_acceleration_z;
  int16_t raw_magnetic_field_x;
  int16_t raw_magnetic_field_y;
  int16_t raw_magnetic_field_z;
  int16_t raw_angular_velocity_x;
  int16_t raw_angular_velocity_y;
  int16_t raw_angular_velocity_z;
  int16_t fused_heading;
  int16_t fused_roll;
  int16_t fused_pitch;
  int16_t fused_orientation_w;
  int16_t fused_orientation_x;
  int16_t fused_orientation_y;
  int16_t fused_orientation_z;
  int16_t fused_linear_acceleration_x;
  int16_t fused_linear_acceleration_y;
  int16_t fused_linear_acceleration_z;
  int16_t gravity_vector_x;
  int16_t gravity_vector_y;
  int16_t gravity_vector_z;
  int8_t temperature;
  uint8_t calibration_status;
  uint8_t self_test_result;
  uint8_t interrupt_status;
  uint8_t system_clock_status;
  uint8_t system_status;
  uint8_t system_error_code;
} IMURecord;

class BNO055I2C : public rclcpp::Node{
public:
  BNO055I2C(std::string i2c_dev, uint8_t i2c_addr);
  ~BNO055I2C();

  void init();
  bool reset();

  IMURecord read();

private:
  int file;
  std::string device;
  uint8_t address;
  float timer_freq;
  
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;
};

#endif // _BNO055_HPP_