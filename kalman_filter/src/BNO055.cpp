/* BNO055.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 * 
 * Modified by Sharwin Patil
 */

#include "BNO055.hpp"
#include "BNO055_Registers.hpp"
#include "smbus_functions.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

BNO055I2C::BNO055I2C(std::string i2c_dev, uint8_t i2c_addr) : Node("bno055") {
  this->device = i2c_dev;
  this->address = i2c_addr;

  // Initialize I2C and the BNO055 device
  this->init();

  // Declare parameters
  this->timer_freq = this->declare_parameter("timer_freq", 100.0); // [Hz]

  // Initialize the publishers
  imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("bno055_data/imu", 10);
  mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>("bno055_data/mag", 10);
  temp_pub = this->create_publisher<sensor_msgs::msg::Temperature>("bno055_data/temp", 10);

  // Timer callback for reading the data and publishing it
  auto timer_callback = [this]() -> void {
    IMURecord record = this->read();

    // Create a header for the IMU data
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "imu_frame";
    
    // IMU data
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header = header;
    imu_msg.linear_acceleration.x = record.raw_linear_acceleration_x;
    imu_msg.linear_acceleration.y = record.raw_linear_acceleration_y;
    imu_msg.linear_acceleration.z = record.raw_linear_acceleration_z;
    imu_msg.angular_velocity.x = record.raw_angular_velocity_x;
    imu_msg.angular_velocity.y = record.raw_angular_velocity_y;
    imu_msg.angular_velocity.z = record.raw_angular_velocity_z;
    imu_msg.orientation.w = record.fused_orientation_w;
    imu_msg.orientation.x = record.fused_orientation_x;
    imu_msg.orientation.y = record.fused_orientation_y;
    imu_msg.orientation.z = record.fused_orientation_z;
    
    // Magnetic field data
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header = header;
    mag_msg.magnetic_field.x = record.raw_magnetic_field_x;
    mag_msg.magnetic_field.y = record.raw_magnetic_field_y;
    mag_msg.magnetic_field.z = record.raw_magnetic_field_z;
    
    // Temperature data
    sensor_msgs::msg::Temperature temp_msg;
    temp_msg.header = header;
    temp_msg.temperature = record.temperature;

    // Publish the data
    imu_pub->publish(imu_msg);
    mag_pub->publish(mag_msg);
    temp_pub->publish(temp_msg);
  };

  // Create a timer with a 100ms period
  auto timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / this->timer_freq), // [s]
    timer_callback);
}

bool BNO055I2C::reset() {
  
  _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  
  // reset
  _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0x20);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  
  int i = 0;
  // wait for chip to come back online
  while (_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (i++ > 500) {
      throw std::runtime_error("chip did not come back online within 5 seconds of reset");
      return false;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // normal power mode
  _i2c_smbus_write_byte_data(file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);
  _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  return true;
}

void BNO055I2C::init() {

  file = open(device.c_str(), O_RDWR);

  if (ioctl(file, I2C_SLAVE, address) < 0) {
    throw std::runtime_error("i2c device open failed");
  }

  if (_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    throw std::runtime_error("incorrect chip ID");
  }

  std::cerr << "rev ids:"
    << " accel:" << _i2c_smbus_read_byte_data(file, BNO055_ACCEL_REV_ID_ADDR)
    << " mag:" << _i2c_smbus_read_byte_data(file, BNO055_MAG_REV_ID_ADDR)
    << " gyro:" << _i2c_smbus_read_byte_data(file, BNO055_GYRO_REV_ID_ADDR)
    << " sw:" << _i2c_smbus_read_word_data(file, BNO055_SW_REV_ID_LSB_ADDR)
    << " bl:" << _i2c_smbus_read_byte_data(file, BNO055_BL_REV_ID_ADDR) << std::endl;

  if (!this->reset()) {
    throw std::runtime_error("chip init failed");
  }
}

IMURecord BNO055I2C::read() {
  IMURecord record;
  unsigned char c = 0;

  // can only read a length of 0x20 at a time, so do it in 2 reads
  // BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
  if (_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
    throw std::runtime_error("read error");
  }
  if (_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13) {
    throw std::runtime_error("read error");
  }

  return record;
}
