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

#include "bno055_registers.hpp"

/**
 * @brief Struct for storing the BNO055 sensor data
 *
 * @note The order of this struct is designed to match the I2C registers
 * so all data can be read in one fell swoop
 *
 */
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

/**
 * @class BNO055
 * @brief A class to interface with the BNO055 IMU sensor over I2C
 *
 */
class BNO055 {
public:
  /**
   * @brief Construct a new BNO055 object
   *
   */
  BNO055() = default;

  /**
   * @brief Destroy the BNO055 object
   *
   */
  ~BNO055() = default;

  /**
   * @brief Initialize the I2C device and establish communication
   *
   * @param i2c_dev the device name for the hardware I2C on the system
   * @param i2c_addr the I2C address of the BNO055 sensor
   */
  void init(std::string i2c_dev, uint8_t i2c_addr);

  /**
   * @brief Resets the sensor
   *
   * @return true if the reset was successful
   * @return false if the reset failed for any reason
   */
  bool reset();

  /**
   * @brief Read all of the sensor data and pack it into one struct to be returned
   *
   * @return IMURecord the struct containing all of the sensor data
   */
  IMURecord read();

private:
  /**
  * @brief The file descriptor for I2C device
  *
  */
  int file;
};

#endif // _BNO055_HPP_