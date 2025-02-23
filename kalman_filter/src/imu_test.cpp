#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

// Sensor Hub Transport Protocol
// #include "sh2.h"

// I2C Bus on Raspberry Pi 5
#define I2C_DEVICE "/dev/i2c-1"
// I2C address of the BN0O85 IMU
#define IMU_ADDR 0x4A

#define FIFTY_HZ 20000

uint8_t CARGO_NO = 0x00; // SHTP Sequence Number
uint8_t START_BNO_ALGO[21] = {
  0x15, 0x00, 0x02, CARGO_NO, 0xFD, 0x28, 0x00,
  0x00, 0x00, 0x20, 0x4e, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// Stream orientation as ARVR-Stabilized Rotation Vector (0x28) at 50 Hz (20us = 0x4e20)


// 0xFD = SET_FEATURE_CMD
// uint8_t START_BNO_ALGO[21] = { 
//   SH2_RAW_GYROSCOPE, 0x00, SH2_GYROSCOPE_CALIBRATED, CARGO_NO, 0xFD, SH2_ARVR_STABILIZED_RV, 
//   0x00, 0x00, 0x00, SH2_TILT_DETECTOR, 0x4e,
//   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 
// };


// Open I2C communication
int openI2CBus() {
  int file = open(I2C_DEVICE, O_RDWR);
  if (file < 0) {
    std::cerr << "Failed to open I2C bus\n";
    return -1;
  } else {
    std::cout << "Succesfully opened I2C bus (" << I2C_DEVICE << ")" << std::endl;
  }
  if (ioctl(file, I2C_SLAVE, IMU_ADDR) < 0) {
    std::cerr << "Failed to connect to I2C device\n";
    return -1;
  } else {
    std::cout << "Succesfully connected to I2C device (ADDR=" << IMU_ADDR << ")" << std::endl;
  }
  return file;
}

// Write data to a register
bool i2cWrite(int file, uint8_t reg, uint8_t* data, size_t len) {
  uint8_t buffer[len + 1];
  buffer[0] = reg;
  for (size_t i = 0; i < len; i++) {
    buffer[i + 1] = data[i];
  }
  return write(file, buffer, len + 1) == len + 1;
}

// Read data from a register
bool i2cRead(int file, uint8_t reg, uint8_t* data, size_t len) {
  if (write(file, &reg, 1) != 1) {
    std::cerr << "Error setting register address\n";
    return false;
  } else {
    std::cout << "Succesfully set register address\n";
  }
  return read(file, data, len) == len;
}

int main() {
  int file = openI2CBus();
  if (file < 0) return 1;

  // Configure IMU to stream data
  if (!i2cWrite(file, 0x00, START_BNO_ALGO, 21)) {
    std::cerr << "Failed to write to IMU\n";
    close(file);
    return 1;
  } else {
    std::cout << "Succesfully wrote to IMU\n";
  }
  sleep(1); // Give the sensor time to start streaming

  const size_t packetLen = 16; // Adjust based on your protocol details
  uint8_t packet[packetLen];
  while (true) {
    if (i2cRead(file, 0x00, packet, packetLen)) {
      std::cout << "Received packet: ";
      for (size_t i = 0; i < packetLen; i++) {
        // std::cout << std::hex << (int)packet[i] << " ";
        std::printf("%02x ", packet[i]);
      }
      std::cout << std::endl;
    } else {
      std::cerr << "Failed to read from IMU\n";
    }
    usleep(FIFTY_HZ); // 50 Hz is 20ms
  }

  close(file);
  return 0;
}