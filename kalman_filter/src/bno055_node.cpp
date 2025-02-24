#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "bno055_node.hpp"
#include "bno055.hpp"

#define I2C_DEVICE "/dev/i2c-1"

BNO055Node::BNO055Node() : Node("bno055") {
  // Declare parameters
  this->sensor_freq = this->declare_parameter("sensor_freq", 100.0); // [Hz]
  // Get parameter from yaml file
  this->sensor_freq = this->get_parameter("sensor_freq").as_double();

  // Initialize the BNO055 sensor
  this->sensor.init(I2C_DEVICE, BNO055_ADDRESS_A); //= BNO055(I2C_DEVICE, BNO055_ADDRESS_A);

  // Initialize the publishers
  this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("bno055_data/imu", 10);
  this->mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>("bno055_data/mag", 10);
  this->temp_pub = this->create_publisher<sensor_msgs::msg::Temperature>("bno055_data/temp", 10);

  // Create a timer with a 100ms period to read and publish
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / this->sensor_freq), // [s]
    [this]() -> void {
      IMURecord record = this->sensor.read();
      const std::string sensor_frame_id = "imu_frame";

      // IMU data
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp = this->now();
      imu_msg.header.frame_id = sensor_frame_id;
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
      mag_msg.header.stamp = this->now();
      mag_msg.header.frame_id = sensor_frame_id;
      mag_msg.magnetic_field.x = record.raw_magnetic_field_x;
      mag_msg.magnetic_field.y = record.raw_magnetic_field_y;
      mag_msg.magnetic_field.z = record.raw_magnetic_field_z;

      // Temperature data
      sensor_msgs::msg::Temperature temp_msg;
      temp_msg.header.stamp = this->now();
      temp_msg.header.frame_id = sensor_frame_id;
      temp_msg.temperature = record.temperature;

      // Publish the data
      imu_pub->publish(imu_msg);
      mag_pub->publish(mag_msg);
      temp_pub->publish(temp_msg);
    }
  );
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BNO055Node>());
  rclcpp::shutdown();
  return 0;
}