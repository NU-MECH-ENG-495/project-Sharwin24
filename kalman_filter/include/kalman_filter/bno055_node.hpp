#ifndef _BNO055_NODE_HPP_
#define _BNO055_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "bno055.hpp"

class BNO055Node : public rclcpp::Node {
public:
  BNO055Node();
  ~BNO055Node() = default;

private:
  BNO055 sensor;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;
};

#endif // _BNO055_NODE_HPP_