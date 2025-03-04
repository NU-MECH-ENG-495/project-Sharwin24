#ifndef KALMAN_HPP
#define KALMAN_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"

using IMU = sensor_msgs::msg::Imu;
using MagField = sensor_msgs::msg::MagneticField;
using Temp = sensor_msgs::msg::Temperature;
using Range = sensor_msgs::msg::Range;

class KalmanFilter : public rclcpp::Node {
public:
  KalmanFilter();
  ~KalmanFilter() = default;

private:
void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
void temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);
void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);
void timer_callback();

rclcpp::Subscription<IMU>::SharedPtr imu_subscription_;
rclcpp::Subscription<MagField>::SharedPtr mag_subscription_;
rclcpp::Subscription<Temp>::SharedPtr temp_subscription_;
rclcpp::Subscription<Range>::SharedPtr range_subscription_;

rclcpp::TimerBase::SharedPtr timer;
float timer_freq; // [Hz]
};

#endif // !KALMAN_HPP