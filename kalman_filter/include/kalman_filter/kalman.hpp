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

typedef struct {
  float alpha;
  float beta;
  float estimate;
  float previousEstimate;
  float previousRateEstimate;
  rclcpp::Time prevMeasureTime;
} AlphaBetaFilter;

class KalmanFilter : public rclcpp::Node {
public:
  KalmanFilter();
  ~KalmanFilter() = default;

private:
float H; // height from the ground to the fixed base of the robot [mm]
AlphaBetaFilter rangeFilter;
AlphaBetaFilter tempFilter;

void applyAlphaBetaFilter(float z, AlphaBetaFilter &filter);

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
void temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);
void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);
void timer_callback();

// Raw data subscriptions
rclcpp::Subscription<IMU>::SharedPtr imu_subscription_;
rclcpp::Subscription<MagField>::SharedPtr mag_subscription_;
rclcpp::Subscription<Temp>::SharedPtr temp_subscription_;
rclcpp::Subscription<Range>::SharedPtr range_subscription_;

// Filtered data publishers
rclcpp::Publisher<IMU>::SharedPtr filtered_imu_pub;
rclcpp::Publisher<MagField>::SharedPtr filtered_mag_pub;
rclcpp::Publisher<Temp>::SharedPtr filtered_temp_pub;
rclcpp::Publisher<Range>::SharedPtr filtered_range_pub;

rclcpp::TimerBase::SharedPtr timer;
};

#endif // !KALMAN_HPP