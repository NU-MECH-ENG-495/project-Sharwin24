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
  float rateEstimate;
  float previousEstimate = 0;
  float previousRateEstimate = 0;
  rclcpp::Time prevMeasureTime = rclcpp::Time(0);
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

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
void tempCallback(const sensor_msgs::msg::Temperature::SharedPtr msg);
void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg);

// Raw data subscriptions
rclcpp::Subscription<IMU>::SharedPtr raw_imu_sub;
rclcpp::Subscription<MagField>::SharedPtr raw_mag_sub;
rclcpp::Subscription<Temp>::SharedPtr raw_temp_sub;
rclcpp::Subscription<Range>::SharedPtr raw_range_sub;

// Filtered data publishers
rclcpp::Publisher<IMU>::SharedPtr filtered_imu_pub;
rclcpp::Publisher<MagField>::SharedPtr filtered_mag_pub;
rclcpp::Publisher<Temp>::SharedPtr filtered_temp_pub;
rclcpp::Publisher<Range>::SharedPtr filtered_range_pub;
};

#endif // !KALMAN_HPP