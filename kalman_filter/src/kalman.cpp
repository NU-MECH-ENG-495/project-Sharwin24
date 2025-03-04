#include "rclcpp/rclcpp.hpp"
#include "kalman.hpp"
#include <eigen3/Eigen/Dense>

using IMU = sensor_msgs::msg::Imu;
using MagField = sensor_msgs::msg::MagneticField;
using Temp = sensor_msgs::msg::Temperature;
using Range = sensor_msgs::msg::Range;

typedef struct {
  float x; // x position of the end effector [mm]
  float y; // y position of the end effector [mm]
  float z; // z position of the end effector [mm]
  float x_dot; // x velocity of the end effector [mm/s]
  float y_dot; // y velocity of the end effector [mm/s]
  float z_dot; // z velocity of the end effector [mm/s]
  float x_ddot; // x acceleration of the end effector [mm/s^2]
  float y_ddot; // y acceleration of the end effector [mm/s^2]
  float z_ddot; // z acceleration of the end effector [mm/s^2]
} StateVector;

KalmanFilter::KalmanFilter() : Node("KalmanFilter") {
  // Declare parameters
  this->timer_freq = this->declare_parameter("timer_freq", 100.0); // [Hz]
  // Get parameter from yaml file
  this->timer_freq = this->get_parameter("timer_freq").as_double();

  // Topics
  const std::string imu_topic = "bno055/imu";
  const std::string mag_topic = "bno055/mag";
  const std::string temp_topic = "bno055/temp";
  const std::string range_topic = "vl53l1x/range";

  // Set the QoS for the subscribers for receiving sensor data
  auto QoS = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  // Initialize the subscribers
  this->imu_subscription_ = this->create_subscription<IMU>(
    imu_topic, QoS, std::bind(&KalmanFilter::imu_callback, this, std::placeholders::_1));
  this->mag_subscription_ = this->create_subscription<MagField>(
    mag_topic, QoS, std::bind(&KalmanFilter::mag_callback, this, std::placeholders::_1));
  this->temp_subscription_ = this->create_subscription<Temp>(
    temp_topic, QoS, std::bind(&KalmanFilter::temp_callback, this, std::placeholders::_1));
  this->range_subscription_ = this->create_subscription<Range>(
    range_topic, QoS, std::bind(&KalmanFilter::range_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Kalman Filter subscribing to topics: (%s), (%s), (%s), (%s)",
    imu_topic.c_str(), mag_topic.c_str(), temp_topic.c_str(), range_topic.c_str());

  // Create a timer with a 100ms period
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / this->timer_freq), // [s]
    std::bind(&KalmanFilter::timer_callback, this)
  );
}

void KalmanFilter::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Timer callback");
}

void KalmanFilter::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "IMU callback");
}

void KalmanFilter::mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Magnetic Field callback");
}

void KalmanFilter::temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Temperature callback");
}

void KalmanFilter::range_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Range callback");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilter>());
  rclcpp::shutdown();
  return 0;
}