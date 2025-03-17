#include "rclcpp/rclcpp.hpp"
#include "kalman.hpp"
#include <eigen3/Eigen/Dense>

typedef struct {
  rclcpp::Time timestamp; // timestamp of the state vector
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

KalmanFilter::KalmanFilter() : Node("kalman_filter") {
  // Declare parameters
  float timer_freq = this->declare_parameter("timer_frequency", 100.0); // [Hz]
  this->H = this->declare_parameter("ground_to_base_height", 400.0); // [mm]
  this->rangeFilter.alpha = this->declare_parameter("range_filter_alpha", 0.1);
  this->rangeFilter.beta = this->declare_parameter("range_filter_beta", 0.01);
  this->tempFilter.alpha = this->declare_parameter("temp_filter_alpha", 0.1);
  this->tempFilter.beta = this->declare_parameter("temp_filter_beta", 0.01);
  // Get parameters from yaml file
  timer_freq = this->get_parameter("timer_frequency").as_double();
  this->H = this->get_parameter("ground_to_base_height").as_double();
  this->rangeFilter.alpha = this->get_parameter("range_filter_alpha").as_double();
  this->rangeFilter.beta = this->get_parameter("range_filter_beta").as_double();
  this->tempFilter.alpha = this->get_parameter("temp_filter_alpha").as_double();
  this->tempFilter.beta = this->get_parameter("temp_filter_beta").as_double();
  
  // Setup RangeFilter
  this->rangeFilter.prevMeasureTime = this->now();
  this->rangeFilter.previousEstimate = 0;
  this->rangeFilter.previousRateEstimate = 0;

  // Setup TempFilter
  this->tempFilter.prevMeasureTime = this->now();
  this->tempFilter.previousEstimate = 0;
  this->tempFilter.previousRateEstimate = 0;

  // Raw data Topics
  const std::string imu_raw_topic = "bno055/raw_imu";
  const std::string mag_raw_topic = "bno055/raw_mag";
  const std::string temp_raw_topic = "bno055/raw_temp";
  const std::string range_raw_topic = "vl53l1x/raw_range";

  // Filtered Data Topics
  const std::string imu_filtered_topic = "bno055/filtered_imu";
  const std::string mag_filtered_topic = "bno055/filtered_mag";
  const std::string temp_filtered_topic = "bno055/filtered_temp";
  const std::string range_filtered_topic = "vl53l1x/filtered_range";

  // Set the QoS for the subscribers for receiving raw sensor data
  auto QoS = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  auto filteredQos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  // Initialize the subscribers
  this->imu_subscription_ = this->create_subscription<IMU>(
    imu_raw_topic, QoS, std::bind(&KalmanFilter::imu_callback, this, std::placeholders::_1));
  this->mag_subscription_ = this->create_subscription<MagField>(
    mag_raw_topic, QoS, std::bind(&KalmanFilter::mag_callback, this, std::placeholders::_1));
  this->temp_subscription_ = this->create_subscription<Temp>(
    temp_raw_topic, QoS, std::bind(&KalmanFilter::temp_callback, this, std::placeholders::_1));
  this->range_subscription_ = this->create_subscription<Range>(
    range_raw_topic, QoS, std::bind(&KalmanFilter::range_callback, this, std::placeholders::_1));

  // Initialize Publishers
  this->filtered_imu_pub = this->create_publisher<IMU>(imu_filtered_topic, filteredQos);
  this->filtered_mag_pub = this->create_publisher<MagField>(mag_filtered_topic, filteredQos);
  this->filtered_temp_pub = this->create_publisher<Temp>(temp_filtered_topic, filteredQos);
  this->filtered_range_pub = this->create_publisher<Range>(range_filtered_topic, filteredQos);

  RCLCPP_INFO(this->get_logger(), "Kalman Filter subscribing to raw data on topics: (%s), (%s), (%s), (%s)",
    imu_raw_topic.c_str(), mag_raw_topic.c_str(), temp_raw_topic.c_str(), range_raw_topic.c_str()
  );

  RCLCPP_INFO(this->get_logger(), "Kalman Filter publishing filtered data on topics: (%s), (%s), (%s), (%s)",
    imu_filtered_topic.c_str(), mag_filtered_topic.c_str(), temp_filtered_topic.c_str(), range_filtered_topic.c_str()
  );

  // Create a timer to run the filter
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_freq), // [s]
    std::bind(&KalmanFilter::timer_callback, this)
  );
}

void KalmanFilter::timer_callback() {

}

void KalmanFilter::applyAlphaBetaFilter(float z, AlphaBetaFilter& filter) {
  // Update Timestep and save the previous measurement time
  float dt = (this->now() - filter.prevMeasureTime).seconds();
  filter.prevMeasureTime = this->now();

  // Prediction Step
  float dx = filter.previousRateEstimate;
  filter.estimate = filter.previousEstimate + (dx * dt);
  
  // Update Step
  float residual = z - filter.estimate;
  filter.estimate += filter.alpha * residual;
  dx += filter.beta * (residual / dt);

  // Update the filter state
  filter.previousRateEstimate = dx;
  filter.previousEstimate = filter.estimate;
}

void KalmanFilter::imu_callback(const IMU::SharedPtr msg) {
  (void)msg;
}

void KalmanFilter::mag_callback(const MagField::SharedPtr msg) {
  (void)msg;
}

void KalmanFilter::temp_callback(const Temp::SharedPtr msg) {
  // Alpha-Beta Filter for Temperature data from BNO055 Sensor
  if (this->tempFilter.previousEstimate == 0 && this->tempFilter.previousRateEstimate == 0) {
    this->tempFilter.previousEstimate = msg->temperature; // Initial Guess
  }

  this->applyAlphaBetaFilter(msg->temperature, this->tempFilter);

  // Publish the filtered temperature value
  auto filtered_msg = std::make_shared<Temp>(*msg);
  filtered_msg->temperature = this->tempFilter.estimate; // Update the temperature value to the filtered value
  filtered_msg->header.stamp = this->now(); // Update the timestamp to the current time
  this->filtered_temp_pub->publish(*filtered_msg);
}

void KalmanFilter::range_callback(const Range::SharedPtr msg) {
  // Alpha-Beta Filter for Range Data from Time-of-Flight Sensor
  if (this->rangeFilter.previousEstimate == 0 && this->rangeFilter.previousRateEstimate == 0) {
    this->rangeFilter.previousEstimate = msg->range; // Initial Guess
  }

  this->applyAlphaBetaFilter(msg->range, this->rangeFilter);

  // Publish the filtered range value
  auto filtered_msg = std::make_shared<Range>(*msg);
  filtered_msg->range = this->rangeFilter.estimate; // Update the range value to the filtered value
  filtered_msg->header.stamp = this->now(); // Update the timestamp to the current time
  // Only publish the filtered range if it is within the min and max range values
  if ((filtered_msg->range >= filtered_msg->min_range) && (filtered_msg->range <= filtered_msg->max_range)) {
    this->filtered_range_pub->publish(*filtered_msg);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilter>());
  rclcpp::shutdown();
  return 0;
}