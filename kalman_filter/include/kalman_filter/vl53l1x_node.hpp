#ifndef VL53L1X_NODE_HPP
#define VL53L1X_NODE_HPP


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "vl53l1x.hpp"  // Library for the range sensor

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class VL53L1XNode : public rclcpp::Node {
public:
  VL53L1XNode();
  ~VL53L1XNode();

private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sensor_pub;
  VL53L1X sensor;
};

#endif // !VL53L1X_NODE_HPP