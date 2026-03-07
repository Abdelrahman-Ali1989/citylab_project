#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <thread>

class Patrol : public rclcpp::Node {

public:
  Patrol();
  ~Patrol();

private:
  // functions
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publish_velocity(double linear, double angular);
  void run_patrol();

  // subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laserscan_subscription_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr reentrant_group_1_;

  // timer for 10 Hz patrol loop
  rclcpp::TimerBase::SharedPtr patrol_timer_;

  // variables
  double direction_;
  float front_range_;
};