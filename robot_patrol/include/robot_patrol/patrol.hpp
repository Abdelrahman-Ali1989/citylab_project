#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

class Patrol : public rclcpp::Node {

public:
  Patrol();
  ~Patrol();

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laserscan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  float direction_;
};