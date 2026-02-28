
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <patrol.hpp>

Patrol::Patrol() : Node("plant_detector_node") {

  // Create a reentrant callback group
  reentrant_group_1_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Set up subscription options to use the reentrant callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = reentrant_group_1_;

  // Create subscription for camera images with reentrant callback group
  laserscan_subscription_ =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
          image_topic_name_, 10,
          [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            this->laserscan_callback(msg);
          },
          sub_options);

  // Create publisher for velocity commands
  cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

Patrol::~Patrol() {}

void Patrol::laserscan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {}