
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cmath>
#include <patrol.hpp>

Patrol::Patrol() : Node("plant_detector_node") {

  // Create a reentrant callback group
  reentrant_group_1_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Set up subscription options to use the reentrant callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = reentrant_group_1_;

  // Create subscription for laser scan data with reentrant callback group
  laserscan_subscription_ =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/fastbot_1/scan", 10,
          [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            this->laserscan_callback(msg);
          },
          sub_options);

  // Create publisher for velocity commands
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/fastbot_1/cmd_vel", 10);

  // setup timer options
  rclcpp::TimerBase::Options timer_options;
  timer_options.callback_group = reentrant_group_1_;

  // create a 10Hz (100ms) wall timer that will be used to call the control loop
  patrol_timer_ = this->create_wall_timer(
      100ms, [this]() { this->run_patrol(); }, timer_options);

  RCLCPP_INFO(this->get_logger(), "Subsicribers and publishers initialized");
}

Patrol::~Patrol() {}

void Patrol::laserscan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  double start_angle = -M_PI / 2.0;
  double end_angle = M_PI / 2.0;

  int start_index = static_cast<int>(
      std::ceil((start_angle - msg->angle_min) / msg->angle_increment));
  int end_index = static_cast<int>(
      std::floor((end_angle - msg->angle_min) / msg->angle_increment));
  int front_index = static_cast<int>(
      std::floor((0.0 - msg->angle_min) / msg->angle_increment));

  front_range_ = msg->ranges[front_index];

  float max_range = -1;
  int index_of_max_range = -1;

  for (int i = start_index; i <= end_index; i++) {
    float range = msg->ranges[i];

    // ignore inf values
    if (!std::isfinite(range)) {
      continue;
    }

    // ignore faulty range values that are out of the official bounds
    if (range < msg->range_min || range > msg->range_max) {
      continue;
    }

    // pick the further range value and put it in the max_range variable, and
    // capture its index
    if (range > max_range) {
      max_range = range;
      index_of_max_range = i;
    }
  }

  // calculate the angle of the furthest range value
  if (index_of_max_range >= 0) {
    direction_ = (index_of_max_range * msg->angle_increment) + msg->angle_min;
  } else {
    RCLCPP_WARN(this->get_logger(), "No valid laser readings found");
  }
}

void Patrol::publish_velocity(double linear, double angular) {
  auto vel_msg = geometry_msgs::msg::Twist();
  vel_msg.linear.x = linear;
  vel_msg.angular.z = angular;
  cmd_vel_publisher_->publish(vel_msg);
}

void Patrol::run_patrol() {

  double angular_vel, linear_vel = 0.1;

  /* If there is an obstacle, let angular_vel get the direction angle value
    divided by 2, otherwise angular_vel should be zero (always heading forward)
  */
  if (front_range_ < 0.35) {
    angular_vel = direction_ / 2.0;
  } else {
    angular_vel = 0.0;
  }

  // publish velocity commands
  publish_velocity(linear_vel, angular_vel);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol_robot = std::make_shared<Patrol>();

  // Use MultiThreadedExecutor with 2 threads
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);
  executor.add_node(patrol_robot);

  try {
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(patrol_robot->get_logger(), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}