#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cmath>
#include <robot_patrol/patrol.hpp>

// The constructor definition
Patrol::Patrol() : Node("robot_patrol_node") {

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

  // create a 10Hz (100ms) wall timer that will be used to call the control loop
  patrol_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { this->run_patrol(); },
      reentrant_group_1_);

  RCLCPP_INFO(this->get_logger(), "Subsicribers and publishers initialized");
}

// The destructor
Patrol::~Patrol() {}

// The laserscan callback function
void Patrol::laserscan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  double start_angle = -M_PI / 2.0;
  double end_angle = M_PI / 2.0;

  double left_corner_angle = -2 * M_PI / 3.0; // corner angle is 45 degrees
  double right_corner_angle = 2 * M_PI / 3.0; // corner angle is 45 degrees

  int start_index = static_cast<int>(
      std::ceil((start_angle - msg->angle_min) / msg->angle_increment));
  int end_index = static_cast<int>(
      std::floor((end_angle - msg->angle_min) / msg->angle_increment));
  int front_index = static_cast<int>(
      std::floor((0.0 - msg->angle_min) / msg->angle_increment));
  int left_corner_index = static_cast<int>(
      std::ceil((left_corner_angle - msg->angle_min) / msg->angle_increment));
  int right_corner_index = static_cast<int>(
      std::floor((right_corner_angle - msg->angle_min) / msg->angle_increment));

  /* determine range in front of the robot (surrounding) that can be used
     to decide avoidance motion
  */
  front_range_ = msg->ranges[front_index];
  left_corner_range_ = msg->ranges[left_corner_index];
  right_corner_range_ = msg->ranges[right_corner_index];

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

  RCLCPP_INFO(this->get_logger(), "Reading laser sensor data...");
}

void Patrol::publish_velocity(double linear, double angular) {
  auto vel_msg = geometry_msgs::msg::Twist();

  // components with always zero values
  vel_msg.linear.y = 0.0;
  vel_msg.linear.z = 0.0;
  vel_msg.angular.x = 0.0;
  vel_msg.angular.y = 0.0;

  // control dynamic value components
  vel_msg.linear.x = linear;
  vel_msg.angular.z = angular;
  cmd_vel_publisher_->publish(vel_msg);

  RCLCPP_INFO(this->get_logger(), "Velocity published...");
}

void Patrol::run_patrol() {

  double angular_vel, linear_vel = 0.1;

  /* If there is an obstacle, let angular_vel get the direction angle value
    divided by 2, otherwise angular_vel should be zero (always heading forward)
  */

  if (front_range_ < 0.35) {
    angular_vel = direction_ / 2.0;
  } else if (left_corner_range_ < 0.2 || right_corner_range_ < 0.2) {
    angular_vel = direction_ / 20.0;
  } else {
    angular_vel = 0.0;
  }

  if (front_range_ < 0.15) {
    angular_vel = direction_ / 2.0;
    linear_vel = -0.2;
  }

  // publish velocity commands
  publish_velocity(linear_vel, angular_vel);

  RCLCPP_INFO(this->get_logger(), "Main Control Loop Running...");
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