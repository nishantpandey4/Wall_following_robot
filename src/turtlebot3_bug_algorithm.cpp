// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#include "turtlebot3_gazebo/turtlebot3_bug_algorithm.hpp"

#include <memory>

using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node"){
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;
  robot_pose_x_=0.0;
  robot_pose_y_=0.0;
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Turtlebot3Drive::scan_callback, \
      this, \
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

  robot_pose_x_ = msg->pose.pose.position.x;
  robot_pose_y_ = msg->pose.pose.position.y;
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  uint16_t scan_angle[3] = {0, 60, 300};

  for (int num = 0; num < 3; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}


/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
    static uint8_t turtlebot3_state_num = 0;
    const double check_forward_dist = 0.7;
    const double check_side_dist = 0.9;
    const double tolerance = 0.2;
    static bool priority = 1;
    // Check the environment and decide the movement
    switch (turtlebot3_state_num)
    {
        case GET_TB3_DIRECTION:
            if (scan_data_[CENTER] > check_forward_dist&&
        scan_data_[LEFT] > check_side_dist &&
        scan_data_[RIGHT] > check_side_dist)
            {
                turtlebot3_state_num = TB3_DRIVE_FORWARD;
            }
            else
            {
                turtlebot3_state_num = TB3_OBSTACLE_AVOIDANCE;
            }
            break;

        case TB3_DRIVE_FORWARD:
            if (scan_data_[CENTER] > check_forward_dist+tolerance)
            {
                update_cmd_vel(LINEAR_VELOCITY, 0.0); // Move forward
            }
            else
            {
                turtlebot3_state_num = TB3_OBSTACLE_AVOIDANCE; // Obstacle detected, switch to avoidance
            }
            break;

        case TB3_OBSTACLE_AVOIDANCE:
            // Implement simple obstacle circumnavigation logic
            if (scan_data_[CENTER] < check_forward_dist)
            {
                // If there's an obstacle ahead, turn right
                update_cmd_vel(0.0, -ANGULAR_VELOCITY-0.2);
                RCLCPP_INFO(this->get_logger(), "TURNING RIGHT");
                priority =0;

            }
            else if (scan_data_[LEFT] > check_side_dist)
            {
                // If there's too much space on the left, turn left to follow the wall
                update_cmd_vel(0.0, ANGULAR_VELOCITY/1.8);
                RCLCPP_INFO(this->get_logger(), "TURNING LEFT");

            }
            else if (!priority&&scan_data_[LEFT] < check_side_dist-tolerance)
            {
                // If there's too much space on the left, turn left to follow the wall
                update_cmd_vel(LINEAR_VELOCITY/2.0, -ANGULAR_VELOCITY);
                RCLCPP_INFO(this->get_logger(), "TURNING RIGHT 1");
                priority =1;
            }
            else
            {
                // Otherwise, keep moving forward while maintaining a safe distance from the wall
                update_cmd_vel(LINEAR_VELOCITY, 0.0);
                RCLCPP_INFO(this->get_logger(), "STRAIGHT");
            }
            break;

        default:
            turtlebot3_state_num = GET_TB3_DIRECTION;
            break;
    }
}



/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}
