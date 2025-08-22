// Copyright (c) 2024 Arnix Robotix
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

#include "app_controller.hpp"

using namespace std::placeholders;

namespace app
{
namespace controller
{

Controller::Controller()
: rclcpp_lifecycle::LifecycleNode{"app_controller_node"}
{
}

LifecycleCallbackReturn_t Controller::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  odometrySubscriber = this->create_subscription<OdometryMsg_t>(
    "odometry", 10, std::bind(&Controller::odometryReader, this, _1));

  commandPublisher = this->create_publisher<HalMotorControlCommandMsg_t>("cmd_torque", 10);

  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Controller::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  commandPublisher->on_activate();

  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Controller::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  commandPublisher->on_deactivate();

  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Controller::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  odometrySubscriber.reset();
  commandPublisher.reset();

  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Controller::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  odometrySubscriber.reset();
  commandPublisher.reset();

  RCLCPP_INFO(get_logger(), "Node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Controller::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void Controller::odometryReader(const OdometryMsg_t & msg)
{
  static float prev_x_position = 0.0;
  static float prev_y_position = 0.0;
  static float distance = 0.0;

  Quaternion quaternion{msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y, msg.pose.pose.orientation.z};

  // pitch (sensor's x-axis rotation)
  float tanPitch = 2 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);
  float quadrantPitch = 2 * (quaternion.w * quaternion.w + quaternion.z * quaternion.z) - 1;
  auto pitch = std::atan2(tanPitch, quadrantPitch);

  // roll (sensor's y-axis rotation)
  float sinRoll = 2 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);
  auto roll = std::asin(sinRoll);

  // yaw (sensor's z-axis rotation)
  float tanYaw = 2 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
  float quadrantYaw = 2 * (quaternion.w * quaternion.w + quaternion.x * quaternion.x) - 1;
  auto yaw = std::atan2(tanYaw, quadrantYaw);

  RCLCPP_INFO(
    get_logger(), "Pitch: %f, Roll: %f, Yaw: %f", pitch * 180 / M_PI, roll * 180 / M_PI,
    yaw * 180 / M_PI);

  float x_position = msg.pose.pose.position.x;
  float y_position = msg.pose.pose.position.y;

  distance +=
    std::sqrt(
    std::pow(x_position - prev_x_position, 2) + std::pow(y_position - prev_y_position, 2));

  prev_x_position = x_position;
  prev_y_position = y_position;

  computeAndPublishCommand(pitch, yaw, distance);
}

void Controller::computeAndPublishCommand(double angle, double heading, double distance)
{
  HalMotorControlCommandMsg_t command;

  // Observer
  static float x_obs_prev = 0.0;
  static float x_dot_obs_prev = 0.0;
  static float x_error_sum = 0.0;
  static float theta_obs_prev = 0.0;
  static float theta_dot_obs_prev = 0.0;
  static float theta_error_sum = 0.0;
  static float psi_obs_prev = 0.0;
  static float psi_dot_obs_prev = 0.0;
  static float psi_error_sum = 0.0;
  static float torque_left_prev = 0.0;
  static float torque_right_prev = 0.0;

  float x_obs = 0.9846 * x_obs_prev + 0.0001 * x_dot_obs_prev - 0.0072 * theta_obs_prev -
    0.000003655 * theta_dot_obs_prev + 0.000052236 * torque_left_prev + 0.000052236 *
    torque_right_prev + 0.0154 * distance + 0.0072 * angle;
  float x_dot_obs = -0.0951 * x_obs_prev + x_dot_obs_prev - 0.4477 * theta_obs_prev + 0.0002 *
    theta_dot_obs_prev + 0.1085 * torque_left_prev + 0.1085 * torque_right_prev +
    0.095 * distance + 0.4366 * angle;
  float theta_obs = -0.0073 * x_obs_prev - 0.0000036676 * x_dot_obs_prev + 0.8833 *
    theta_obs_prev + 0.0009 * theta_dot_obs_prev + 0.0007 * torque_left_prev + 0.0007 *
    torque_right_prev + 0.0073 * distance + 0.1170 * angle;
  float theta_dot_obs = -0.526 * x_obs_prev - 0.0003 * x_dot_obs_prev - 6.2310 * theta_obs_prev +
    0.9968 * theta_dot_obs_prev + 1.478 * torque_left_prev + 1.478 * torque_right_prev +
    0.526 * distance + 6.8643 * angle;
  float psi_obs = 0.9146 * psi_obs_prev + 0.001 * psi_dot_obs_prev - 0.0004 *
    torque_left_prev + 0.0004 * torque_right_prev + 0.0854 * heading;
  float psi_dot_obs = -3.6 * psi_obs_prev + 0.9981 * psi_dot_obs_prev - 0.8406 *
    torque_left_prev + 0.8406 * torque_right_prev + 3.6 * heading;

  // Command
  x_error_sum -= distance;
  theta_error_sum -= angle;
  psi_error_sum -= heading;
  float torque_left = 1.528 * x_obs + 0.3149 * x_dot_obs + 1.3802 * theta_obs - 0.0521 *
    theta_dot_obs + 0.0855 * psi_obs + 0.0095 * psi_dot_obs + 4.2317 * x_error_sum * 0.01 -
    1.967 * theta_error_sum * 0.01 + 0.4045 * psi_error_sum * 0.01;
  float torque_right = 1.528 * x_obs + 0.3149 * x_dot_obs + 1.3802 * theta_obs - 0.0521 *
    theta_dot_obs - 0.0855 * psi_obs - 0.0095 * psi_dot_obs + 4.2317 * x_error_sum * 0.01 -
    1.967 * theta_error_sum * 0.01 - 0.4045 * psi_error_sum * 0.01;

  x_obs_prev = x_obs;
  x_dot_obs_prev = x_dot_obs;
  theta_obs_prev = theta_obs;
  theta_dot_obs_prev = theta_dot_obs;
  psi_obs_prev = psi_obs;
  psi_dot_obs_prev = psi_dot_obs;
  torque_left_prev = torque_left;
  torque_right_prev = torque_right;

  command.motor_left_command = torque_left;
  command.motor_right_command = torque_right;

  commandPublisher->publish(command);
}

}  // namespace controller
}  // namespace app
