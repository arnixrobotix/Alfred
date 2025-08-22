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

  computeAndPublishCommand(pitch);
}

void Controller::computeAndPublishCommand(double angle)
{
  HalMotorControlCommandMsg_t command;

  // Observer
  static float theta_obs_prev = 0.0;
  static float theta_dot_obs_prev = 0.0;
  static float theta_error_sum = 0.0;
  static float torque_prev = 0.0;

  float theta_obs = -0.7758 * theta_obs_prev + 0.002784 * theta_dot_obs_prev - 0.03122 *
    torque_prev + 1.0812 * angle;
  float theta_dot_obs = -45.5195 * theta_obs_prev + 0.6344 * theta_dot_obs_prev - 11.8724 *
    torque_prev + 46.9097 * angle;
  theta_error_sum -= angle;
  float torque = 0.46837 * theta_obs + 0.02747 * theta_dot_obs + 1.4972 * theta_error_sum * 0.01;

  theta_obs_prev = theta_obs;
  theta_dot_obs_prev = theta_dot_obs;
  torque_prev = torque;

  command.motor_left_command = torque / 2.0;
  command.motor_right_command = torque / 2.0;

  commandPublisher->publish(command);
}

}  // namespace controller
}  // namespace app
