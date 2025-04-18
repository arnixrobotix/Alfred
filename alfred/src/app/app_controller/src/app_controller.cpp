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

  twistPublisher = this->create_publisher<TwistMsg_t>("cmd_velocity", 10);

  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Controller::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  twistPublisher->on_activate();

  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Controller::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  twistPublisher->on_deactivate();

  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Controller::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  odometrySubscriber.reset();
  twistPublisher.reset();

  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Controller::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  odometrySubscriber.reset();
  twistPublisher.reset();

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

  // phi (sensor's x-axis rotation)
  float tanPhi = 2 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);
  float quadrantPhi = 2 * (quaternion.w * quaternion.w + quaternion.z * quaternion.z) - 1;
  auto phi = std::atan2(tanPhi, quadrantPhi) * 180 / M_PI;

  // theta (sensor's y-axis rotation)
  float sinTheta = 2 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);
  auto theta = -std::asin(sinTheta) * 180 / M_PI;

  // psi (sensor's z-axis rotation)
  float tanPsi = 2 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
  float quadrantPsi = 2 * (quaternion.w * quaternion.w + quaternion.x * quaternion.x) - 1;
  auto psi = std::atan2(tanPsi, quadrantPsi) * 180 / M_PI;

  RCLCPP_INFO(get_logger(), "Phi: %f, Theta: %f, Psi: %f", phi, theta, psi);
}

}  // namespace controller
}  // namespace app
