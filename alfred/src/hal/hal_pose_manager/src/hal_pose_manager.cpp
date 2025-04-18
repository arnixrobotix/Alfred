// Copyright (c) 2023 Arnix Robotix
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

#include "hal_pose_manager.hpp"

namespace hal
{
namespace pose_manager
{

using namespace std::placeholders;

HalPoseManager::HalPoseManager()
: rclcpp_lifecycle::LifecycleNode{"hal_pose_manager_node"},
  encodersCount{.rightCurrrent = 0, .rightPrevious = 0,
    .leftCurrrent = 0, .leftPrevious = 0,
    .timestampNs = 0},
  wheelsVelocity{.right = 0.0, .left = 0.0},
  orientation{QuaternionMsg_t()},
  angularVelocity{Vector3Msg_t()},
  position_{0.0, 0.0, 0.0},
  prev_position{0.0, 0.0, 0.0}
{
}

LifecycleCallbackReturn_t HalPoseManager::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher = this->create_publisher<OdometryMsg_t>("odometry", 10);
  wheelsVelocityCmdPublisher = this->create_publisher<HalMotorControlCommandMsg_t>(
    "wheelsVelocityCmd", 10);
  twistSubscriber = this->create_subscription<TwistMsg_t>(
    "cmd_velocity", 10, std::bind(&HalPoseManager::computeAndPublishwheelsVelocityCmd, this, _1));
  motorsECSubscriber = this->create_subscription<HalMotorControlEncodersMsg_t>(
    "motorsEncoderCountValue", 10,
    std::bind(&HalPoseManager::computeAndPublishOdometry, this, _1));
  imuSubscriber = this->create_subscription<ImuDataMsg_t>(
    "imuData", 10, std::bind(&HalPoseManager::imuDataReader, this, _1));

  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t HalPoseManager::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher->on_activate();
  wheelsVelocityCmdPublisher->on_activate();

  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t HalPoseManager::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher->on_deactivate();
  wheelsVelocityCmdPublisher->on_deactivate();

  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t HalPoseManager::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher.reset();
  wheelsVelocityCmdPublisher.reset();
  twistSubscriber.reset();
  motorsECSubscriber.reset();

  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t HalPoseManager::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher.reset();
  wheelsVelocityCmdPublisher.reset();
  twistSubscriber.reset();
  motorsECSubscriber.reset();

  RCLCPP_INFO(get_logger(), "Node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t HalPoseManager::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void HalPoseManager::computeAndPublishwheelsVelocityCmd(const TwistMsg_t & msg)
{
  auto wheelsVelocityCommandMsg = HalMotorControlCommandMsg_t();

  wheelsVelocityCommandMsg.motor_left_velocity_command = msg.twist.linear.x;
  wheelsVelocityCommandMsg.motor_right_velocity_command = msg.twist.linear.x;

  wheelsVelocityCmdPublisher->publish(wheelsVelocityCommandMsg);
}

void HalPoseManager::imuDataReader(const ImuDataMsg_t & msg)
{
  orientation = msg.orientation;
  angularVelocity = msg.angular_velocity;
}

void HalPoseManager::computeAndPublishOdometry(const HalMotorControlEncodersMsg_t & msg)
{
  auto header = HeaderMsg_t();
  auto odometry = OdometryMsg_t();
  auto pose = PoseMsg_t();
  auto twist = TwistMsg_t();
  auto position = PointMsg_t();

  int32_t encoderCountDeltaLeft = 0;
  int32_t encoderCountDeltaRight = 0;

  encodersCount.leftPrevious = encodersCount.leftCurrrent;
  encodersCount.leftCurrrent = msg.motor_left_encoder_count;
  encoderCountDeltaLeft = encodersCount.leftCurrrent - encodersCount.leftPrevious;

  encodersCount.rightPrevious = encodersCount.rightCurrrent;
  encodersCount.rightCurrrent = msg.motor_right_encoder_count;
  encoderCountDeltaRight = encodersCount.rightCurrrent - encodersCount.rightPrevious;

  // Robot frame: X axis is oriented to the front of the robot
  //              Y axis is colinear to the axis of the wheels
  //              Z axis is vertical up
  // This frame is attached to the motors, so only the pendulum movement happens in it,
  // i.e. the rotation of the body around the axis of the motors
  twist.twist.angular.y = angularVelocity.y;
  odometry.twist = std::move(twist);

  double theta = wheelRadius_m / (2.0 * robotWidth_m) *
    (encoderCountDeltaRight - encoderCountDeltaLeft) * EncoderCountToRadians;
  double deltaX = wheelRadius_m / 2.0 * std::cos(theta) *
    (encoderCountDeltaRight + encoderCountDeltaLeft) * EncoderCountToRadians;
  double deltaY = wheelRadius_m / 2.0 * std::sin(theta) *
    (encoderCountDeltaRight + encoderCountDeltaLeft) * EncoderCountToRadians;

  position_.x = prev_position.x + deltaX;
  position_.y = prev_position.y + deltaY;
  prev_position = position_;

  position.x = position_.x;
  position.y = position_.y;
  pose.pose.position = position;

  // Orientation from IMU is already expressed in World frame
  pose.pose.orientation = orientation;
  odometry.child_frame_id = "Robot";
  odometry.pose = std::move(pose);

  header.frame_id = "World";
  header.stamp = rclcpp::Clock().now();
  odometry.header = std::move(header);

  odometryPublisher->publish(odometry);
}

}  // namespace pose_manager
}  // namespace hal
