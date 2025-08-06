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
  prevEncoderCount{.right = 0, .left = 0},
  prevPosition{0.0, 0.0, 0.0},
  prevTimestampNs{0},
  wheelsVelocity{.right = 0.0, .left = 0.0},
  orientation{QuaternionMsg_t()},
  angularVelocity{Vector3Msg_t()},
  heading{0.0}
{
}

LifecycleCallbackReturn_t HalPoseManager::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  odometryPublisher = this->create_publisher<OdometryMsg_t>("odometry", 10);
  wheelsVelocityCmdPublisher = this->create_publisher<HalMotorControlCommandMsg_t>(
    "wheelsVelocityCmd", 10);
  // twistSubscriber = this->create_subscription<TwistMsg_t>(
  //   "cmd_velocity", 10, std::bind(&HalPoseManager::computeAndPublishwheelsVelocityCmd,
  //    this, _1));
  positionSubscriber = this->create_subscription<PointMsg_t>(
    "cmd_position", 10, std::bind(&HalPoseManager::computeAndPublishwheelsVelocityCmd, this, _1));
  motorsECSubscriber = this->create_subscription<HalMotorControlEncodersMsg_t>(
    "motorsEncoderCountValue", 10,
    std::bind(&HalPoseManager::publishOdometry, this, _1));
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

void HalPoseManager::computeAndPublishwheelsVelocityCmd(const PointMsg_t & msg)
{
  auto wheelsVelocityCommandMsg = HalMotorControlCommandMsg_t();

  auto desiredPosition = msg.x;

  auto wheelsVelocityCommand = (desiredPosition - prevPosition.x) * 0.1 - angularVelocity.y * 0.1;
  wheelsVelocityCommandMsg.motor_left_velocity_command = wheelsVelocityCommand;
  wheelsVelocityCommandMsg.motor_right_velocity_command = wheelsVelocityCommand;

  RCLCPP_INFO(
    get_logger(), "Position error: %f Velocity command: %f",
    (desiredPosition - prevPosition.x), wheelsVelocityCommand);

  wheelsVelocityCmdPublisher->publish(wheelsVelocityCommandMsg);
}

void HalPoseManager::imuDataReader(const ImuDataMsg_t & msg)
{
  orientation = msg.orientation;
  angularVelocity = msg.angular_velocity;
}

void HalPoseManager::computePosition(
  PointMsg_t & position,
  const HalMotorControlEncodersMsg_t & encoderMessage)
{
  int32_t encoderCountDeltaLeft = encoderMessage.motor_left_encoder_count - prevEncoderCount.left;
  prevEncoderCount.left = encoderMessage.motor_left_encoder_count;

  int32_t encoderCountDeltaRight = encoderMessage.motor_right_encoder_count -
    prevEncoderCount.right;
  prevEncoderCount.right = encoderMessage.motor_right_encoder_count;

  double angle = wheelRadius_m / robotWidth_m *
    (encoderCountDeltaRight - encoderCountDeltaLeft) * EncoderCountToRadians;
  double distance = wheelRadius_m / 2.0 *
    (encoderCountDeltaRight + encoderCountDeltaLeft) * EncoderCountToRadians;

  heading += angle;

  // Small angles approximation
  double deltaX = distance * std::cos(heading);
  double deltaY = distance * std::sin(heading);

  position.x = prevPosition.x + deltaX;
  position.y = prevPosition.y + deltaY;
}

void HalPoseManager::computeLinearVelocities(
  const PointMsg_t & position, TwistMsg_t & twist,
  const HalMotorControlEncodersMsg_t & encoderMessage)
{
  uint64_t timestampNs = encoderMessage.header.stamp.nanosec + encoderMessage.header.stamp.sec *
    1000000000;
  float timeDeltaS = (prevTimestampNs - timestampNs) / 1000000000.0;
  prevTimestampNs = timestampNs;

  twist.twist.linear.x = (position.x - prevPosition.x) / timeDeltaS;
  twist.twist.linear.y = (position.y - prevPosition.y) / timeDeltaS;
}

void HalPoseManager::publishOdometry(const HalMotorControlEncodersMsg_t & msg)
{
  auto header = HeaderMsg_t();
  auto odometry = OdometryMsg_t();
  auto pose = PoseMsg_t();
  auto twist = TwistMsg_t();
  auto position = PointMsg_t();

  // Robot frame: X axis is oriented towards the front of the robot
  //              Y axis is colinear to the axis of the wheels oriented to the left
  //              Z axis is vertical up
  // This frame is attached to the motors, so only the pendulum movement happens in it,
  // i.e. the rotation of the body around the axis of the motors

  computePosition(position, msg);
  computeLinearVelocities(position, twist, msg);

  prevPosition.x = position.x;
  prevPosition.y = position.y;
  pose.pose.position = std::move(position);

  odometry.twist = std::move(twist);

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
