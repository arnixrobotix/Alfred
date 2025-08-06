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

#ifndef HAL_POSE_MANAGER_HPP_
#define HAL_POSE_MANAGER_HPP_

#include <cmath>
#include <array>

#include "common.hpp"

// Services and messages headers (generated)
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"
#include "hal_motor_control_interfaces/msg/hal_motor_control_encoders.hpp"
#include "hal_motor_control_interfaces/msg/hal_motor_control_command.hpp"

namespace hal
{
namespace pose_manager
{

// 1440 EC per motor revolution, reduction ratio of 34 => 1440 * 34 = 48960 EC per wheel revolution
constexpr double EncoderCountToRadians = 2.0 * M_PI / 48960.0;
constexpr double wheelRadius_m = 0.0375;
constexpr double robotWidth_m = 0.15;
constexpr double encoderCountPerMsToRadPerS = EncoderCountToRadians * 1000000.0;

using ImuDataMsg_t = sensor_msgs::msg::Imu;
using OdometryMsg_t = nav_msgs::msg::Odometry;
using QuaternionMsg_t = geometry_msgs::msg::Quaternion;
using Vector3Msg_t = geometry_msgs::msg::Vector3;
using PointMsg_t = geometry_msgs::msg::Point;
using PoseMsg_t = geometry_msgs::msg::PoseWithCovariance;
using TwistMsg_t = geometry_msgs::msg::TwistWithCovariance;
using HeaderMsg_t = std_msgs::msg::Header;
using HalMotorControlEncodersMsg_t = hal_motor_control_interfaces::msg::HalMotorControlEncoders;
using HalMotorControlCommandMsg_t = hal_motor_control_interfaces::msg::HalMotorControlCommand;

struct EncodersCount
{
  int32_t right;
  int32_t left;
};

struct WheelsVelocity
{
  double right;
  double left;
};

struct Point
{
  double x;
  double y;
  double z;
};

class HalPoseManager : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp_lifecycle::LifecyclePublisher<OdometryMsg_t>::SharedPtr odometryPublisher;
  rclcpp_lifecycle::LifecyclePublisher<HalMotorControlCommandMsg_t>::SharedPtr
    wheelsVelocityCmdPublisher;
  rclcpp::Subscription<TwistMsg_t>::SharedPtr twistSubscriber;
  rclcpp::Subscription<HalMotorControlEncodersMsg_t>::SharedPtr motorsECSubscriber;
  rclcpp::Subscription<ImuDataMsg_t>::SharedPtr imuSubscriber;
  rclcpp::Subscription<PointMsg_t>::SharedPtr positionSubscriber;

  EncodersCount prevEncoderCount;
  Point prevPosition;
  uint64_t prevTimestampNs;
  WheelsVelocity wheelsVelocity;
  QuaternionMsg_t orientation;
  Vector3Msg_t angularVelocity;
  double heading;

public:
  HalPoseManager();
  ~HalPoseManager() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void computeAndPublishwheelsVelocityCmd(const PointMsg_t & msg);
  void publishOdometry(const HalMotorControlEncodersMsg_t & msg);
  void imuDataReader(const ImuDataMsg_t & msg);
  void computePosition(PointMsg_t & position, const HalMotorControlEncodersMsg_t & encoderMessage);
  void computeLinearVelocities(
    const PointMsg_t & position, TwistMsg_t & twist,
    const HalMotorControlEncodersMsg_t & encoderMessage);
};

}  // namespace pose_manager
}  // namespace hal

#endif  // HAL_POSE_MANAGER_HPP_
