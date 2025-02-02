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
#include "std_msgs/msg/header.hpp"
#include "hal_motor_control_interfaces/msg/hal_motor_control_encoders.hpp"
#include "hal_motor_control_interfaces/msg/hal_motor_control_command.hpp"

namespace hal
{
namespace pose_manager
{

#define EC_PER_NS_TO_M_PER_S 19231  // 52EC = 1mm

using OdometryMsg_t = nav_msgs::msg::Odometry;
using PoseMsg_t = geometry_msgs::msg::PoseWithCovariance;
using TwistMsg_t = geometry_msgs::msg::TwistWithCovariance;
using HeaderMsg_t = std_msgs::msg::Header;
using HalMotorControlEncodersMsg_t = hal_motor_control_interfaces::msg::HalMotorControlEncoders;
using HalMotorControlCommandMsg_t = hal_motor_control_interfaces::msg::HalMotorControlCommand;

struct EncodersCount
{
  int32_t rightCurrrent;
  int32_t rightPrevious;
  int32_t leftCurrrent;
  int32_t leftPrevious;
  uint32_t timestampNs;
};

struct WheelsVelocity
{
  double right;
  double left;
};

class HalPoseManager : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp_lifecycle::LifecyclePublisher<OdometryMsg_t>::SharedPtr odometryPublisher;
  rclcpp_lifecycle::LifecyclePublisher<HalMotorControlCommandMsg_t>::SharedPtr
    wheelsVelocityCmdPublisher;
  rclcpp::Subscription<TwistMsg_t>::SharedPtr twistSubscriber;
  rclcpp::Subscription<HalMotorControlEncodersMsg_t>::SharedPtr motorsECSubscriber;

  EncodersCount encodersCount;
  WheelsVelocity wheelsVelocity;

public:
  HalPoseManager();
  ~HalPoseManager() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void computeAndPublishwheelsVelocityCmd(const TwistMsg_t & msg);
  void computeAndPublishOdometry(const HalMotorControlEncodersMsg_t & msg);
};

}  // namespace pose_manager
}  // namespace hal

#endif  // HAL_POSE_MANAGER_HPP_
