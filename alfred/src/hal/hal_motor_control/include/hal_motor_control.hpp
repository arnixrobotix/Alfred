// Copyright (c) 2022 Arnix Robotix
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

#ifndef HAL_MOTOR_CONTROL_HPP_
#define HAL_MOTOR_CONTROL_HPP_

#include "hal_motor_control_commonDefinitions.hpp"
#include "hal_motor.hpp"

namespace hal
{
namespace motor
{

using setInputModeSyncClientNode_t = ServiceNodeSync<HalPigpioSetInputMode_t>;
using setOutputModeSyncClientNode_t = ServiceNodeSync<HalPigpioSetOutputMode_t>;
using setEncoderCallbackSyncClientNode_t = ServiceNodeSync<HalPigpioSetEncoderCallback_t>;
using setPwmFrequencySyncClientNode_t = ServiceNodeSync<HalPigpioSetPwmFrequency_t>;

namespace control
{

class MotorControl : public rclcpp_lifecycle::LifecycleNode
{
private:
  Motor motorLeft;
  Motor motorRight;

  setInputModeSyncClientNode_t setInputModeSyncClient;
  setPullUpSyncClientNode_t setPullUpSyncClient;
  setOutputModeSyncClientNode_t setOutputModeSyncClient;
  setEncoderCallbackSyncClientNode_t setEncoderCallbackSyncClient;
  setPwmFrequencySyncClientNode_t setPwmFrequencySyncClient;

  rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr gpioSetPwmDutycycleClient;

  rclcpp_lifecycle::LifecyclePublisher<HalMotorControlEncodersMsg_t>::SharedPtr motorControlPub;

  rclcpp::Subscription<HalPigpioEncoderCountMsg_t>::SharedPtr motorControlECSub;
  rclcpp::Subscription<HalMotorControlCommandMsg_t>::SharedPtr motorControlCmdSub;

  rclcpp::TimerBase::SharedPtr encoderCountsTimer;

public:
  MotorControl();
  ~MotorControl() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void activatePublisher(void);
  void configureMotors(void);
  void publishMessage(void);
  void pigpioEncoderCountCallback(const HalPigpioEncoderCountMsg_t & msg);
  void wheelsVelocityCmdCallback(const HalMotorControlCommandMsg_t & msg);
  void setPwmLeft(uint8_t dutycycle, Direction direction);
  void setPwmRight(uint8_t dutycycle, Direction direction);
};

}  // namespace control
}  // namespace motor
}  // namespace hal

#endif  // HAL_MOTOR_CONTROL_HPP_
