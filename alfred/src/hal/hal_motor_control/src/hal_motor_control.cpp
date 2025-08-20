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

#include "hal_motor_control.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace hal
{
namespace motor
{
namespace control
{

MotorControl::MotorControl()
: rclcpp_lifecycle::LifecycleNode{"hal_motorControl_node"},
  motorLeft{MOTOR_LEFT_PWM_A_GPIO, MOTOR_LEFT_PWM_B_GPIO,
    MOTOR_LEFT_ENCODER_CH_A_GPIO, MOTOR_LEFT_ENCODER_CH_B_GPIO,
    MOTOR_LEFT},
  motorRight{MOTOR_RIGHT_PWM_A_GPIO, MOTOR_RIGHT_PWM_B_GPIO,
    MOTOR_RIGHT_ENCODER_CH_A_GPIO, MOTOR_RIGHT_ENCODER_CH_B_GPIO,
    MOTOR_RIGHT},
  setInputModeSyncClient{"setInputModeSyncClientMotor_node"},
  setPullUpSyncClient{"setPullUpSyncClientMotor_node"},
  setOutputModeSyncClient{"setOutputModeSyncClientMotor_node"},
  setEncoderCallbackSyncClient{"setEncoderCallbackSyncClientMotor_node"},
  setPwmFrequencySyncClient{"setPwmFrequencySyncClientMotor_node"}
{
}

LifecycleCallbackReturn_t MotorControl::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  setInputModeSyncClient.init("hal_pigpioSetInputMode");
  setPullUpSyncClient.init("hal_pigpioSetPullUp");
  setOutputModeSyncClient.init("hal_pigpioSetOutputMode");
  setEncoderCallbackSyncClient.init("hal_pigpioSetEncoderCallback");
  setPwmFrequencySyncClient.init("hal_pigpioSetPwmFrequency");

  gpioSetPwmDutycycleClient =
    this->create_client<HalPigpioSetPwmDutycycle_t>("hal_pigpioSetPwmDutycycle");

  motorControlPub = this->create_publisher<HalMotorControlEncodersMsg_t>(
    "motorsEncoderCountValue", 10);

  motorControlECSub = this->create_subscription<HalPigpioEncoderCountMsg_t>(
    "hal_pigpioEncoderCount", 10, std::bind(&MotorControl::pigpioEncoderCountCallback, this, _1));
  motorControlCmdSub = this->create_subscription<HalMotorControlCommandMsg_t>(
    "wheelsVelocityCmd", 10, std::bind(&MotorControl::wheelsVelocityCmdCallback, this, _1));

  encoderCountsTimer = create_wall_timer(10ms, std::bind(&MotorControl::publishMessage, this));

  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  activatePublisher();
  configureMotors();

  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  motorControlPub->on_deactivate();

  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  motorControlPub.reset();
  encoderCountsTimer.reset();

  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  motorControlPub.reset();
  encoderCountsTimer.reset();

  RCLCPP_INFO(get_logger(), "Node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t MotorControl::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void MotorControl::activatePublisher(void)
{
  motorControlPub->on_activate();
}

void MotorControl::configureMotors(void)
{
  motorLeft.configureGpios(
    setOutputModeSyncClient, setPullUpSyncClient, setInputModeSyncClient,
    setEncoderCallbackSyncClient, setPwmFrequencySyncClient);
  motorRight.configureGpios(
    setOutputModeSyncClient, setPullUpSyncClient, setInputModeSyncClient,
    setEncoderCallbackSyncClient, setPwmFrequencySyncClient);
}

void MotorControl::pigpioEncoderCountCallback(
  const HalPigpioEncoderCountMsg_t & msg)
{
  for (uint8_t index = 0; index < msg.motor_id.size(); ++index) {
    if (msg.motor_id.at(index) == MOTOR_LEFT) {
      motorLeft.setEncoderCount(msg.encoder_count[index]);
    } else if (msg.motor_id.at(index) == MOTOR_RIGHT) {
      motorRight.setEncoderCount(msg.encoder_count[index]);
    } else {
      RCLCPP_ERROR(get_logger(), "Encoder count message received for unknown motor!");
    }
  }
}

void MotorControl::wheelsVelocityCmdCallback(const HalMotorControlCommandMsg_t & msg)
{
  uint16_t leftPwmDutycycle =
    static_cast<uint16_t>(std::abs(msg.motor_left_command) * torque_to_dutycycle);
  if (leftPwmDutycycle >= 256) {
    leftPwmDutycycle = 255;
  }

  uint16_t rightPwmDutycycle =
    static_cast<uint16_t>(std::abs(msg.motor_right_command) * torque_to_dutycycle);
  if (rightPwmDutycycle >= 256) {
    rightPwmDutycycle = 255;
  }

  RCLCPP_INFO(
    get_logger(), "Right dutycycle: %d, left dutycycle: %d", rightPwmDutycycle,
    leftPwmDutycycle);

  auto leftDirection = forward;
  auto rightDirection = forward;

  if (msg.motor_left_command < 0.0) {
    leftDirection = backward;
  }

  if (msg.motor_right_command < 0.0) {
    rightDirection = backward;
  }

  setPwmLeft(static_cast<uint8_t>(leftPwmDutycycle), leftDirection);
  setPwmRight(static_cast<uint8_t>(rightPwmDutycycle), rightDirection);
}

void MotorControl::publishMessage(void)
{
  auto encoderCounts = HalMotorControlEncodersMsg_t();
  auto header = HeaderMsg_t();

  header.stamp = rclcpp::Clock().now();
  header.frame_id = "Wheels";
  encoderCounts.header = header;

  encoderCounts.motor_left_encoder_count = -motorLeft.getEncoderCount();
  encoderCounts.motor_right_encoder_count = motorRight.getEncoderCount();

  motorControlPub->publish(encoderCounts);
}

void MotorControl::setPwmLeft(uint8_t dutycycle, Direction direction)
{
  motorLeft.setPwmDutyCycleAndDirection(gpioSetPwmDutycycleClient, dutycycle, direction);
}

void MotorControl::setPwmRight(uint8_t dutycycle, Direction direction)
{
  motorRight.setPwmDutyCycleAndDirection(gpioSetPwmDutycycleClient, dutycycle, direction);
}

}  // namespace control
}  // namespace motor
}  // namespace hal
