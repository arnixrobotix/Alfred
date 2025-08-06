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

#ifndef HAL_MOTOR_TESTS_HPP_
#define HAL_MOTOR_TESTS_HPP_

#include <memory>
#include <future>

#include "hal_motor_control_tests_helpers.hpp"
#include "hal_motor.hpp"

namespace hal
{
namespace motor
{
namespace test
{

#define MOTOR_ID_1 0
#define GPIO_PWM_CHANNEL_A_M1 GPIO3
#define GPIO_PWM_CHANNEL_B_M1 GPIO4
#define GPIO_ENCODER_CHANNEL_A_M1 GPIO5
#define GPIO_ENCODER_CHANNEL_B_M1 GPIO6

class MotorNode : public rclcpp::Node
{
public:
  MotorNode();

  Motor motorOk;
};

class MotorCheckerNode : public rclcpp::Node
{
public:
  MotorCheckerNode();
  ~MotorCheckerNode() = default;

  setOutputModeSyncClientNode_t setOutputModeClient;
  setPullUpSyncClientNode_t setPullUpClient;
  setInputModeSyncClientNode_t setInputModeClient;
  setEncoderCallbackSyncClientNode_t setEncoderCallbackClient;
  setPwmFrequencySyncClientNode_t setPwmFrequencyClient;

  rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr setPwmDutycycleClient;
};

/* Test fixture */
class MotorTest : public testing::Test
{
protected:
  std::shared_ptr<HalPigpioDummyNode> pigpioDummy;
  std::shared_ptr<MotorNode> motor;
  std::shared_ptr<MotorCheckerNode> motorChecker;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp()
  {
    pigpioDummy = std::make_shared<HalPigpioDummyNode>();
    motorChecker = std::make_shared<MotorCheckerNode>();
    motor = std::make_shared<MotorNode>();

    executor.add_node(motor);
    executor.add_node(motorChecker);
    executor.add_node(pigpioDummy);

    motorChecker->setInputModeClient.init("hal_pigpioSetInputMode");
    motorChecker->setPullUpClient.init("hal_pigpioSetPullUp");
    motorChecker->setOutputModeClient.init("hal_pigpioSetOutputMode");
    motorChecker->setEncoderCallbackClient.init("hal_pigpioSetEncoderCallback");
    motorChecker->setPwmFrequencyClient.init("hal_pigpioSetPwmFrequency");

    executor.spin_some();
  }

  void TearDown()
  {
    executor.cancel();
    executor.remove_node(motor);
    executor.remove_node(motorChecker);
    executor.remove_node(pigpioDummy);
    pigpioDummy.reset();
    motorChecker.reset();
    motor.reset();
  }
};

}  // namespace test
}  // namespace motor
}  // namespace hal

#endif  // HAL_MOTOR_TESTS_HPP_
