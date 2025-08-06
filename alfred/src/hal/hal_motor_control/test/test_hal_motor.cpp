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

#include "hal_motor_tests.hpp"

using namespace std::placeholders;

namespace hal
{
namespace motor
{
namespace test
{

MotorNode::MotorNode()
: rclcpp::Node{"hal_motor_node"},
  motorOk{
    GPIO_PWM_CHANNEL_A_M1, GPIO_PWM_CHANNEL_B_M1,
    GPIO_ENCODER_CHANNEL_A_M1, GPIO_ENCODER_CHANNEL_B_M1,
    MOTOR_ID_1}
{
}

MotorCheckerNode::MotorCheckerNode()
: rclcpp::Node{"hal_motor_checker_node"},
  setPwmDutycycleClient{
    this->create_client<HalPigpioSetPwmDutycycle_t>("hal_pigpioSetPwmDutycycle")},
  setInputModeClient{"setInputModeSyncClientMotorChecker_node"},
  setPullUpClient{"setPullUpSyncClientMotorChecker_node"},
  setOutputModeClient{"setOutputModeSyncClientMotorChecker_node"},
  setEncoderCallbackClient{"setEncoderCallbackSyncClientMotorChecker_node"},
  setPwmFrequencyClient{"setPwmFrequencySyncClientMotorChecker_node"}
{
}

TEST_F(MotorTest, EncoderCountUtils)
{
  const int32_t encoderCount = 1024;
  motor->motorOk.setEncoderCount(encoderCount);

  ASSERT_EQ(motor->motorOk.getEncoderCount(), encoderCount);
}

TEST_F(MotorTest, GetId)
{
  ASSERT_EQ(motor->motorOk.getId(), MOTOR_ID_1);
}

TEST_F(MotorTest, ConfigureGpios)
{
  std::function<void(
      setOutputModeSyncClientNode_t,
      setPullUpSyncClientNode_t,
      setInputModeSyncClientNode_t,
      setEncoderCallbackSyncClientNode_t,
      setPwmFrequencySyncClientNode_t)> configureGpios = std::bind(
    &Motor::configureGpios, motor->motorOk, _1, _2, _3, _4, _5);

  auto future = std::async(
    std::launch::async,
    configureGpios,
    motorChecker->setOutputModeClient,
    motorChecker->setPullUpClient,
    motorChecker->setInputModeClient,
    motorChecker->setEncoderCallbackClient,
    motorChecker->setPwmFrequencyClient);

  auto status = executor.spin_until_future_complete(future);

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, GPIO_ENCODER_CHANNEL_A_M1), INPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, GPIO_ENCODER_CHANNEL_B_M1), INPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, GPIO_PWM_CHANNEL_A_M1), OUTPUT);
    ASSERT_EQ(get_mode(pigpioDummy->piHandle, GPIO_PWM_CHANNEL_B_M1), OUTPUT);
    ASSERT_EQ(get_PWM_frequency(pigpioDummy->piHandle, GPIO_PWM_CHANNEL_A_M1), MOTOR_PWM_FREQUENCY);
    ASSERT_EQ(get_PWM_frequency(pigpioDummy->piHandle, GPIO_PWM_CHANNEL_B_M1), MOTOR_PWM_FREQUENCY);
  } else {
    FAIL() << "Future didn't complete successfully";
  }
}

TEST_F(MotorTest, SetPwmDutyCycleAndDirectionForward)
{
  std::function<void(
      setOutputModeSyncClientNode_t,
      setPullUpSyncClientNode_t,
      setInputModeSyncClientNode_t,
      setEncoderCallbackSyncClientNode_t,
      setPwmFrequencySyncClientNode_t)> configureGpios = std::bind(
    &Motor::configureGpios, motor->motorOk, _1, _2, _3, _4, _5);

  auto future = std::async(
    std::launch::async,
    configureGpios,
    motorChecker->setOutputModeClient,
    motorChecker->setPullUpClient,
    motorChecker->setInputModeClient,
    motorChecker->setEncoderCallbackClient,
    motorChecker->setPwmFrequencyClient);

  auto status = executor.spin_until_future_complete(future);

  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL() << "Future didn't complete successfully";
  }

  motor->motorOk.setPwmDutyCycleAndDirection(motorChecker->setPwmDutycycleClient, 20, forward);

  executor.spin_some();
  executor.spin_some();

  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, GPIO_PWM_CHANNEL_A_M1), 20);
  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, GPIO_PWM_CHANNEL_B_M1), 0);
}

TEST_F(MotorTest, SetPwmDutyCycleAndDirectionBackward)
{
  std::function<void(
      setOutputModeSyncClientNode_t,
      setPullUpSyncClientNode_t,
      setInputModeSyncClientNode_t,
      setEncoderCallbackSyncClientNode_t,
      setPwmFrequencySyncClientNode_t)> configureGpios = std::bind(
    &Motor::configureGpios, motor->motorOk, _1, _2, _3, _4, _5);

  auto future = std::async(
    std::launch::async,
    configureGpios,
    motorChecker->setOutputModeClient,
    motorChecker->setPullUpClient,
    motorChecker->setInputModeClient,
    motorChecker->setEncoderCallbackClient,
    motorChecker->setPwmFrequencyClient);

  auto status = executor.spin_until_future_complete(future);

  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL() << "Future didn't complete successfully";
  }

  motor->motorOk.setPwmDutyCycleAndDirection(motorChecker->setPwmDutycycleClient, 20, backward);

  executor.spin_some();
  executor.spin_some();

  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, GPIO_PWM_CHANNEL_A_M1), 0);
  ASSERT_EQ(get_PWM_dutycycle(pigpioDummy->piHandle, GPIO_PWM_CHANNEL_B_M1), 20);
}

}  // namespace test
}  // namespace motor
}  // namespace hal

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
