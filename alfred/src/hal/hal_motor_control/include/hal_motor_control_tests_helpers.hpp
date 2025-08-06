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

#ifndef HAL_MOTOR_CONTROL_TESTS_HELPERS_HPP_
#define HAL_MOTOR_CONTROL_TESTS_HELPERS_HPP_

#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hal_motor_control_commonDefinitions.hpp"

#include "pigpiod_if2.h" // NOLINT

namespace hal
{
namespace motor
{
namespace test
{

#define INPUT 0
#define OUTPUT 1

class HalPigpioDummyNode : public rclcpp::Node
{
public:
  HalPigpioDummyNode();
  ~HalPigpioDummyNode();

  int piHandle;
  int32_t callbackId;

  rclcpp::Service<HalPigpioSetInputMode_t>::SharedPtr setInputModeService;
  rclcpp::Service<HalPigpioSetPullUp_t>::SharedPtr setPullUpService;
  rclcpp::Service<HalPigpioSetOutputMode_t>::SharedPtr setOutputModeService;
  rclcpp::Service<HalPigpioSetPwmDutycycle_t>::SharedPtr setPwmDutycycleService;
  rclcpp::Service<HalPigpioSetPwmFrequency_t>::SharedPtr setPwmFrequencyService;
  rclcpp::Service<HalPigpioSetEncoderCallback_t>::SharedPtr setEncoderCallbackService;

  void setInputMode(
    const std::shared_ptr<HalPigpioSetInputMode_t::Request> request,
    std::shared_ptr<HalPigpioSetInputMode_t::Response> response);
  void setPullUp(
    const std::shared_ptr<HalPigpioSetPullUp_t::Request> request,
    std::shared_ptr<HalPigpioSetPullUp_t::Response> response);
  void setOutputMode(
    const std::shared_ptr<HalPigpioSetOutputMode_t::Request> request,
    std::shared_ptr<HalPigpioSetOutputMode_t::Response> response);
  void setPwmDutycycle(
    const std::shared_ptr<HalPigpioSetPwmDutycycle_t::Request> request,
    std::shared_ptr<HalPigpioSetPwmDutycycle_t::Response> response);
  void setPwmFrequency(
    const std::shared_ptr<HalPigpioSetPwmFrequency_t::Request> request,
    std::shared_ptr<HalPigpioSetPwmFrequency_t::Response> response);
  void setEncoderCallback(
    const std::shared_ptr<HalPigpioSetEncoderCallback_t::Request> request,
    std::shared_ptr<HalPigpioSetEncoderCallback_t::Response> response);
};

}  // namespace test
}  // namespace motor
}  // namespace hal

#endif  // HAL_MOTOR_CONTROL_TESTS_HELPERS_HPP_
