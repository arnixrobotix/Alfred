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

#include "hal_motor_control_tests_helpers.hpp"

using namespace std::placeholders;

namespace hal
{
namespace motor
{
namespace test
{

HalPigpioDummyNode::HalPigpioDummyNode()
: rclcpp::Node{"hal_pigpio_dummy_node"},
  piHandle{pigpio_start(NULL, NULL)},
  callbackId{pigif_bad_callback},
  setInputModeService{this->create_service<HalPigpioSetInputMode_t>(
      "hal_pigpioSetInputMode", std::bind(&HalPigpioDummyNode::setInputMode, this, _1, _2))},
  setPullUpService{this->create_service<HalPigpioSetPullUp_t>(
      "hal_pigpioSetPullUp", std::bind(&HalPigpioDummyNode::setPullUp, this, _1, _2))},
  setOutputModeService{this->create_service<HalPigpioSetOutputMode_t>(
      "hal_pigpioSetOutputMode", std::bind(&HalPigpioDummyNode::setOutputMode, this, _1, _2))},
  setPwmDutycycleService{this->create_service<HalPigpioSetPwmDutycycle_t>(
      "hal_pigpioSetPwmDutycycle", std::bind(&HalPigpioDummyNode::setPwmDutycycle, this, _1, _2))},
  setPwmFrequencyService{this->create_service<HalPigpioSetPwmFrequency_t>(
      "hal_pigpioSetPwmFrequency", std::bind(&HalPigpioDummyNode::setPwmFrequency, this, _1, _2))},
  setEncoderCallbackService{this->create_service<HalPigpioSetEncoderCallback_t>(
      "hal_pigpioSetEncoderCallback",
      std::bind(&HalPigpioDummyNode::setEncoderCallback, this, _1, _2))}
{
}

HalPigpioDummyNode::~HalPigpioDummyNode()
{
  if (callbackId >= 0) {
    callback_cancel(callbackId);
  }

  pigpio_stop(piHandle);
}

void HalPigpioDummyNode::setInputMode(
  const std::shared_ptr<HalPigpioSetInputMode_t::Request> request,
  std::shared_ptr<HalPigpioSetInputMode_t::Response> response)
{
  if (set_mode(piHandle, request->gpio_id, PI_INPUT) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void HalPigpioDummyNode::setPullUp(
  const std::shared_ptr<HalPigpioSetPullUp_t::Request> request,
  std::shared_ptr<HalPigpioSetPullUp_t::Response> response)
{
  if (set_pull_up_down(piHandle, request->gpio_id, PI_PUD_UP) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void HalPigpioDummyNode::setOutputMode(
  const std::shared_ptr<HalPigpioSetOutputMode_t::Request> request,
  std::shared_ptr<HalPigpioSetOutputMode_t::Response> response)
{
  if (set_mode(piHandle, request->gpio_id, PI_OUTPUT) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void HalPigpioDummyNode::setPwmDutycycle(
  const std::shared_ptr<HalPigpioSetPwmDutycycle_t::Request> request,
  std::shared_ptr<HalPigpioSetPwmDutycycle_t::Response> response)
{
  if (set_PWM_dutycycle(piHandle, request->gpio_id, request->dutycycle) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void HalPigpioDummyNode::setPwmFrequency(
  const std::shared_ptr<HalPigpioSetPwmFrequency_t::Request> request,
  std::shared_ptr<HalPigpioSetPwmFrequency_t::Response> response)
{
  if (set_PWM_frequency(piHandle, request->gpio_id, request->frequency) == 0) {
    response->has_succeeded = true;
  } else {
    response->has_succeeded = false;
  }
}

void gpioEncoderEdgeChangeCallback(
  int handle, unsigned gpioId, unsigned edgeChangeType,
  uint32_t timeSinceBoot_us)
{
  (void)handle;
  (void)edgeChangeType;
  (void)timeSinceBoot_us;
}

void HalPigpioDummyNode::setEncoderCallback(
  const std::shared_ptr<HalPigpioSetEncoderCallback_t::Request> request,
  std::shared_ptr<HalPigpioSetEncoderCallback_t::Response> response)
{
  response->callback_id = callback(
    piHandle, request->gpio_id, request->edge_change_type, gpioEncoderEdgeChangeCallback);
  if (response->callback_id >= 0) {
    response->has_succeeded = true;
    callbackId = response->callback_id;
  } else {
    response->has_succeeded = false;
  }
}

}  // namespace test
}  // namespace motor
}  // namespace hal
