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

#include "hal_motor.hpp"

namespace hal
{
namespace motor
{

Motor::Motor(
  uint8_t gpioPwmChannelA, uint8_t gpioPwmChannelB,
  uint8_t gpioEncoderChannelA, uint8_t gpioEncoderChannelB,
  uint8_t motorId)
: encoder{.channelA{.gpio = gpioEncoderChannelA},
    .channelB{.gpio = gpioEncoderChannelB},
    .encoderCount = 0},
  pwmA{.gpio = gpioPwmChannelA, .dutycycle = 0},
  pwmB{.gpio = gpioPwmChannelB, .dutycycle = 0},
  id{motorId}
{
}

void Motor::configureGpios(
  setOutputModeSyncClientNode_t gpioSetOutputModeClient,
  setPullUpSyncClientNode_t gpioSetPullUpClient,
  setInputModeSyncClientNode_t gpioSetInputModeClient,
  setEncoderCallbackSyncClientNode_t gpioSetEncoderCallbackClient,
  setPwmFrequencySyncClientNode_t gpioSetPwmFrequencyClient)
{
  auto setInputModeEncoderChARequest = std::make_shared<HalPigpioSetInputMode_t::Request>();
  auto setInputModeEncoderChBRequest = std::make_shared<HalPigpioSetInputMode_t::Request>();
  auto setPullUpEncoderChARequest = std::make_shared<HalPigpioSetPullUp_t::Request>();
  auto setPullUpEncoderChBRequest = std::make_shared<HalPigpioSetPullUp_t::Request>();
  auto setEncoderCallbackChARequest = std::make_shared<HalPigpioSetEncoderCallback_t::Request>();
  auto setEncoderCallbackChBRequest = std::make_shared<HalPigpioSetEncoderCallback_t::Request>();
  auto setOutputModePwmChARequest = std::make_shared<HalPigpioSetOutputMode_t::Request>();
  auto setOutputModePwmChBRequest = std::make_shared<HalPigpioSetOutputMode_t::Request>();
  auto setPwmFrequencyPwmChARequest = std::make_shared<HalPigpioSetPwmFrequency_t::Request>();
  auto setPwmFrequencyPwmChBRequest = std::make_shared<HalPigpioSetPwmFrequency_t::Request>();

  /* Encoder channel A */
  setInputModeEncoderChARequest->gpio_id = encoder.channelA.gpio;
  if (!gpioSetInputModeClient.sendRequest(setInputModeEncoderChARequest)->has_succeeded) {
    RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setInputMode");
  }

  setPullUpEncoderChARequest->gpio_id = encoder.channelA.gpio;
  if (!gpioSetPullUpClient.sendRequest(setPullUpEncoderChARequest)->has_succeeded) {
    RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setPullUp");
  }

  setEncoderCallbackChARequest->gpio_id = encoder.channelA.gpio;
  setEncoderCallbackChARequest->edge_change_type =
    static_cast<uint8_t>(EdgeChangeConfiguration::asEitherEdge);
  setEncoderCallbackChARequest->motor_id = id;
  setEncoderCallbackChARequest->channel = static_cast<uint8_t>(EncoderChannel::channelA);
  if (!gpioSetEncoderCallbackClient.sendRequest(setEncoderCallbackChARequest)->has_succeeded) {
    RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setEncoderCallback");
  }

  /* Encoder channel B */
  setInputModeEncoderChBRequest->gpio_id = encoder.channelB.gpio;
  if (!gpioSetInputModeClient.sendRequest(setInputModeEncoderChBRequest)->has_succeeded) {
    RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setInputMode");
  }

  setPullUpEncoderChBRequest->gpio_id = encoder.channelB.gpio;
  if (!gpioSetPullUpClient.sendRequest(setPullUpEncoderChBRequest)->has_succeeded) {
    RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setPullUp");
  }

  setEncoderCallbackChBRequest->gpio_id = encoder.channelB.gpio;
  setEncoderCallbackChBRequest->edge_change_type =
    static_cast<uint8_t>(EdgeChangeConfiguration::asEitherEdge);
  setEncoderCallbackChBRequest->motor_id = id;
  setEncoderCallbackChBRequest->channel = static_cast<uint8_t>(EncoderChannel::channelB);
  if (!gpioSetEncoderCallbackClient.sendRequest(setEncoderCallbackChBRequest)->has_succeeded) {
    RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setEncoderCallback");
  }

  /* PWM channel A */
  setOutputModePwmChARequest->gpio_id = pwmA.gpio;
  if (!gpioSetOutputModeClient.sendRequest(setOutputModePwmChARequest)->has_succeeded) {
    RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setOutputMode");
  }

  setPwmFrequencyPwmChARequest->gpio_id = pwmA.gpio;
  setPwmFrequencyPwmChARequest->frequency = MOTOR_PWM_FREQUENCY;
  if (!gpioSetPwmFrequencyClient.sendRequest(setPwmFrequencyPwmChARequest)->has_succeeded) {
    RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setPwmFrequency");
  }

  /* PWM channel B */
  setOutputModePwmChBRequest->gpio_id = pwmB.gpio;
  if (!gpioSetOutputModeClient.sendRequest(setOutputModePwmChBRequest)->has_succeeded) {
    RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setOutputMode");
  }

  setPwmFrequencyPwmChBRequest->gpio_id = pwmB.gpio;
  setPwmFrequencyPwmChBRequest->frequency = MOTOR_PWM_FREQUENCY;
  if (!gpioSetPwmFrequencyClient.sendRequest(setPwmFrequencyPwmChBRequest)->has_succeeded) {
    RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setPwmFrequency");
  }
}

int32_t Motor::getEncoderCount(void)
{
  return encoder.encoderCount;
}

void Motor::setEncoderCount(int32_t count)
{
  encoder.encoderCount = count;
}

void Motor::setPwmDutyCycleAndDirection(
  rclcpp::Client<HalPigpioSetPwmDutycycle_t>::SharedPtr gpioSetPwmDutycycleClient,
  uint8_t dutycycle,
  bool isDirectionForward)
{
  auto setPwmDutycyclePwmChARequest = std::make_shared<HalPigpioSetPwmDutycycle_t::Request>();
  auto setPwmDutycyclePwmChBRequest = std::make_shared<HalPigpioSetPwmDutycycle_t::Request>();

  auto setPwmDutycycleCallback = [this](SetPwmDutycycleFuture_t future)
    {
      if (!future.get()->has_succeeded) {
        RCLCPP_ERROR(rclcpp::get_logger("Motors"), "Failed to call service setPwmDutycycle");
      }
    };

  pwmB.dutycycle = dutycycle;
  pwmA.dutycycle = dutycycle;

  if (isDirectionForward) {
    setPwmDutycyclePwmChARequest->gpio_id = pwmA.gpio;
    setPwmDutycyclePwmChARequest->dutycycle = pwmA.dutycycle;
    auto setPwmDutycyclePwmChAFuture = gpioSetPwmDutycycleClient->async_send_request(
      setPwmDutycyclePwmChARequest, setPwmDutycycleCallback);

    setPwmDutycyclePwmChBRequest->gpio_id = pwmB.gpio;
    setPwmDutycyclePwmChBRequest->dutycycle = 0;
    auto setPwmDutycyclePwmChBFuture = gpioSetPwmDutycycleClient->async_send_request(
      setPwmDutycyclePwmChBRequest, setPwmDutycycleCallback);
  } else {
    setPwmDutycyclePwmChBRequest->gpio_id = pwmB.gpio;
    setPwmDutycyclePwmChBRequest->dutycycle = pwmB.dutycycle;
    auto setPwmDutycyclePwmChBFuture = gpioSetPwmDutycycleClient->async_send_request(
      setPwmDutycyclePwmChBRequest, setPwmDutycycleCallback);

    setPwmDutycyclePwmChARequest->gpio_id = pwmA.gpio;
    setPwmDutycyclePwmChARequest->dutycycle = 0;
    auto setPwmDutycyclePwmChAFuture = gpioSetPwmDutycycleClient->async_send_request(
      setPwmDutycyclePwmChARequest, setPwmDutycycleCallback);
  }
}

uint8_t Motor::getId(void)
{
  return id;
}

}  // namespace motor
}  // namespace hal
