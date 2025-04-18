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

#include "hal_pigpio.hpp"

namespace hal
{
namespace pigpio
{

using namespace std::chrono_literals;
using namespace std::placeholders;

Pigpio::Pigpio()
: rclcpp_lifecycle::LifecycleNode{"hal_pigpio_node"},
  pigpioHandle{PI_NO_HANDLE},
  i2cHandle{PI_NO_HANDLE},
  quaternion_{0.0, 0.0, 0.0, 0.0},
  timestamp{rclcpp::Clock().now()},
  prev_quaternion{0.0, 0.0, 0.0, 0.0},
  prev_timestamp{rclcpp::Clock().now()},
  isImuReady{false},
  callbackList{},
  motors{}
{
}

LifecycleCallbackReturn_t Pigpio::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  pigpioHandle = pigpio_start(NULL, NULL);
  if (pigpioHandle < 0) {
    RCLCPP_ERROR(get_logger(), "Pigpio daemon not running!");
    return LifecycleCallbackReturn_t::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Pigpio handle: %d.", pigpioHandle);

  i2cOpenService = this->create_service<HalPigpioI2cOpen_t>(
    "hal_pigpioI2cOpen", std::bind(&Pigpio::i2cOpen, this, _1, _2));
  i2cCloseService = this->create_service<HalPigpioI2cClose_t>(
    "hal_pigpioI2cClose", std::bind(&Pigpio::i2cClose, this, _1, _2));
  i2cReadByteDataService =
    this->create_service<HalPigpioI2cReadByteData_t>(
    "hal_pigpioI2cReadByteData", std::bind(&Pigpio::i2cReadByteData, this, _1, _2));
  i2cReadWordDataService =
    this->create_service<HalPigpioI2cReadWordData_t>(
    "hal_pigpioI2cReadWordData", std::bind(&Pigpio::i2cReadWordData, this, _1, _2));
  i2cReadBlockDataService =
    this->create_service<HalPigpioI2cReadBlockData_t>(
    "hal_pigpioI2cReadBlockData", std::bind(&Pigpio::i2cReadBlockData, this, _1, _2));
  i2cWriteByteDataService =
    this->create_service<HalPigpioI2cWriteByteData_t>(
    "hal_pigpioI2cWriteByteData", std::bind(&Pigpio::i2cWriteByteData, this, _1, _2));
  i2cWriteWordDataService =
    this->create_service<HalPigpioI2cWriteWordData_t>(
    "hal_pigpioI2cWriteWordData", std::bind(&Pigpio::i2cWriteWordData, this, _1, _2));
  i2cWriteBlockDataService =
    this->create_service<HalPigpioI2cWriteBlockData_t>(
    "hal_pigpioI2cWriteBlockData", std::bind(&Pigpio::i2cWriteBlockData, this, _1, _2));
  imuReadingService = this->create_service<HalPigpioI2cImuReading_t>(
    "hal_pigpioI2cImuReading", std::bind(&Pigpio::i2cImuReading, this, _1, _2));
  setInputModeService = this->create_service<HalPigpioSetInputMode_t>(
    "hal_pigpioSetInputMode", std::bind(&Pigpio::setInputMode, this, _1, _2));
  setOutputModeService = this->create_service<HalPigpioSetOutputMode_t>(
    "hal_pigpioSetOutputMode", std::bind(&Pigpio::setOutputMode, this, _1, _2));
  setPullUpService = this->create_service<HalPigpioSetPullUp_t>(
    "hal_pigpioSetPullUp", std::bind(&Pigpio::setPullUp, this, _1, _2));
  setPullDownService = this->create_service<HalPigpioSetPullDown_t>(
    "hal_pigpioSetPullDown", std::bind(&Pigpio::setPullDown, this, _1, _2));
  clearResistorService = this->create_service<HalPigpioClearResistor_t>(
    "hal_pigpioClearResistor", std::bind(&Pigpio::clearResistor, this, _1, _2));
  getModeService = this->create_service<HalPigpioGetMode_t>(
    "hal_pigpioGetMode", std::bind(&Pigpio::getMode, this, _1, _2));
  readGpioService = this->create_service<HalPigpioReadGpio_t>(
    "hal_pigpioReadGpio", std::bind(&Pigpio::readGpio, this, _1, _2));
  setCallbackService = this->create_service<HalPigpioSetCallback_t>(
    "hal_pigpioSetCallback", std::bind(&Pigpio::setCallback, this, _1, _2));
  setEncoderCallbackService =
    this->create_service<HalPigpioSetEncoderCallback_t>(
    "hal_pigpioSetEncoderCallback", std::bind(&Pigpio::setEncoderCallback, this, _1, _2));
  setPwmDutycycleService =
    this->create_service<HalPigpioSetPwmDutycycle_t>(
    "hal_pigpioSetPwmDutycycle", std::bind(&Pigpio::setPwmDutycycle, this, _1, _2));
  setPwmFrequencyService =
    this->create_service<HalPigpioSetPwmFrequency_t>(
    "hal_pigpioSetPwmFrequency", std::bind(&Pigpio::setPwmFrequency, this, _1, _2));
  setGpioHighService = this->create_service<HalPigpioSetGpioHigh_t>(
    "hal_pigpioSetGpioHigh", std::bind(&Pigpio::setGpioHigh, this, _1, _2));
  setGpioLowService = this->create_service<HalPigpioSetGpioLow_t>(
    "hal_pigpioSetGpioLow", std::bind(&Pigpio::setGpioLow, this, _1, _2));
  sendTriggerPulseService =
    this->create_service<HalPigpioSendTriggerPulse_t>(
    "hal_pigpioSendTriggerPulse", std::bind(&Pigpio::sendTriggerPulse, this, _1, _2));

  gpioEdgeChangePub = this->create_publisher<HalPigpioEdgeChangeMsg_t>(
    "gpioEdgeChange", 10);
  gpioEncoderCountPub = this->create_publisher<HalPigpioEncoderCountMsg_t>(
    "hal_pigpioEncoderCount", 10);
  imuPublisher = this->create_publisher<HalPigpioImuMsg_t>("hal_pigpioImu", 10);

  readImuDataAndPublishMessageTimer =
    create_wall_timer(10ms, std::bind(&Pigpio::readImuDataAndPublishMessage, this));
  encoderCountTimer = create_wall_timer(10ms, std::bind(&Pigpio::publishEncoderCount, this));

  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Pigpio::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  gpioEdgeChangePub->on_activate();
  gpioEncoderCountPub->on_activate();
  imuPublisher->on_activate();

  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Pigpio::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  gpioEdgeChangePub->on_deactivate();
  gpioEncoderCountPub->on_deactivate();
  imuPublisher->on_deactivate();

  isImuReady = false;
  quaternion_ = {0.0, 0.0, 0.0, 0.0};

  i2cHandle = PI_NO_HANDLE;

  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Pigpio::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  for (uint callbackId : callbackList) {
    callback_cancel(callbackId);
  }
  callbackList.clear();

  motors.clear();

  gpioEdgeChangePub.reset();
  gpioEncoderCountPub.reset();
  imuPublisher.reset();

  readImuDataAndPublishMessageTimer.reset();
  encoderCountTimer.reset();

  if (pigpioHandle >= 0) {
    RCLCPP_INFO(get_logger(), "Releasing pigpio daemon.");
    pigpio_stop(pigpioHandle);
    pigpioHandle = PI_NO_HANDLE;
  }

  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Pigpio::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  for (uint callbackId : callbackList) {
    callback_cancel(callbackId);
  }
  callbackList.clear();

  motors.clear();

  gpioEdgeChangePub.reset();
  gpioEncoderCountPub.reset();
  imuPublisher.reset();

  readImuDataAndPublishMessageTimer.reset();
  encoderCountTimer.reset();

  if (pigpioHandle >= 0) {
    RCLCPP_INFO(get_logger(), "Releasing pigpio daemon.");
    pigpio_stop(pigpioHandle);
    pigpioHandle = PI_NO_HANDLE;
  }

  RCLCPP_INFO(get_logger(), "Node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Pigpio::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

}  // namespace pigpio
}  // namespace hal
