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

#ifndef HAL_PIGPIO_HPP_
#define HAL_PIGPIO_HPP_

#include <vector>
#include <memory>

#include "common.hpp"

// Pigpio library
#include "pigpiod_if2.h"  // NOLINT

#include "hal_pigpioImu.hpp"
#include "hal_pigpioI2c.hpp"
#include "hal_pigpioInit.hpp"
#include "hal_pigpioInput.hpp"
#include "hal_pigpioOutput.hpp"

namespace hal
{
namespace pigpio
{

using HalPigpioSetInputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetInputMode;
using HalPigpioSetOutputMode_t = hal_pigpio_interfaces::srv::HalPigpioSetOutputMode;
using HalPigpioGetMode_t = hal_pigpio_interfaces::srv::HalPigpioGetMode;
using HalPigpioSetPullUp_t = hal_pigpio_interfaces::srv::HalPigpioSetPullUp;
using HalPigpioSetPullDown_t = hal_pigpio_interfaces::srv::HalPigpioSetPullDown;
using HalPigpioClearResistor_t = hal_pigpio_interfaces::srv::HalPigpioClearResistor;
using HalPigpioSetPwmDutycycle_t = hal_pigpio_interfaces::srv::HalPigpioSetPwmDutycycle;
using HalPigpioSetPwmFrequency_t = hal_pigpio_interfaces::srv::HalPigpioSetPwmFrequency;
using HalPigpioSetGpioHigh_t = hal_pigpio_interfaces::srv::HalPigpioSetGpioHigh;
using HalPigpioSetGpioLow_t = hal_pigpio_interfaces::srv::HalPigpioSetGpioLow;
using HalPigpioSendTriggerPulse_t = hal_pigpio_interfaces::srv::HalPigpioSendTriggerPulse;
using HalPigpioReadGpio_t = hal_pigpio_interfaces::srv::HalPigpioReadGpio;
using HalPigpioSetCallback_t = hal_pigpio_interfaces::srv::HalPigpioSetCallback;
using HalPigpioSetEncoderCallback_t = hal_pigpio_interfaces::srv::HalPigpioSetEncoderCallback;
using HalPigpioEdgeChangeMsg_t = hal_pigpio_interfaces::msg::HalPigpioEdgeChange;
using HalPigpioEncoderCountMsg_t = hal_pigpio_interfaces::msg::HalPigpioEncoderCount;
using HalPigpioI2cOpen_t = hal_pigpio_interfaces::srv::HalPigpioI2cOpen;
using HalPigpioI2cClose_t = hal_pigpio_interfaces::srv::HalPigpioI2cClose;
using HalPigpioI2cReadByteData_t = hal_pigpio_interfaces::srv::HalPigpioI2cReadByteData;
using HalPigpioI2cReadWordData_t = hal_pigpio_interfaces::srv::HalPigpioI2cReadWordData;
using HalPigpioI2cReadBlockData_t = hal_pigpio_interfaces::srv::HalPigpioI2cReadBlockData;
using HalPigpioI2cWriteByteData_t = hal_pigpio_interfaces::srv::HalPigpioI2cWriteByteData;
using HalPigpioI2cWriteWordData_t = hal_pigpio_interfaces::srv::HalPigpioI2cWriteWordData;
using HalPigpioI2cWriteBlockData_t = hal_pigpio_interfaces::srv::HalPigpioI2cWriteBlockData;
using HalPigpioI2cImuReading_t = hal_pigpio_interfaces::srv::HalPigpioI2cImuReading;
using HalPigpioImuMsg_t = sensor_msgs::msg::Imu;
using QuaternionMsg_t = geometry_msgs::msg::Quaternion;
using Vector3Msg_t = geometry_msgs::msg::Vector3;
using HeaderMsg_t = std_msgs::msg::Header;

class Pigpio : public rclcpp_lifecycle::LifecycleNode
{
private:
  int pigpioHandle;
  int32_t i2cHandle;
  Quaternion quaternion_;
  rclcpp::Time timestamp;
  Quaternion prev_quaternion;
  rclcpp::Time prev_timestamp;
  bool isImuReady;
  std::vector<uint> callbackList;
  std::vector<Motor> motors;

  rclcpp::Service<HalPigpioI2cOpen_t>::SharedPtr i2cOpenService;
  rclcpp::Service<HalPigpioI2cClose_t>::SharedPtr i2cCloseService;
  rclcpp::Service<HalPigpioI2cReadByteData_t>::SharedPtr i2cReadByteDataService;
  rclcpp::Service<HalPigpioI2cReadWordData_t>::SharedPtr i2cReadWordDataService;
  rclcpp::Service<HalPigpioI2cReadBlockData_t>::SharedPtr i2cReadBlockDataService;
  rclcpp::Service<HalPigpioI2cWriteByteData_t>::SharedPtr i2cWriteByteDataService;
  rclcpp::Service<HalPigpioI2cWriteWordData_t>::SharedPtr i2cWriteWordDataService;
  rclcpp::Service<HalPigpioI2cWriteBlockData_t>::SharedPtr i2cWriteBlockDataService;
  rclcpp::Service<HalPigpioI2cImuReading_t>::SharedPtr imuReadingService;
  rclcpp::Service<HalPigpioSetInputMode_t>::SharedPtr setInputModeService;
  rclcpp::Service<HalPigpioSetOutputMode_t>::SharedPtr setOutputModeService;
  rclcpp::Service<HalPigpioSetPullUp_t>::SharedPtr setPullUpService;
  rclcpp::Service<HalPigpioSetPullDown_t>::SharedPtr setPullDownService;
  rclcpp::Service<HalPigpioClearResistor_t>::SharedPtr clearResistorService;
  rclcpp::Service<HalPigpioGetMode_t>::SharedPtr getModeService;
  rclcpp::Service<HalPigpioReadGpio_t>::SharedPtr readGpioService;
  rclcpp::Service<HalPigpioSetCallback_t>::SharedPtr setCallbackService;
  rclcpp::Service<HalPigpioSetEncoderCallback_t>::SharedPtr setEncoderCallbackService;
  rclcpp::Service<HalPigpioSetPwmDutycycle_t>::SharedPtr setPwmDutycycleService;
  rclcpp::Service<HalPigpioSetPwmFrequency_t>::SharedPtr setPwmFrequencyService;
  rclcpp::Service<HalPigpioSetGpioHigh_t>::SharedPtr setGpioHighService;
  rclcpp::Service<HalPigpioSetGpioLow_t>::SharedPtr setGpioLowService;
  rclcpp::Service<HalPigpioSendTriggerPulse_t>::SharedPtr sendTriggerPulseService;

  rclcpp_lifecycle::LifecyclePublisher<HalPigpioEdgeChangeMsg_t>::SharedPtr gpioEdgeChangePub;
  rclcpp_lifecycle::LifecyclePublisher<HalPigpioEncoderCountMsg_t>::SharedPtr gpioEncoderCountPub;
  rclcpp_lifecycle::LifecyclePublisher<HalPigpioImuMsg_t>::SharedPtr imuPublisher;

  rclcpp::TimerBase::SharedPtr readImuDataAndPublishMessageTimer;
  rclcpp::TimerBase::SharedPtr encoderCountTimer;

  static void c_gpioEdgeChangeCallback(
    int handle, unsigned gpioId, unsigned edgeChangeType,
    uint32_t timeSinceBoot_us, void * userData);
  static void c_gpioEncoderEdgeChangeCallback(
    int handle, unsigned gpioId, unsigned edgeChangeType,
    uint32_t timeSinceBoot_us, void * userData);

public:
  Pigpio();
  ~Pigpio() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void readQuaternionData(void);
  void computeQuaternion(char (& data)[MPU6050_DMP_FIFO_QUAT_SIZE]);
  void computeAngularVelocities(Vector3Msg_t & angularVelocity);
  void computeLinearAcceleration(Vector3Msg_t & linearAcceleration);
  void publishImuMessage(void);
  void readImuDataAndPublishMessage(void);
  void resetFifo(void);
  uint16_t readFifoCount(void);
  bool isFifoOverflowed(void);
  void publishEncoderCount(void);

  void gpioEdgeChangeCallback(
    int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);
  void gpioEncoderEdgeChangeCallback(
    int handle, unsigned gpioId, unsigned edgeChangeType, uint32_t timeSinceBoot_us);

  MotorDirection computeDirection(
    const EncoderChannel & previousChannel, const EncoderChannel & channel,
    const EdgeChangeType & previousEdgeChange, const EdgeChangeType & edgeChange);

  void i2cOpen(
    const std::shared_ptr<HalPigpioI2cOpen_t::Request> request,
    std::shared_ptr<HalPigpioI2cOpen_t::Response> response);
  void i2cClose(
    const std::shared_ptr<HalPigpioI2cClose_t::Request> request,
    std::shared_ptr<HalPigpioI2cClose_t::Response> response);
  void i2cReadByteData(
    const std::shared_ptr<HalPigpioI2cReadByteData_t::Request> request,
    std::shared_ptr<HalPigpioI2cReadByteData_t::Response> response);
  void i2cReadWordData(
    const std::shared_ptr<HalPigpioI2cReadWordData_t::Request> request,
    std::shared_ptr<HalPigpioI2cReadWordData_t::Response> response);
  void i2cReadBlockData(
    const std::shared_ptr<HalPigpioI2cReadBlockData_t::Request> request,
    std::shared_ptr<HalPigpioI2cReadBlockData_t::Response> response);
  void i2cWriteByteData(
    const std::shared_ptr<HalPigpioI2cWriteByteData_t::Request> request,
    std::shared_ptr<HalPigpioI2cWriteByteData_t::Response> response);
  void i2cWriteWordData(
    const std::shared_ptr<HalPigpioI2cWriteWordData_t::Request> request,
    std::shared_ptr<HalPigpioI2cWriteWordData_t::Response> response);
  void i2cWriteBlockData(
    const std::shared_ptr<HalPigpioI2cWriteBlockData_t::Request> request,
    std::shared_ptr<HalPigpioI2cWriteBlockData_t::Response> response);
  void i2cImuReading(
    const std::shared_ptr<HalPigpioI2cImuReading_t::Request> request,
    std::shared_ptr<HalPigpioI2cImuReading_t::Response> response);
  void getMode(
    const std::shared_ptr<HalPigpioGetMode_t::Request> request,
    std::shared_ptr<HalPigpioGetMode_t::Response> response);
  void setInputMode(
    const std::shared_ptr<HalPigpioSetInputMode_t::Request> request,
    std::shared_ptr<HalPigpioSetInputMode_t::Response> response);
  void setOutputMode(
    const std::shared_ptr<HalPigpioSetOutputMode_t::Request> request,
    std::shared_ptr<HalPigpioSetOutputMode_t::Response> response);
  void setPullUp(
    const std::shared_ptr<HalPigpioSetPullUp_t::Request> request,
    std::shared_ptr<HalPigpioSetPullUp_t::Response> response);
  void setPullDown(
    const std::shared_ptr<HalPigpioSetPullDown_t::Request> request,
    std::shared_ptr<HalPigpioSetPullDown_t::Response> response);
  void clearResistor(
    const std::shared_ptr<HalPigpioClearResistor_t::Request> request,
    std::shared_ptr<HalPigpioClearResistor_t::Response> response);
  void readGpio(
    const std::shared_ptr<HalPigpioReadGpio_t::Request> request,
    std::shared_ptr<HalPigpioReadGpio_t::Response> response);
  void setCallback(
    const std::shared_ptr<HalPigpioSetCallback_t::Request> request,
    std::shared_ptr<HalPigpioSetCallback_t::Response> response);
  void setEncoderCallback(
    const std::shared_ptr<HalPigpioSetEncoderCallback_t::Request> request,
    std::shared_ptr<HalPigpioSetEncoderCallback_t::Response> response);
  void setPwmDutycycle(
    const std::shared_ptr<HalPigpioSetPwmDutycycle_t::Request> request,
    std::shared_ptr<HalPigpioSetPwmDutycycle_t::Response> response);
  void setPwmFrequency(
    const std::shared_ptr<HalPigpioSetPwmFrequency_t::Request> request,
    std::shared_ptr<HalPigpioSetPwmFrequency_t::Response> response);
  void setGpioHigh(
    const std::shared_ptr<HalPigpioSetGpioHigh_t::Request> request,
    std::shared_ptr<HalPigpioSetGpioHigh_t::Response> response);
  void setGpioLow(
    const std::shared_ptr<HalPigpioSetGpioLow_t::Request> request,
    std::shared_ptr<HalPigpioSetGpioLow_t::Response> response);
  void sendTriggerPulse(
    const std::shared_ptr<HalPigpioSendTriggerPulse_t::Request> request,
    std::shared_ptr<HalPigpioSendTriggerPulse_t::Response> response);
};

}  // namespace pigpio
}  // namespace hal

#endif  // HAL_PIGPIO_HPP_
