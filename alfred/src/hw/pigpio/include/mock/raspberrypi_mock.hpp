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

#ifndef MOCK__RASPBERRYPI_MOCK_HPP_
#define MOCK__RASPBERRYPI_MOCK_HPP_

#include <cstdint>
#include <map>
#include <utility>
#include <vector>
#include <tuple>
#include <algorithm>
#include <mutex>

#include "gpioDefinitions.hpp"

namespace hw
{
namespace raspberry_pi
{

#define MPU6050_I2C_ADDRESS_LOW 0x67
#define MPU6050_I2C_ADDRESS_HIGH 0x68
#define MPU6050_USER_CONTROL_REGISTER 0x6A
#define MPU6050_FIFO_COUNT_H_REGISTER 0x72
#define MPU6050_FIFO_COUNT_L_REGISTER 0x73
#define MPU6050_FIFO_REGISTER 0x74
#define MPU6050_INTERRUPT_STATUS_REGISTER 0x3A
#define MPU6050_POWER_MANAGEMENT_1_REGISTER 0x6B
#define MPU6050_SIGNAL_PATH_RESET_REGISTER 0x68
#define MPU6050_SAMPLE_RATE_REGISTER 0x19
#define MPU6050_CONFIGURATION_REGISTER 0x1A
#define MPU6050_GYROSCOPE_CONFIGURATION_REGISTER 0x1B
#define MPU6050_ACCELEROMETER_CONFIGURATION_REGISTER 0x1C
#define MPU6050_BANK_SELECTION_REGISTER 0x6D
#define MPU6050_ADDRESS_IN_BANK_REGISTER 0x6E
#define MPU6050_READ_WRITE_REGISTER 0x6F
#define MPU6050_DMP_START_ADDRESS_H_REGISTER 0x70
#define MPU6050_DMP_START_ADDRESS_L_REGISTER 0x71
#define SENSOR_BIAS_MSB_REGISTER 0x11
#define SENSOR_BIAS_LSB_REGISTER 0x12

#define MPU6050_GYROSCOPE_X_OFFSET_MSB_REGISTER 0x13
#define MPU6050_GYROSCOPE_X_OFFSET_LSB_REGISTER 0x14
#define MPU6050_GYROSCOPE_Y_OFFSET_MSB_REGISTER 0x15
#define MPU6050_GYROSCOPE_Y_OFFSET_LSB_REGISTER 0x16
#define MPU6050_GYROSCOPE_Z_OFFSET_MSB_REGISTER 0x17
#define MPU6050_GYROSCOPE_Z_OFFSET_LSB_REGISTER 0x18

#define MPU6050_ACCELEROMETER_X_OFFSET_MSB_REGISTER 0x06
#define MPU6050_ACCELEROMETER_X_OFFSET_LSB_REGISTER 0x07
#define MPU6050_ACCELEROMETER_Y_OFFSET_MSB_REGISTER 0x08
#define MPU6050_ACCELEROMETER_Y_OFFSET_LSB_REGISTER 0x09
#define MPU6050_ACCELEROMETER_Z_OFFSET_MSB_REGISTER 0x0A
#define MPU6050_ACCELEROMETER_Z_OFFSET_LSB_REGISTER 0x0B

#define MPU6050_ACCELEROMETER_OUT_REGISTER 0x3B
#define MPU6050_GYROSCOPE_OUT_REGISTER 0x43

#define MAX_I2C_BUS_ID 1

enum gpioType
{
  input,
  output
};

enum gpioResistor
{
  off,
  pullDown,
  pullUp
};

enum gpioLevel
{
  low,
  high
};

struct gpioPwm
{
  bool isEnabled;
  uint8_t dutycycle;
  uint16_t frequency;
};

enum gpioEdgeChangeType
{
  risingEdge,
  fallingEdge,
  eitherEdge,
  neitherEdge
};

struct gpioCallback
{
  bool isInitialised;
  uint16_t id;
  gpioEdgeChangeType edgeChangeType;
};

struct gpio
{
  gpioType type;
  gpioResistor resistorConfiguration;
  gpioLevel level;
  gpioPwm pwm;
  gpioCallback callback;
};

struct i2cHandle_t
{
  uint32_t handle;
  uint8_t bus;
  uint8_t device;
};

struct i2cRegister_t
{
  uint32_t device;
  uint32_t address;
  uint16_t size;
  std::vector<uint8_t> bytes;
};

class RaspberryPi
{
private:
  std::map<uint8_t, gpio> gpios;
  std::vector<i2cHandle_t> i2cHandles;
  std::vector<i2cRegister_t> i2cRegisters;
  std::mutex mutex;

public:
  RaspberryPi();
  ~RaspberryPi();
  void reset(void);
  // void initRegisters(void);
  void addGpio(gpioId gpioId);
  uint32_t addI2cHandle(uint8_t bus, uint8_t device);
  bool i2cHandleExists(uint8_t bus, uint8_t device);
  bool i2cHandleExists(uint32_t handle);
  bool removeI2cHandle(uint32_t handle);
  bool registerExists(uint32_t address);
  int16_t readRegister(uint32_t i2cHandle, uint32_t address, uint16_t byteIndex);
  bool writeRegister(uint32_t i2cHandle, uint32_t address, std::vector<uint8_t> bytes);
  int setGpioType(gpioId gpioId, gpioType type);
  int setGpioResistor(gpioId gpioId, gpioResistor resistorConfiguration);
  int setGpioLevel(gpioId gpioId, gpioLevel level);
  int setGpioPwm(gpioId gpioId, gpioPwm pwm);
  int setGpioCallback(gpioId gpioId, gpioCallback callback);
  std::tuple<bool, gpioType> getGpioType(gpioId gpioId);
  std::tuple<bool, gpioResistor> getGpioResistor(gpioId gpioId);
  std::tuple<bool, gpioLevel> getGpioLevel(gpioId gpioId);
  std::tuple<bool, gpioPwm> getGpioPwm(gpioId gpioId);
  std::tuple<bool, gpioCallback> getGpioCallback(gpioId gpioId);
};

}  // namespace raspberry_pi
}  // namespace hw

#endif  // MOCK__RASPBERRYPI_MOCK_HPP_
