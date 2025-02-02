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

#include "mock/raspberrypi_mock.hpp"

namespace hw
{
namespace raspberry_pi
{

RaspberryPi::RaspberryPi()
: gpios{},
  i2cHandles{},
  i2cRegisters{{MPU6050_I2C_ADDRESS_HIGH, 0x10, 1, {}}, {MPU6050_I2C_ADDRESS_HIGH, 0x20, 2, {}},
    {MPU6050_I2C_ADDRESS_LOW, 0x10, 1, {}}, {MPU6050_I2C_ADDRESS_LOW, 0x20, 4, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_USER_CONTROL_REGISTER, 1, {0}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_FIFO_COUNT_H_REGISTER, 1, {0}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_FIFO_COUNT_L_REGISTER, 1, {16}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_INTERRUPT_STATUS_REGISTER, 1, {0}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_FIFO_REGISTER, 16, std::vector<uint8_t>(16, 0)},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_POWER_MANAGEMENT_1_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_SIGNAL_PATH_RESET_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_SAMPLE_RATE_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_CONFIGURATION_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_GYROSCOPE_CONFIGURATION_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_ACCELEROMETER_CONFIGURATION_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_BANK_SELECTION_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_ADDRESS_IN_BANK_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_READ_WRITE_REGISTER, 16, std::vector<uint8_t>(16, 0)},
    {MPU6050_I2C_ADDRESS_HIGH, SENSOR_BIAS_MSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, SENSOR_BIAS_LSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_ACCELEROMETER_X_OFFSET_MSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_ACCELEROMETER_X_OFFSET_LSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_ACCELEROMETER_Y_OFFSET_MSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_ACCELEROMETER_Y_OFFSET_LSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_ACCELEROMETER_Z_OFFSET_MSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_ACCELEROMETER_Z_OFFSET_LSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_GYROSCOPE_X_OFFSET_MSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_GYROSCOPE_X_OFFSET_LSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_GYROSCOPE_Y_OFFSET_MSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_GYROSCOPE_Y_OFFSET_LSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_GYROSCOPE_Z_OFFSET_MSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_GYROSCOPE_Z_OFFSET_LSB_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_ACCELEROMETER_OUT_REGISTER, 6, std::vector<uint8_t>(6, 0)},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_GYROSCOPE_OUT_REGISTER, 6, std::vector<uint8_t>(6, 0)},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_DMP_START_ADDRESS_H_REGISTER, 1, {}},
    {MPU6050_I2C_ADDRESS_HIGH, MPU6050_DMP_START_ADDRESS_L_REGISTER, 1, {}}
  }
{
}

RaspberryPi::~RaspberryPi()
{
  gpios = {};
  i2cHandles = {};
  i2cRegisters = {};
}

void RaspberryPi::reset(void)
{
  std::lock_guard<std::mutex> guard(mutex);
  gpios = {};
  i2cHandles = {};
  for (i2cRegister_t i2cRegister : i2cRegisters) {
    i2cRegister.bytes.clear();
  }
}

void RaspberryPi::addGpio(gpioId gpioId)
{
  gpioPwm gpioPwm({false, 0, 0});
  gpioCallback gpioCallback({false, 0, neitherEdge});
  gpio newGpio({gpioType::input, gpioResistor::off, gpioLevel::low, gpioPwm, gpioCallback});

  std::lock_guard<std::mutex> guard(mutex);
  gpios.insert({gpioId, newGpio});
}

uint32_t RaspberryPi::addI2cHandle(uint8_t bus, uint8_t device)
{
  uint32_t newHandle = 0;

  if (i2cHandles.size() != 0) {
    newHandle = i2cHandles.back().handle + 1;
  }

  std::lock_guard<std::mutex> guard(mutex);
  i2cHandles.push_back({static_cast<uint32_t>(newHandle), bus, device});

  return newHandle;
}

bool RaspberryPi::i2cHandleExists(uint8_t bus, uint8_t device)
{
  for (int index = 0; index < i2cHandles.size(); ++index) {
    if ((i2cHandles.at(index).bus == bus) &&
      (i2cHandles.at(index).device == device))
    {
      return true;
    }
  }

  return false;
}

bool RaspberryPi::i2cHandleExists(uint32_t handle)
{
  for (int index = 0; index < i2cHandles.size(); ++index) {
    if (i2cHandles.at(index).handle == handle) {
      return true;
    }
  }

  return false;
}

bool RaspberryPi::removeI2cHandle(uint32_t handle)
{
  std::lock_guard<std::mutex> guard(mutex);
  for (int index = 0; index < i2cHandles.size(); ++index) {
    if (i2cHandles.at(index).handle == handle) {
      i2cHandles.erase(i2cHandles.begin() + index);
      return true;
    }
  }

  return false;
}

bool RaspberryPi::registerExists(uint32_t address)
{
  for (int index = 0; index < i2cRegisters.size(); ++index) {
    if (i2cRegisters.at(index).address == address) {
      return true;
    }
  }

  return false;
}

int16_t RaspberryPi::readRegister(uint32_t i2cHandle, uint32_t address, uint16_t byteIndex)
{
  auto handleIndex = std::find_if(
    i2cHandles.begin(),
    i2cHandles.end(),
    [i2cHandle](i2cHandle_t deviceHandle) {
      return deviceHandle.handle == i2cHandle;
    });

  if (handleIndex != i2cHandles.end()) {
    auto index = std::find_if(
      i2cRegisters.begin(),
      i2cRegisters.end(),
      [this, handleIndex, address](i2cRegister_t i2cRegister) {
        return i2cRegister.device == i2cHandles.at(
          handleIndex - i2cHandles.begin()).device &&
        i2cRegister.address == address;
      });

    if ((index != i2cRegisters.end()) &&
      (byteIndex < i2cRegisters.at(index - i2cRegisters.begin()).bytes.size()))
    {
      return i2cRegisters.at(index - i2cRegisters.begin()).bytes.at(byteIndex);
    }
  }

  return -1;
}

bool RaspberryPi::writeRegister(uint32_t i2cHandle, uint32_t address, std::vector<uint8_t> bytes)
{
  auto handleIndex = std::find_if(
    i2cHandles.begin(),
    i2cHandles.end(),
    [i2cHandle](i2cHandle_t deviceHandle) {
      return deviceHandle.handle == i2cHandle;
    });

  if (handleIndex != i2cHandles.end()) {
    auto index = std::find_if(
      i2cRegisters.begin(),
      i2cRegisters.end(),
      [this, handleIndex, address](i2cRegister_t i2cRegister) {
        return i2cRegister.device == i2cHandles.at(
          handleIndex - i2cHandles.begin()).device &&
        i2cRegister.address == address;
      });

    if (index != i2cRegisters.end()) {
      std::lock_guard<std::mutex> guard(mutex);
      i2cRegister_t & deviceRegister = i2cRegisters.at(index - i2cRegisters.begin());

      if (bytes.size() <= deviceRegister.size) {
        deviceRegister.bytes.clear();
        deviceRegister.bytes = bytes;

        return true;
      }
    }
  }
  return false;
}

int RaspberryPi::setGpioType(gpioId gpioId, gpioType type)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    std::lock_guard<std::mutex> guard(mutex);
    gpio->second.type = type;
    return 0;
  }
  return -1;
}

int RaspberryPi::setGpioResistor(gpioId gpioId, gpioResistor resistorConfiguration)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    std::lock_guard<std::mutex> guard(mutex);
    gpio->second.resistorConfiguration = resistorConfiguration;
    return 0;
  }
  return -1;
}

int RaspberryPi::setGpioLevel(gpioId gpioId, gpioLevel level)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    std::lock_guard<std::mutex> guard(mutex);
    gpio->second.level = level;

    if (std::get<1>(getGpioType(gpioId)) == output) {
      return 0;
    } else {
      return 1;
    }
  }
  return -1;
}

int RaspberryPi::setGpioPwm(gpioId gpioId, gpioPwm pwm)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end() && std::get<1>(getGpioType(gpioId)) == output) {
    std::lock_guard<std::mutex> guard(mutex);
    gpio->second.pwm = pwm;
    return 0;
  }
  return -2;
}

int RaspberryPi::setGpioCallback(gpioId gpioId, gpioCallback callback)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    std::lock_guard<std::mutex> guard(mutex);
    gpio->second.callback = callback;
    return gpio->second.callback.id;
  }
  return -1;
}

std::tuple<bool, gpioType> RaspberryPi::getGpioType(gpioId gpioId)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    return std::make_tuple(true, gpio->second.type);
  } else {
    return std::make_tuple(false, gpioType::input);
  }
}

std::tuple<bool, gpioResistor> RaspberryPi::getGpioResistor(gpioId gpioId)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    return std::make_tuple(true, gpio->second.resistorConfiguration);
  } else {
    return std::make_tuple(false, gpioResistor::off);
  }
}

std::tuple<bool, gpioLevel> RaspberryPi::getGpioLevel(gpioId gpioId)
{
  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    return std::make_tuple(true, gpio->second.level);
  } else {
    return std::make_tuple(false, gpioLevel::low);
  }
}

std::tuple<bool, gpioPwm> RaspberryPi::getGpioPwm(gpioId gpioId)
{
  gpioPwm gpioPwm({false, 0, 0});

  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    return std::make_tuple(true, gpio->second.pwm);
  } else {
    return std::make_tuple(false, gpioPwm);
  }
}

std::tuple<bool, gpioCallback> RaspberryPi::getGpioCallback(gpioId gpioId)
{
  gpioCallback gpioCallback({false, 0, neitherEdge});

  auto gpio = gpios.find(gpioId);
  if (gpio != gpios.end()) {
    return std::make_tuple(true, gpio->second.callback);
  } else {
    return std::make_tuple(false, gpioCallback);
  }
}

}  // namespace raspberry_pi
}  // namespace hw
