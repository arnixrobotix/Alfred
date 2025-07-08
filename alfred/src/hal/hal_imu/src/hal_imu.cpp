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

#include "hal_imu.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace hal
{
namespace imu
{

Imu::Imu()
: rclcpp_lifecycle::LifecycleNode{"hal_imu"},
  imuHandle{MPU6050_I2C_NO_HANDLE},
  imuGetHandleSyncClient{"getHandleSyncClientImu_node"},
  i2cReadByteDataSyncClient{"readByteDataSyncClientImu_node"},
  i2cWriteByteDataSyncClient{"writeByteDataSyncClientImu_node"},
  i2cWriteBlockDataSyncClient{"writeBlockDataSyncClientImu_node"}
{
}

LifecycleCallbackReturn_t Imu::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  i2cImuReadingClient = this->create_client<HalPigpioI2cImuReading_t>("hal_pigpioI2cImuReading");
  imuDmpWritingClient = rclcpp_action::create_client<HalImuWriteDmpAction>(this, "hal_imuWriteDmp");
  imuGetHandleSyncClient.init("hal_imuGetHandle");
  i2cReadByteDataSyncClient.init("hal_pigpioI2cReadByteData");
  i2cWriteByteDataSyncClient.init("hal_pigpioI2cWriteByteData");
  i2cWriteBlockDataSyncClient.init("hal_pigpioI2cWriteBlockData");

  imuSubscriber = this->create_subscription<ImuDataMsg_t>(
    "hal_pigpioImu", 10, std::bind(&Imu::imuForwarder, this, _1));
  imuDataPublisher = this->create_publisher<ImuDataMsg_t>("imuData", 10);

  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Imu::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  imuHandle = hal::common::getI2cHandle(imuGetHandleSyncClient);

  resetImu(imuHandle);
  setClockSource(imuHandle);
  setAccelerometerSensitivity(imuHandle);
  setGyroscopeSensitivity(imuHandle);
  setConfiguration(imuHandle);
  setMpuRate(imuHandle, MPU6050_DMP_SAMPLE_RATE);
  dmpInit();

  imuDataPublisher->on_activate();

  RCLCPP_INFO(get_logger(), "Node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Imu::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  imuHandle = MPU6050_I2C_NO_HANDLE;
  imuDataPublisher->on_deactivate();
  stopImuReading();

  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Imu::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  imuDataPublisher.reset();
  imuSubscriber.reset();

  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Imu::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  imuDataPublisher.reset();
  imuSubscriber.reset();
  stopImuReading();

  RCLCPP_INFO(get_logger(), "Node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Imu::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void Imu::goal_response_callback(HalImuWriteDmpGoal::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server!");
  }
}

void Imu::result_callback(const HalImuWriteDmpGoal::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "DMP code written successfully.");
    setDmpRate(imuHandle, MPU6050_DMP_RATE_100HZ);
    setAccelerometerOffsets(imuHandle);
    setGyroscopeOffsets(imuHandle);
    configureDmpFeatures(imuHandle);
    enableDmpAndStartReading(imuHandle);
  } else {
    RCLCPP_ERROR(get_logger(), "Error while writing DMP code!");
  }
}

void Imu::feedback_callback(
  HalImuWriteDmpGoal::SharedPtr,
  const std::shared_ptr<const HalImuWriteDmpAction::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "Bank %u written.", feedback->bank);
}

void Imu::resetImu(int32_t imuHandle)
{
  RCLCPP_INFO(get_logger(), "IMU resetting...");

  /* Reset MPU6050 */
  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle, MPU6050_POWER_MANAGEMENT_1_REGISTER,
      MPU6050_RESET))
  {
    RCLCPP_ERROR(get_logger(), "Failed to reset IMU because chip couldn't be resetted.");
    return;
  }

  rclcpp::sleep_for(100ms);

  /* Reset signal paths */
  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle, MPU6050_SIGNAL_PATH_RESET_REGISTER,
      MPU6050_SIGNAL_PATH_RESET))
  {
    RCLCPP_ERROR(get_logger(), "Failed to reset IMU because signal paths couldn't be resetted.");
    return;
  }

  rclcpp::sleep_for(100ms);

  /* Disable sleep mode */
  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle, MPU6050_POWER_MANAGEMENT_1_REGISTER,
      0x00))
  {
    RCLCPP_ERROR(get_logger(), "Failed to reset IMU because sleep mode couldn't be disabled.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Successfully resetted IMU.");
}

void Imu::setClockSource(int32_t imuHandle)
{
  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle, MPU6050_POWER_MANAGEMENT_1_REGISTER,
      MPU6050_CLOCK_SOURCE_PLL_X))
  {
    RCLCPP_ERROR(get_logger(), "Failed to enable PLL_X clock source.");
  } else {
    RCLCPP_INFO(get_logger(), "Successfully enabled PLL_X clock source.");
  }
}

void Imu::setMpuRate(int32_t imuHandle, uint16_t rate)
{
  uint8_t div;

  if (rate > MPU6050_MAX_SAMPLE_RATE) {
    rate = MPU6050_MAX_SAMPLE_RATE;
  } else if (rate < MPU6050_MIN_SAMPLE_RATE) {
    rate = MPU6050_MIN_SAMPLE_RATE;
  }

  div = MPU6050_MAX_SAMPLE_RATE / rate - 1;

  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle, MPU6050_SAMPLE_RATE_REGISTER, div))
  {
    RCLCPP_ERROR(get_logger(), "Failed to set MPU sample rate.");
  } else {
    RCLCPP_INFO(get_logger(), "Successfully set MPU sample rate.");
  }
}

void Imu::setConfiguration(int32_t imuHandle)
{
  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle, MPU6050_CONFIGURATION_REGISTER,
      MPU6050_DLPF_BANDWITH_188))
  {
    RCLCPP_ERROR(get_logger(), "Failed to write configuration of MPU.");
  } else {
    RCLCPP_INFO(get_logger(), "Successfully wrote configuration of MPU.");
  }
}

void Imu::setAccelerometerSensitivity(int32_t imuHandle)
{
  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle,
      MPU6050_ACCELEROMETER_CONFIGURATION_REGISTER, MPU6050_ACCELEROMETER_FULL_SENSITIVITY))
  {
    RCLCPP_ERROR(get_logger(), "Failed to set accelerometer sensitivity!");
  } else {
    RCLCPP_INFO(get_logger(), "Successfully set accelerometer sensitivity.");
  }
}

void Imu::setGyroscopeSensitivity(int32_t imuHandle)
{
  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle,
      MPU6050_GYROSCOPE_CONFIGURATION_REGISTER, MPU6050_GYROSCOPE_FULL_SENSITIVITY))
  {
    RCLCPP_ERROR(get_logger(), "Failed to set gyroscope sensitivity!");
  } else {
    RCLCPP_INFO(get_logger(), "Successfully set gyroscope sensitivity.");
  }
}

void Imu::dmpInit(void)
{
  auto goal = HalImuWriteDmpAction::Goal();

  auto goal_callbacks = rclcpp_action::Client<HalImuWriteDmpAction>::SendGoalOptions();
  goal_callbacks.goal_response_callback = std::bind(&Imu::goal_response_callback, this, _1);
  goal_callbacks.feedback_callback = std::bind(&Imu::feedback_callback, this, _1, _2);
  goal_callbacks.result_callback = std::bind(&Imu::result_callback, this, _1);

  auto goal_future = imuDmpWritingClient->async_send_goal(goal, goal_callbacks);
}

void Imu::setDmpRate(int32_t imuHandle, uint16_t rate)
{
  uint16_t div;
  std::vector<uint8_t> div_vec;
  std::vector<uint8_t> dmpRegisterDiviserData{0xFE, 0xF2, 0xAB, 0xC4, 0xAA, 0xF1, 0xDF, 0xDF, 0xBB,
    0xAF, 0xDF, 0xDF};

  if (rate > MPU6050_DMP_SAMPLE_RATE) {
    div = MPU6050_DMP_SAMPLE_RATE / MPU6050_DMP_SAMPLE_RATE - 1;
  } else {
    div = MPU6050_DMP_SAMPLE_RATE / rate - 1;
  }

  div_vec.push_back(static_cast<uint8_t>(div >> 8));
  div_vec.push_back(static_cast<uint8_t>(div & 0xFF));

  if (!writeDataToDmp(
      imuHandle, MPU6050_DMP_SAMPLE_RATE_BANK, MPU6050_DMP_SAMPLE_RATE_ADDRESS, div_vec))
  {
    RCLCPP_ERROR(get_logger(), "Failed to write DMP sample rate.");
    return;
  }

  if (!writeDataToDmp(
      imuHandle, MPU6050_DMP_DIVISER_BANK, MPU6050_DMP_DIVISER_ADDRESS,
      dmpRegisterDiviserData))
  {
    RCLCPP_ERROR(get_logger(), "Failed to write DMP diviser data.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Successfully set DMP sample rate.");
}

void Imu::resetFifo(int32_t imuHandle)
{
  if (!hal::common::writeBitInRegister(
      i2cReadByteDataSyncClient, i2cWriteByteDataSyncClient, imuHandle,
      MPU6050_USER_CONTROL_REGISTER, MPU6050_FIFO_RESET_BIT, 1))
  {
    RCLCPP_ERROR(get_logger(), "Failed to reset FIFO!");
  } else {
    RCLCPP_INFO(get_logger(), "Successfully resetted FIFO.");
  }
}

void Imu::setAccelerometerOffsets(int32_t imuHandle)
{
  SensorBias accelerometerXBias{'X', ACCELEROMETER_X_OFFSET,
    MPU6050_ACCELEROMETER_X_OFFSET_MSB_REGISTER, MPU6050_ACCELEROMETER_X_OFFSET_LSB_REGISTER};
  SensorBias accelerometerYBias{'Y', ACCELEROMETER_Y_OFFSET,
    MPU6050_ACCELEROMETER_Y_OFFSET_MSB_REGISTER, MPU6050_ACCELEROMETER_Y_OFFSET_LSB_REGISTER};
  SensorBias accelerometerZBias{'Z', ACCELEROMETER_Z_OFFSET,
    MPU6050_ACCELEROMETER_Z_OFFSET_MSB_REGISTER, MPU6050_ACCELEROMETER_Z_OFFSET_LSB_REGISTER};

  const std::vector<SensorBias> accelerometerBiases{accelerometerXBias, accelerometerYBias,
    accelerometerZBias};

  if (!writeSensorBiases(imuHandle, accelerometerBiases)) {
    RCLCPP_ERROR(get_logger(), "Failed to set accelerometer offsets.");
  } else {
    RCLCPP_INFO(get_logger(), "Successfully set accelerometer offsets.");
  }
}

void Imu::setGyroscopeOffsets(int32_t imuHandle)
{
  SensorBias gyroscopeXBias{'X', GYROSCOPE_X_OFFSET, MPU6050_GYROSCOPE_X_OFFSET_MSB_REGISTER,
    MPU6050_GYROSCOPE_X_OFFSET_LSB_REGISTER};
  SensorBias gyroscopeYBias{'Y', GYROSCOPE_Y_OFFSET, MPU6050_GYROSCOPE_Y_OFFSET_MSB_REGISTER,
    MPU6050_GYROSCOPE_Y_OFFSET_LSB_REGISTER};
  SensorBias gyroscopeZBias{'Z', GYROSCOPE_Z_OFFSET, MPU6050_GYROSCOPE_Z_OFFSET_MSB_REGISTER,
    MPU6050_GYROSCOPE_Z_OFFSET_LSB_REGISTER};

  const std::vector<SensorBias> gyroscopeBiases{gyroscopeXBias, gyroscopeYBias, gyroscopeZBias};

  if (!writeSensorBiases(imuHandle, gyroscopeBiases)) {
    RCLCPP_ERROR(get_logger(), "Failed to set gyroscope offsets.");
  } else {
    RCLCPP_INFO(get_logger(), "Successfully set gyroscope offsets.");
  }
}

bool Imu::writeSensorBiases(int32_t imuHandle, const std::vector<SensorBias> sensorBiases)
{
  for (auto sensorBias : sensorBiases) {
    uint8_t sensorBiasMsb = static_cast<uint8_t>((sensorBias.bias >> 8) & 0xFF);
    uint8_t sensorBiasLsb = static_cast<uint8_t>(sensorBias.bias & 0xFF);

    if (!hal::common::writeByteInRegister(
        i2cWriteByteDataSyncClient, imuHandle, sensorBias.msbRegister,
        sensorBiasMsb))
    {
      RCLCPP_ERROR(get_logger(), "Failed to set sensor %c offset!", sensorBias.axis);
      return false;
    }

    if (!hal::common::writeByteInRegister(
        i2cWriteByteDataSyncClient, imuHandle, sensorBias.lsbRegister,
        sensorBiasLsb))
    {
      RCLCPP_ERROR(get_logger(), "Failed to set sensor %c offset!", sensorBias.axis);
      return false;
    }
  }

  return true;
}

void Imu::configureDmpFeatures(int32_t imuHandle)
{
  std::vector<uint8_t> dmpNoSensorData(MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_SIZE, 0xA3);
  std::vector<uint8_t> dmpNoGestureData{0xD8};
  std::vector<uint8_t> gyroscopeCalibrationDisabled{0xB8, 0xAA, 0xAA, 0xAA, 0xB0, 0x88, 0xC3, 0xC5,
    0xC7};
  std::vector<uint8_t> dmpNoTapData{0xD8};
  std::vector<uint8_t> dmpNoAndroidOrientation{0xD8};
  std::vector<uint8_t> dmpQuaternionDisabled(MPU6050_DMP_FEATURE_QUATERNION_SIZE, 0x8B);
  std::vector<uint8_t> dmp6AxisQuaternionEnabled{0x20, 0x28, 0x30, 0x38};

  if (!writeDataToDmp(
      imuHandle, MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_BANK,
      MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_ADDRESS, dmpNoSensorData))
  {
    RCLCPP_ERROR(get_logger(), "Failed to configure DMP features (sensor data disabled)!");
    return;
  }

  if (!writeDataToDmp(
      imuHandle, MPU6050_DMP_FEATURE_SEND_GESTURE_DATA_BANK,
      MPU6050_DMP_FEATURE_SEND_GESTURE_DATA_ADDRESS, dmpNoGestureData))
  {
    RCLCPP_ERROR(get_logger(), "Failed to configure DMP features (gesture data disabled)!");
    return;
  }

  if (!writeDataToDmp(
      imuHandle, MPU6050_CFG_MOTION_BIAS_BANK, MPU6050_CFG_MOTION_BIAS_ADDRESS,
      gyroscopeCalibrationDisabled))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to configure DMP features (gyroscope calibration disabled)!");
    return;
  }

  if (!writeDataToDmp(
      imuHandle, MPU6050_DMP_FEATURE_SEND_TAP_DATA_BANK,
      MPU6050_DMP_FEATURE_SEND_TAP_DATA_ADDRESS, dmpNoTapData))
  {
    RCLCPP_ERROR(get_logger(), "Failed to configure DMP features (tap data disabled)!");
    return;
  }

  if (!writeDataToDmp(
      imuHandle, MPU6050_DMP_FEATURE_SEND_ANDROID_ORIENTATION_BANK,
      MPU6050_DMP_FEATURE_SEND_ANDROID_ORIENTATION_ADDRESS, dmpNoAndroidOrientation))
  {
    RCLCPP_ERROR(get_logger(), "Failed to configure DMP features (android orientation disabled)!");
    return;
  }

  if (!writeDataToDmp(
      imuHandle, MPU6050_DMP_FEATURE_QUATERNION_BANK, MPU6050_DMP_FEATURE_QUATERNION_ADDRESS,
      dmpQuaternionDisabled))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to configure DMP features (quaternion computation disabled)!");
    return;
  }

  if (!writeDataToDmp(
      imuHandle, MPU6050_DMP_FEATURE_6X_LP_QUAT_BANK, MPU6050_DMP_FEATURE_6X_LP_QUAT_ADDRESS,
      dmp6AxisQuaternionEnabled))
  {
    RCLCPP_ERROR(
      get_logger(), "Failed to configure DMP features (6 axis quaternion computation enabled)!");
    return;
  }

  resetFifo(imuHandle);

  RCLCPP_INFO(get_logger(), "Successfully configured DMP features.");
}

void Imu::enableDmpAndStartReading(int32_t imuHandle)
{
  /* Enable DMP and FIFO */
  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle, MPU6050_USER_CONTROL_REGISTER,
      MPU6050_DMP_EMABLE | MPU6050_FIFO_ENABLE))
  {
    RCLCPP_ERROR(get_logger(), "Failed to enable DMP!");
    return;
  }

  resetFifo(imuHandle);

  startImuReading();

  RCLCPP_INFO(get_logger(), "Successfully enabled DMP.");
}

bool Imu::writeDataToDmp(
  int32_t imuHandle, uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data)
{
  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle, MPU6050_BANK_SELECTION_REGISTER,
      bank))
  {
    RCLCPP_ERROR(get_logger(), "Failed to write bank!");
    return false;
  }

  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle, MPU6050_ADDRESS_IN_BANK_REGISTER,
      addressInBank))
  {
    RCLCPP_ERROR(get_logger(), "Failed to write address!");
    return false;
  }

  if (!hal::common::writeDataBlock(
      i2cWriteBlockDataSyncClient, imuHandle, MPU6050_READ_WRITE_REGISTER, data))
  {
    RCLCPP_ERROR(get_logger(), "Failed to write data!");
    return false;
  }

  return true;
}

void Imu::startImuReading()
{
  auto i2cImuReadingRequest = std::make_shared<HalPigpioI2cImuReading_t::Request>();

  i2cImuReadingRequest->imu_handle = imuHandle;
  i2cImuReadingRequest->is_imu_ready = true;

  auto i2cImuReadingFuture = i2cImuReadingClient->async_send_request(i2cImuReadingRequest);
}

void Imu::stopImuReading()
{
  auto i2cImuReadingRequest = std::make_shared<HalPigpioI2cImuReading_t::Request>();

  i2cImuReadingRequest->imu_handle = imuHandle;
  i2cImuReadingRequest->is_imu_ready = false;

  auto i2cImuReadingFuture = i2cImuReadingClient->async_send_request(i2cImuReadingRequest);
}

void Imu::imuForwarder(const ImuDataMsg_t & msg)
{
  auto imuData = ImuDataMsg_t(msg);

  imuDataPublisher->publish(imuData);
}

}  // namespace imu
}  // namespace hal
