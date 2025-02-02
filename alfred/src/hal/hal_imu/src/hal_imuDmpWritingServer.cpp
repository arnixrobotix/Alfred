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

#include "hal_imuDmpWritingServer.hpp"

using namespace std::placeholders;

namespace hal
{
namespace imu
{
namespace dmp
{

DmpWritingServer::DmpWritingServer()
: rclcpp_lifecycle::LifecycleNode{"hal_imuDmpWritingServer_node"},
  imuHandle{MPU6050_I2C_NO_HANDLE},
  imuGetHandleSyncClient{"getHandleSyncClientDmp_node"},
  i2cWriteByteDataSyncClient{"writeByteDataSyncClientDmp_node"},
  i2cWriteBlockDataSyncClient{"writeBlockDataSyncClientDmp_node"}
{
}

LifecycleCallbackReturn_t DmpWritingServer::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  imuDmpWritingServer = rclcpp_action::create_server<HalImuWriteDmpAction>(
    this,
    "hal_imuWriteDmp",
    std::bind(&DmpWritingServer::handle_goal, this, _1, _2),
    std::bind(&DmpWritingServer::handle_cancel, this, _1),
    std::bind(&DmpWritingServer::handle_accepted, this, _1));
  imuGetHandleSyncClient.init("hal_imuGetHandle");
  i2cWriteByteDataSyncClient.init("hal_pigpioI2cWriteByteData");
  i2cWriteBlockDataSyncClient.init("hal_pigpioI2cWriteBlockData");

  RCLCPP_INFO(get_logger(), "Node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t DmpWritingServer::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  imuHandle = hal::common::getI2cHandle(imuGetHandleSyncClient);

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t DmpWritingServer::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  imuHandle = MPU6050_I2C_NO_HANDLE;
  RCLCPP_INFO(get_logger(), "Node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t DmpWritingServer::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t DmpWritingServer::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t DmpWritingServer::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

rclcpp_action::GoalResponse DmpWritingServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const HalImuWriteDmpAction::Goal> goal)
{
  (void)uuid;
  (void)goal;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DmpWritingServer::handle_cancel(
  const std::shared_ptr<HalImuWriteDmpGoal> goal_handle)
{
  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void DmpWritingServer::handle_accepted(const std::shared_ptr<HalImuWriteDmpGoal> goal_handle)
{
  std::thread{std::bind(&DmpWritingServer::writeDmp, this, _1), goal_handle}.detach();
}

void DmpWritingServer::writeDmp(const std::shared_ptr<HalImuWriteDmpGoal> goal_handle)
{
  /* This address is the start address of DMP code */
  /* It is coming from InvenSense */
  const uint8_t startAddressMsb = 0x04;
  const uint8_t startAddressLsb = 0x00;

  uint8_t bank = 0;
  uint8_t byteAddressInBank = 0;
  uint8_t chunkAddressInBank = 0;
  uint8_t indexInChunk = 0;
  std::vector<uint8_t> data;
  bool writeSuccess = false;

  auto feedback = std::make_shared<HalImuWriteDmpAction::Feedback>();
  auto result = std::make_shared<HalImuWriteDmpAction::Result>();

  (void)goal_handle;

  RCLCPP_INFO(get_logger(), "Started writing DMP code.");

  for (uint16_t byte = 0; byte < DMP_CODE_SIZE; ++byte) {
    indexInChunk = byte % MPU6050_CHUNK_SIZE;
    byteAddressInBank = byte % MPU6050_BANK_SIZE;
    bank = (byte - byteAddressInBank) / MPU6050_BANK_SIZE;

    data.push_back(dmp_memory[byte]);

    /* The chunk is full and ready to be written or we reached the end of the DMP code */
    if ((indexInChunk == (MPU6050_CHUNK_SIZE - 1)) || (byte == DMP_CODE_SIZE - 1)) {
      chunkAddressInBank = byteAddressInBank - indexInChunk;

      if (chunkAddressInBank == 0) {
        /* A new bank is starting */
        feedback->bank = bank;
        goal_handle->publish_feedback(feedback);
      }

      if (!writeData(bank, chunkAddressInBank, data)) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to write DMP code: chunk at address %u of bank %u not written!",
          chunkAddressInBank,
          bank);
        goal_handle->abort(result);
        return;
      }

      data.clear();
    }
  }

  writeSuccess = hal::common::writeByteInRegister(
    i2cWriteByteDataSyncClient, imuHandle,
    MPU6050_DMP_START_ADDRESS_H_REGISTER, startAddressMsb);
  writeSuccess &= hal::common::writeByteInRegister(
    i2cWriteByteDataSyncClient, imuHandle,
    MPU6050_DMP_START_ADDRESS_L_REGISTER, startAddressLsb);
  if (!writeSuccess) {
    RCLCPP_ERROR(get_logger(), "Failed to write DMP code: start address not written!");
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(get_logger(), "Successfully wrote DMP code.");
  goal_handle->succeed(result);
}

bool DmpWritingServer::writeData(uint8_t bank, uint8_t addressInBank, std::vector<uint8_t> data)
{
  if (addressInBank == 0) {
    /* A new bank is starting */
    if (!hal::common::writeByteInRegister(
        i2cWriteByteDataSyncClient, imuHandle, MPU6050_BANK_SELECTION_REGISTER,
        bank))
    {
      return false;
    }
  }

  if (!hal::common::writeByteInRegister(
      i2cWriteByteDataSyncClient, imuHandle, MPU6050_ADDRESS_IN_BANK_REGISTER,
      addressInBank))
  {
    return false;
  }

  if (!hal::common::writeDataBlock(
      i2cWriteBlockDataSyncClient, imuHandle, MPU6050_READ_WRITE_REGISTER, data))
  {
    return false;
  }

  return true;
}

}  // namespace dmp
}  // namespace imu
}  // namespace hal
