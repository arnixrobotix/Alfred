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

#ifndef MPU6050_HPP_
#define MPU6050_HPP_

#define MPU6050_I2C_ADDRESS 0x68

#define MPU6050_I2C_NO_HANDLE -1

#define MPU6050_BANK_SELECTION_REGISTER 0x6D
#define MPU6050_ADDRESS_IN_BANK_REGISTER 0x6E
#define MPU6050_READ_WRITE_REGISTER 0x6F
#define MPU6050_BANK_SIZE 256
#define MPU6050_CHUNK_SIZE 16

#define MPU6050_SAMPLE_RATE_REGISTER 0x19
#define MPU6050_MIN_SAMPLE_RATE 4
#define MPU6050_MAX_SAMPLE_RATE 1000

#define MPU6050_DMP_SAMPLE_RATE 200
#define MPU6050_DMP_RATE_100HZ 100
#define MPU6050_DMP_SAMPLE_RATE_BANK 0x02
#define MPU6050_DMP_SAMPLE_RATE_ADDRESS 0x16
#define MPU6050_DMP_DIVISER_BANK 0x0A
#define MPU6050_DMP_DIVISER_ADDRESS 0xC1

#define MPU6050_DMP_FEATURE_6X_LP_QUAT_BANK 0x0A
#define MPU6050_DMP_FEATURE_6X_LP_QUAT_ADDRESS 0x9E
#define MPU6050_DMP_FEATURE_6X_LP_QUAT_SIZE 4
#define MPU6050_DMP_FIFO_QUAT_SIZE 16
#define MPU6050_QUATERNION_SCALE 1073741824.0f

#define MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_BANK 0x0A
#define MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_ADDRESS 0xA7
#define MPU6050_DMP_FEATURE_SEND_SENSOR_DATA_SIZE 10

#define MPU6050_DMP_FEATURE_SEND_GESTURE_DATA_BANK 0x0A
#define MPU6050_DMP_FEATURE_SEND_GESTURE_DATA_ADDRESS 0xB6

#define MPU6050_DMP_FEATURE_SEND_TAP_DATA_BANK 0x08
#define MPU6050_DMP_FEATURE_SEND_TAP_DATA_ADDRESS 0xB0

#define MPU6050_DMP_FEATURE_SEND_ANDROID_ORIENTATION_BANK 0x07
#define MPU6050_DMP_FEATURE_SEND_ANDROID_ORIENTATION_ADDRESS 0x3D

#define MPU6050_DMP_FEATURE_QUATERNION_BANK 0x0A
#define MPU6050_DMP_FEATURE_QUATERNION_ADDRESS 0x98
#define MPU6050_DMP_FEATURE_QUATERNION_SIZE 4

#define MPU6050_CFG_MOTION_BIAS_BANK 0x04
#define MPU6050_CFG_MOTION_BIAS_ADDRESS 0xB8
#define MPU6050_DMP_FEATURE_GYRO_CAL_SIZE 9

#define MPU6050_DMP_ORIENTATION_MATRIX_VECTOR_SIZE 3
#define MPU6050_DMP_ORIENTATION_MATRIX_GYRO_AXES_BANK 0x04
#define MPU6050_DMP_ORIENTATION_MATRIX_GYRO_AXES_ADDRESS 0x26
#define MPU6050_DMP_ORIENTATION_MATRIX_GYRO_SIGNS_BANK 0x04
#define MPU6050_DMP_ORIENTATION_MATRIX_GYRO_SIGNS_ADDRESS 0x40
#define MPU6050_DMP_ORIENTATION_MATRIX_ACCEL_AXES_BANK 0x04
#define MPU6050_DMP_ORIENTATION_MATRIX_ACCEL_AXES_ADDRESS 0x2A
#define MPU6050_DMP_ORIENTATION_MATRIX_ACCEL_SIGNS_BANK 0x04
#define MPU6050_DMP_ORIENTATION_MATRIX_ACCEL_SIGNS_ADDRESS 0x31

#define MPU6050_SIGNAL_PATH_RESET_REGISTER 0x68
#define MPU6050_SIGNAL_PATH_RESET 0x07

#define MPU6050_USER_CONTROL_REGISTER 0x6A
#define MPU6050_FIFO_RESET 0x04
#define MPU6050_FIFO_RESET_BIT 2
#define MPU6050_AUX_I2C_MASTER_ENABLE 0x20
#define MPU6050_DMP_RESET 0x08
#define MPU6050_FIFO_ENABLE 0x40
#define MPU6050_DMP_EMABLE 0x80

#define MPU6050_POWER_MANAGEMENT_1_REGISTER 0x6B
#define MPU6050_RESET 0x80
#define MPU6050_RESET_BIT 7
#define MPU6050_ENABLE_SLEEP_BIT 6
#define MPU6050_CLOCK_SOURCE_BIT_1 0
#define MPU6050_CLOCK_SOURCE_BIT_2 1
#define MPU6050_CLOCK_SOURCE_BIT_3 2
#define MPU6050_CLOCK_SOURCE_PLL_X 0x1

#define MPU6050_SAMPLE_RATE_DIVIDER_REGISTER 0x19

#define MPU6050_DMP_START_ADDRESS_H_REGISTER 0x70
#define MPU6050_DMP_START_ADDRESS_L_REGISTER 0x71

#define MPU6050_INTERRUPT_ENABLE_REGISTER 0x38
#define MPU6050_DMP_INTERRUPT_ONLY 0x02

#define MPU6050_INTERRUPT_STATUS_REGISTER 0x3A
#define MPU6050_FIFO_OVERFLOW 0x10

#define MPU6050_FIFO_ENABLE_REGISTER 0x23
#define MPU6050_FIFO_COUNT_H_REGISTER 0x72
#define MPU6050_FIFO_COUNT_L_REGISTER 0x73
#define MPU6050_FIFO_REGISTER 0x74
#define MPU6050_ACCELEROMETER_FIFO_ENABLE_BIT 0x8

#define MPU6050_MAX_FIFO_SAMPLES 1024
#define MPU6050_MAX_QUATERNIONS_SAMPLES 32

#define MPU6050_CONFIGURATION_REGISTER 0x1A
#define MPU6050_DLPF_BANDWITH_42 0x03
#define MPU6050_DLPF_BANDWITH_188 0x01
#define MPU6050_DLPF_BANDWITH_256 0x00
#define MPU6050_EXT_SYNC_TEMP_OUT_L 0x08

#define MPU6050_GYROSCOPE_CONFIGURATION_REGISTER 0x1B
#define MPU6050_GYROSCOPE_FULL_SENSITIVITY 0x18

#define MPU6050_ACCELEROMETER_CONFIGURATION_REGISTER 0x1C
#define MPU6050_ACCELEROMETER_FULL_SENSITIVITY 0x00

#define MPU6050_DATA_SIZE 6
#define MPU6050_DATA_NB_VALUES 3

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

#endif  // MPU6050_HPP_
