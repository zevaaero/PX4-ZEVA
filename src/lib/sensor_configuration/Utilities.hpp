/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <lib/conversion/rotation.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <matrix/math.hpp>

namespace sensor_configuration
{

/**
 * @brief Find sensor's calibration index if it exists.
 *
 * @param sensor_type Configuration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param device_id
 * @return int8_t Valid calibration index on success, -1 otherwise
 */
int8_t FindCurrentConfigurationIndex(const char *prefix, const char *sensor_type, uint32_t device_id);

/**
 * @brief Find sensor's calibration index if it exists, otherwise select an available slot.
 *
 * @param sensor_type Configuration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param device_id
 * @param preferred_index preferred index (optional)
 * @return int8_t Valid calibration index on success, -1 otherwise
 */
int8_t FindAvailableConfigurationIndex(const char *prefix, const char *sensor_type, uint32_t device_id,
				       int8_t preferred_index = -1);

/**
 * @brief Get sensor calibration parameter value.
 *
 * @param prefix Configuration parameter prefix string ("CAL", "SENS")
 * @param sensor_type Configuration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Configuration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance
 * @return int32_t The calibration value.
 */
int32_t GetConfigurationParamInt32(const char *prefix, const char *sensor_type, const char *cal_type, uint8_t instance);
float GetConfigurationParamFloat(const char *prefix, const char *sensor_type, const char *cal_type, uint8_t instance);

/**
 * @brief Set a single configuration paramter.
 *
 * @param prefix Configuration parameter prefix string ("CAL", "SENS")
 * @param sensor_type Configuration parameter abbreviated sensor string ("ACC", "GNSS", "GYRO", "MAG")
 * @param cal_type Configuration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance Configuration index (0 - 3)
 * @param value int32_t parameter value
 * @return true if the parameter name was valid and value saved successfully, false otherwise.
 */
template<typename T>
bool SetConfigurationParam(const char *prefix, const char *sensor_type, const char *cal_type, uint8_t instance, T value)
{
	char str[20] {};

	// eg CAL_MAGn_ID/CAL_MAGn_ROT
	sprintf(str, "%s_%s%u_%s", prefix, sensor_type, instance, cal_type);

	int ret = param_set_no_notification(param_find(str), &value);

	if (ret != PX4_OK) {
		PX4_ERR("failed to set %s", str);
	}

	return ret == PX4_OK;
}

/**
 * @brief Get the Configuration Params Vector 3f object
 *
 * @param sensor_type Configuration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Configuration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance Configuration index (0 - 3)
 * @return matrix::Vector3f Vector of calibration values.
 */
matrix::Vector3f GetConfigurationParamsVector3f(const char *prefix, const char *sensor_type, const char *cal_type,
		uint8_t instance);

/**
 * @brief Set the Configuration Params Vector 3f object
 *
 * @param sensor_type Configuration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Configuration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance Configuration index (0 - 3)
 * @param values Vector of calibration values x, y, z.
 * @return true if the parameter name was valid and all values saved successfully, false otherwise.
 */
bool SetConfigurationParamsVector3f(const char *prefix, const char *sensor_type, const char *cal_type, uint8_t instance,
				    matrix::Vector3f values);

/**
 * @brief Get the board sensor level adjustment (SENS_BOARD_X_OFF, SENS_BOARD_Y_OFF, SENS_BOARD_Z_OFF).
 *
 * @return matrix::Eulerf
 */
matrix::Eulerf GetSensorLevelAdjustment();

/**
 * @brief Get the board rotation.
 *
 * @return enum Rotation
 */
Rotation GetBoardRotation();

/**
 * @brief Get the board rotation Dcm.
 *
 * @return matrix::Dcmf
 */
matrix::Dcmf GetBoardRotationMatrix();

/**
 * @brief Determine if device is on an external bus
 *
 * @return true if device is on an external bus
 */
bool DeviceExternal(uint32_t device_id);

} // namespace sensor_configuration
