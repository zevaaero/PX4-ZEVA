/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

/**
 * @file SDP6X.hpp
 *
 * Driver for Sensirion SDP6X Differential Pressure Sensor
 *
 * Datasheet: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/8_Differential_Pressure/Datasheets/Sensirion_Differential_Pressure_Sensors_SDP600Series_Datasheet.pdf
 *
 * Note:
 * - Expected to be used with SDP610 - 500Pa
 * - The board needs to set:
 *   CONFIG_STM32F7_I2C_DYNTIMEO_STARTSTOP=50
 *   CONFIG_STM32F7_I2C_DYNTIMEO_USECPERBYTE=10000
 */

#pragma once

#include <drivers/airspeed/airspeed.h>
#include <math.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>

#define I2C_ADDRESS_SDP6X		0x40

#define SDP6X_RESET_CMD			0xfe
#define SDP6X_MEASURE_CMD			0xf1

#define PATH_SDP6X "/dev/sdp6x"

#define SPD6X_MEAS_RATE 20
#define SDP6X_MEAS_DRIVER_FILTER_FREQ 3.0f
#define CONVERSION_INTERVAL	(1000000 / SPD6X_MEAS_RATE)	/* microseconds */

class SDP6X : public Airspeed, public I2CSPIDriver<SDP6X>
{
public:
	SDP6X(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = I2C_ADDRESS_SDP6X) :
		Airspeed(bus, bus_frequency, address, CONVERSION_INTERVAL),
		I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
	{
	}

	virtual ~SDP6X() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void	RunImpl();

private:

	int	measure() override { return 0; }
	int	collect() override;
	int	probe() override;

	math::LowPassFilter2p _filter{SPD6X_MEAS_RATE, SDP6X_MEAS_DRIVER_FILTER_FREQ};

	bool init_sdp6x();

	/**
	 * Calculate the CRC8 for the sensor payload data
	 */
	bool crc(const uint8_t data[], unsigned size, uint8_t checksum);
};
