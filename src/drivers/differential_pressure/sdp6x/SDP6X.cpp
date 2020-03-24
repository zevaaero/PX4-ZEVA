/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 */

#include "SDP6X.hpp"

int
SDP6X::probe()
{
	return !init_sdp6x();
}

bool
SDP6X::init_sdp6x()
{
	uint8_t reset_cmd = SDP6X_RESET_CMD;
	int ret = transfer(&reset_cmd, 1, nullptr, 0);

	if (ret == 0) {
		_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SDP31;
		// wait until sensor is ready
		px4_usleep(20000);
	}

	return ret == 0;
}

int
SDP6X::collect()
{
	perf_begin(_sample_perf);

	// this takes about 5ms. But even if we split the write and read with a delay in between,
	// the device will still block for several ms. So effectively can't run another device on that bus.
	uint8_t measure_cmd = SDP6X_MEASURE_CMD;
	uint8_t data[3];
	int ret = transfer(&measure_cmd, 1, (uint8_t *)&data[0], sizeof(data));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Check the CRC
	if (!crc(&data[0], 2, data[2])) {
		perf_count(_comms_errors);
		return EAGAIN;
	}

	int16_t P = (((int16_t)data[0]) << 8) | data[1];

	float diff_press_pa_raw = static_cast<float>(P) / 60.f; // 500Pa -> scale factor=60

	differential_pressure_s report{};

	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.temperature = 0.f;
	report.differential_pressure_filtered_pa = _filter.apply(diff_press_pa_raw) - _diff_pres_offset;
	report.differential_pressure_raw_pa = diff_press_pa_raw - _diff_pres_offset;
	report.device_id = _device_id.devid;

	_airspeed_pub.publish(report);

	perf_end(_sample_perf);

	return ret;
}

void
SDP6X::RunImpl()
{
	int ret = PX4_ERROR;

	// measurement phase
	ret = collect();

	if (PX4_OK != ret) {
		_sensor_ok = false;
		DEVICE_DEBUG("measure error");
	}

	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(CONVERSION_INTERVAL);
}

bool SDP6X::crc(const uint8_t data[], unsigned size, uint8_t checksum)
{
	uint8_t crc_value = 0;

	// calculate 8-bit checksum with polynomial 0x31 (x^8 + x^5 + x^4 + 1)
	for (unsigned i = 0; i < size; i++) {
		crc_value ^= (data[i]);

		for (int bit = 8; bit > 0; --bit) {
			if (crc_value & 0x80) {
				crc_value = (crc_value << 1) ^ 0x31;

			} else {
				crc_value = (crc_value << 1);
			}
		}
	}

	// verify checksum
	return (crc_value == checksum);
}
