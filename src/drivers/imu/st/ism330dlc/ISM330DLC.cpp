/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "ISM330DLC.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t lsb, uint8_t msb) { return (msb << 8u) | lsb; }

ISM330DLC::ISM330DLC(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		     spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	SPI(MODULE_NAME, nullptr, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_drdy_gpio(drdy_gpio),
	_px4_accel(get_device_id(), ORB_PRIO_DEFAULT, rotation),
	_px4_gyro(get_device_id(), ORB_PRIO_DEFAULT, rotation)
{
	set_device_type(DRV_IMU_DEVTYPE_ST_ISM330DLC);
	_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ST_ISM330DLC);
	_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ST_ISM330DLC);

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

ISM330DLC::~ISM330DLC()
{
	perf_free(_bad_transfer_perf);
	perf_free(_interval_perf);
	perf_free(_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_count_perf);
	perf_free(_drdy_interval_perf);
	perf_free(_bad_register_perf);
}

void ISM330DLC::exit_and_cleanup()
{
	if (_drdy_gpio != 0) {
		// Disable data ready callback
		px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr);

		RegisterWrite(Register::INT1_CTRL, 0);
	}

	I2CSPIDriverBase::exit_and_cleanup();
}

int ISM330DLC::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != ISM330DLC_WHO_AM_I) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int ISM330DLC::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	if (!Reset()) {
		PX4_ERR("reset failed");
		return PX4_ERROR;
	}

	Start();

	return PX4_OK;
}

void ISM330DLC::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = 1000; // default to 1 kHz
	}

	_fifo_empty_interval_us = math::max(((1000000 / sample_rate) / 250) * 250, 250); // round down to nearest 250 us
	_fifo_gyro_samples = math::min(_fifo_empty_interval_us / (1000000 / GYRO_RATE), FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1000000 / GYRO_RATE);

	_fifo_accel_samples = math::min(_fifo_empty_interval_us / (1000000 / ACCEL_RATE), FIFO_MAX_SAMPLES);

	_px4_accel.set_update_rate(1000000 / _fifo_empty_interval_us);
	_px4_gyro.set_update_rate(1000000 / _fifo_empty_interval_us);

}

bool ISM330DLC::Reset()
{
	// CTRL3_C: SW_RESET
	RegisterWrite(Register::CTRL3_C, CTRL3_C_BIT::SW_RESET);
	usleep(50);	// Wait 50 μs (or wait until the SW_RESET bit of the CTRL3_C register returns to 0).

	const bool reset_done = ((RegisterRead(Register::CTRL3_C) & CTRL3_C_BIT::SW_RESET) == 0);

	return reset_done && Configure();
}

bool ISM330DLC::Configure()
{
	for (const auto &reg : _register_cfg) {
		RegisterCheck(reg);
	}

	_px4_accel.set_scale(0.488f * (CONSTANTS_ONE_G / 1000.0f));	// 0.488 mg/LSB
	_px4_accel.set_range(16.0f * CONSTANTS_ONE_G);

	_px4_gyro.set_scale(math::radians(70.0f / 1000.0f));	// 70 mdps/LSB
	_px4_gyro.set_range(math::radians(2000.0f));

	// FIFO watermark threshold in number of words
	const uint16_t fifo_watermark_threshold = _fifo_gyro_samples * sizeof(FIFO::DATA) / 2;
	RegisterWrite(Register::FIFO_CTRL1, fifo_watermark_threshold & 0xFF);
	RegisterWrite(Register::FIFO_CTRL2, (fifo_watermark_threshold >> 8) & 0x7);

	RegisterWrite(Register::DRDY_PULSE_CFG, 0);

	// INT1: FIFO full, overrun, or threshold
	RegisterWrite(Register::INT1_CTRL, INT1_CTRL_BIT::INT1_FULL_FLAG | INT1_CTRL_BIT::INT1_FIFO_OVR |
		      INT1_CTRL_BIT::INT1_FTH);

	// Note: When the FIFO is used, the IF_INC and BDU bits must be equal to 1
	// (but enable them last, otherwise they cause problems (failing health checks)).
	RegisterWrite(Register::CTRL3_C, CTRL3_C_BIT::BDU | CTRL3_C_BIT::IF_INC);

	return true;
}

void ISM330DLC::ResetFIFO()
{
	perf_count(_fifo_reset_perf);

	// FIFO_CTRL5 - disable FIFO
	RegisterWrite(Register::FIFO_CTRL5, 0);

	_fifo_watermark_interrupt_timestamp = 0;
	_fifo_read_samples.store(0);

	// FIFO_CTRL3: full gyro and accel data to FIFO
	RegisterWrite(Register::FIFO_CTRL3, FIFO_CTRL3_BIT::DEC_FIFO_GYRO | FIFO_CTRL3_BIT::DEC_FIFO_XL);
	RegisterWrite(Register::FIFO_CTRL4, 0);

	// FIFO_CTRL5: FIFO ODR is set to 6.66 kHz, and FIFO continuous mode enabled
	RegisterWrite(Register::FIFO_CTRL5, FIFO_CTRL5_BIT::ODR_FIFO_6_66_KHZ | FIFO_CTRL5_BIT::FIFO_MODE_CONTINUOUS);
}

bool ISM330DLC::RegisterCheck(const register_config_t &reg_cfg, bool notify)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && (reg_value & reg_cfg.set_bits) != reg_cfg.set_bits) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && (reg_value & reg_cfg.clear_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	if (!success) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

		if (notify) {
			perf_count(_bad_register_perf);
			_px4_accel.increase_error_count();
			_px4_gyro.increase_error_count();
		}
	}

	return success;
}

uint8_t ISM330DLC::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void ISM330DLC::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void ISM330DLC::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void ISM330DLC::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

void ISM330DLC::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = orig_val;

	if (setbits) {
		val |= setbits;
	}

	if (clearbits) {
		val &= ~clearbits;
	}

	RegisterWrite(reg, val);
}

int ISM330DLC::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	ISM330DLC *dev = reinterpret_cast<ISM330DLC *>(arg);
	dev->DataReady();
	return 0;
}

void ISM330DLC::DataReady()
{
	_fifo_watermark_interrupt_timestamp = hrt_absolute_time();
	_fifo_read_samples.store(_fifo_gyro_samples);
	ScheduleNow();
	perf_count(_drdy_interval_perf);
}

void ISM330DLC::Start()
{
	ResetFIFO();

	// FIXME: DRDY is not working properly yet. Perf counters and sampling generally looks good, but
	// there are rare cases where timestamps/number of samples are not correct (data is overlapping when plotting)
	if (_drdy_gpio != 0 && false) {
		// Setup data ready on rising edge
		px4_arch_gpiosetevent(_drdy_gpio, true, false, true, &ISM330DLC::DataReadyInterruptCallback, this);

		_data_ready_interrupt_enabled = true;

		// backup schedule as a watchdog timeout
		ScheduleDelayed(10_ms);

	} else {
		ScheduleOnInterval(_fifo_empty_interval_us, 10_ms);
		_data_ready_interrupt_enabled = false;
	}
}

void ISM330DLC::RunImpl()
{
	hrt_abstime timestamp_sample = 0;
	uint8_t samples = 0;

	if (_data_ready_interrupt_enabled) {
		// re-schedule as watchdog timeout
		ScheduleDelayed(10_ms);

		// timestamp set in data ready interrupt
		samples = _fifo_read_samples.load();
		timestamp_sample = _fifo_watermark_interrupt_timestamp;
	}

	bool failure = false;

	// manually check FIFO count if no samples from DRDY or timestamp looks bogus
	if (!_data_ready_interrupt_enabled || (samples == 0)
	    || (hrt_elapsed_time(&timestamp_sample) > (_fifo_empty_interval_us / 2))) {

		// use the time now roughly corresponding with the last sample we'll pull from the FIFO
		timestamp_sample = hrt_absolute_time();
		const uint16_t fifo_count_words = FIFOReadCount();

		if (fifo_count_words == 0) {
			failure = true;
			perf_count(_fifo_empty_perf);
		}

		samples = fifo_count_words * 2 / sizeof(FIFO::DATA);
	}

	if (samples > FIFO_MAX_SAMPLES) {
		// not technically an overflow, but more samples than we expected or can publish
		perf_count(_fifo_overflow_perf);
		failure = true;
		ResetFIFO();

	} else if (samples >= 1) {
		if (!FIFORead(timestamp_sample, samples)) {
			failure = true;
			_px4_accel.increase_error_count();
			_px4_gyro.increase_error_count();
		}
	}

	if (failure || hrt_elapsed_time(&_last_config_check_timestamp) > 10_ms) {
		// check registers incrementally
		if (RegisterCheck(_register_cfg[_checked_register], true)) {
			_last_config_check_timestamp = timestamp_sample;
			_checked_register = (_checked_register + 1) % size_register_cfg;

		} else {
			// register check failed, force reconfigure
			PX4_DEBUG("Health check failed, reconfiguring");
			Reset();
		}
	}

	perf_count(_interval_perf);

}

uint16_t ISM330DLC::FIFOReadCount()
{
	struct StatusRegs {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_STATUS1) | DIR_READ};
		uint8_t fifo_status1;
		uint8_t fifo_status2;
		uint8_t fifo_status3;
		uint8_t fifo_status4;
	};

	StatusRegs fifo_status{};

	if (transfer((uint8_t *)&fifo_status, (uint8_t *)&fifo_status, sizeof(StatusRegs)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	// Number of unread words (16-bit axes) stored in FIFO.
	const uint16_t fifo_words = (((uint16_t)(fifo_status.fifo_status2 & 0x7)) << 8) | fifo_status.fifo_status1;

	// check for FIFO status
	if (fifo_status.fifo_status2 & FIFO_STATUS2_BIT::OVER_RUN) {
		// overflow
		perf_count(_fifo_overflow_perf);
		ResetFIFO();
		return 0;

	} else if (fifo_status.fifo_status2 & FIFO_STATUS2_BIT::FIFO_EMPTY) {
		// fifo empty could indicate a problem, reset
		perf_count(_fifo_empty_perf);
		ResetFIFO();
		return 0;
	}

	// FIFO pattern: indicates Next reading from FIFO output registers (Gx, Gy, Gz, XLx, XLy, XLz)
	if (fifo_status.fifo_status3 != 0) {
		PX4_DEBUG("check FIFO pattern: %d", fifo_status.fifo_status3);
	}

	return fifo_words;
}

bool ISM330DLC::FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples)
{
	perf_begin(_transfer_perf);
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = samples * sizeof(FIFO::DATA) + 1;

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		perf_count(_bad_transfer_perf);
		return false;
	}

	perf_end(_transfer_perf);

	bool bad_data = false;

	PX4Gyroscope::FIFOSample gyro;
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = _fifo_empty_interval_us / _fifo_gyro_samples;

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = buffer.f[i];
		// sensor Z is up (RHC), flip y & z for publication
		int16_t gyro_x = combine(fifo_sample.OUTX_L_G, fifo_sample.OUTX_H_G);
		int16_t gyro_y = combine(fifo_sample.OUTY_L_G, fifo_sample.OUTY_H_G);
		int16_t gyro_z = combine(fifo_sample.OUTZ_L_G, fifo_sample.OUTZ_H_G);

		gyro.x[i] = gyro_x;
		gyro.y[i] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

	_px4_gyro.updateFIFO(gyro);

	PX4Accelerometer::FIFOSample accel;
	accel.timestamp_sample = timestamp_sample;
	accel.samples = samples;
	accel.dt = _fifo_empty_interval_us / _fifo_accel_samples;

	for (int i = 0; i < samples; ++i) {
		const FIFO::DATA &fifo_sample = buffer.f[i];
		int16_t accel_x = combine(fifo_sample.OUTX_L_XL, fifo_sample.OUTX_H_XL);
		int16_t accel_y = combine(fifo_sample.OUTY_L_XL, fifo_sample.OUTY_H_XL);
		int16_t accel_z = combine(fifo_sample.OUTZ_L_XL, fifo_sample.OUTZ_H_XL);

		accel.x[i] = accel_x;
		accel.y[i] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
		accel.z[i] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
	}

	_px4_accel.updateFIFO(accel);

	// get current temperature at 1 Hz
	if (timestamp_sample - _temperature_update_timestamp > 1_s) {
		_temperature_update_timestamp = timestamp_sample;
		uint8_t temperature_buf[3] {};
		temperature_buf[0] = static_cast<uint8_t>(Register::OUT_TEMP_L) | DIR_READ;

		if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) == PX4_OK) {
			// 16 bits in two’s complement format with a sensitivity of 256 LSB/°C. The output zero level corresponds to 25 °C.
			const int16_t OUT_TEMP = combine(temperature_buf[1], temperature_buf[2]);
			const float temperature = ((float)OUT_TEMP / 256.0f) + 25.0f;

			_px4_accel.set_temperature(temperature);
			_px4_gyro.set_temperature(temperature);

		} else {
			bad_data = true;
		}
	}

	return !bad_data;
}

void ISM330DLC::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("FIFO empty interval: %d us (%.3f Hz)", _fifo_empty_interval_us,
		 static_cast<double>(1000000 / _fifo_empty_interval_us));

	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_interval_perf);
	perf_print_counter(_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_count_perf);
	perf_print_counter(_drdy_interval_perf);
	perf_print_counter(_bad_register_perf);

	_px4_accel.print_status();
	_px4_gyro.print_status();
}
