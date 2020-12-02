/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
* @file FailureDetector.cpp
*
* @author Mathieu Bresciani	<brescianimathieu@gmail.com>
*
*/

#include "FailureDetector.hpp"

using namespace time_literals;

FailureDetector::FailureDetector(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool FailureDetector::update(const vehicle_status_s &vehicle_status)
{
	uint8_t previous_status = _status;

	if (isAttitudeStabilized(vehicle_status)) {
		updateAttitudeStatus();

		if (_param_fd_ext_ats_en.get()) {
			updateExternalAtsStatus();
		}

	} else {
		_status &= ~(FAILURE_ROLL | FAILURE_PITCH | FAILURE_ALT | FAILURE_EXT);
	}

	if (_param_batteries_en.get()) {
		updateBatteryStatus(vehicle_status);
	}

	if (_param_escs_en.get()) {
		updateEscsStatus(vehicle_status);
	}

	if (_param_fd_wind_max.get() > FLT_EPSILON) {
		updateHighWindStatus();
	}

	return _status != previous_status;
}

bool FailureDetector::isAttitudeStabilized(const vehicle_status_s &vehicle_status)
{
	bool attitude_is_stabilized{false};
	const uint8_t vehicle_type = vehicle_status.vehicle_type;
	const uint8_t nav_state = vehicle_status.nav_state;

	if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		attitude_is_stabilized =  nav_state != vehicle_status_s::NAVIGATION_STATE_ACRO &&
					  nav_state != vehicle_status_s::NAVIGATION_STATE_RATTITUDE;

	} else if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		attitude_is_stabilized =  nav_state != vehicle_status_s::NAVIGATION_STATE_MANUAL &&
					  nav_state != vehicle_status_s::NAVIGATION_STATE_ACRO &&
					  nav_state != vehicle_status_s::NAVIGATION_STATE_RATTITUDE;
	}

	return attitude_is_stabilized;
}

void FailureDetector::updateAttitudeStatus()
{
	vehicle_attitude_s attitude;

	if (_vehicule_attitude_sub.update(&attitude)) {

		const matrix::Eulerf euler(matrix::Quatf(attitude.q));
		const float roll(euler.phi());
		const float pitch(euler.theta());

		const float max_roll_deg = _param_fd_fail_r.get();
		const float max_pitch_deg = _param_fd_fail_p.get();
		const float max_roll(fabsf(math::radians(max_roll_deg)));
		const float max_pitch(fabsf(math::radians(max_pitch_deg)));

		const bool roll_status = (max_roll > 0.0f) && (fabsf(roll) > max_roll);
		const bool pitch_status = (max_pitch > 0.0f) && (fabsf(pitch) > max_pitch);

		hrt_abstime time_now = hrt_absolute_time();

		// Update hysteresis
		_roll_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_r_ttri.get()));
		_pitch_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_p_ttri.get()));
		_roll_failure_hysteresis.set_state_and_update(roll_status, time_now);
		_pitch_failure_hysteresis.set_state_and_update(pitch_status, time_now);

		// Update bitmask
		_status &= ~(FAILURE_ROLL | FAILURE_PITCH);

		if (_roll_failure_hysteresis.get_state()) {
			_status |= FAILURE_ROLL;
		}

		if (_pitch_failure_hysteresis.get_state()) {
			_status |= FAILURE_PITCH;
		}
	}
}

void FailureDetector::updateExternalAtsStatus()
{
	pwm_input_s pwm_input;

	if (_pwm_input_sub.update(&pwm_input)) {

		uint32_t pulse_width = pwm_input.pulse_width;
		bool ats_trigger_status = (pulse_width >= (uint32_t)_param_fd_ext_ats_trig.get()) && (pulse_width < 3_ms);

		hrt_abstime time_now = hrt_absolute_time();

		// Update hysteresis
		_ext_ats_failure_hysteresis.set_hysteresis_time_from(false, 100_ms); // 5 consecutive pulses at 50hz
		_ext_ats_failure_hysteresis.set_state_and_update(ats_trigger_status, time_now);

		_status &= ~FAILURE_EXT;

		if (_ext_ats_failure_hysteresis.get_state()) {
			_status |= FAILURE_EXT;
		}
	}
}

void FailureDetector::updateBatteryStatus(const vehicle_status_s &vehicle_status)
{
	hrt_abstime time_now = hrt_absolute_time();

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		battery_status_s battery_status;

		if (_battery_status_sub.update(&battery_status)) {
			bool battery_nominal = (battery_status.mode == battery_status_s::BATTERY_MODE_UNKNOWN
						&& battery_status.faults == battery_status_s::BATTERY_FAULT_NONE);

			_battery_failure_hysteresis.set_hysteresis_time_from(false, 300_ms);
			_battery_failure_hysteresis.set_state_and_update(battery_nominal, time_now);

			if (_battery_failure_hysteresis.get_state()) {
				_status |= FAILURE_BATTERY;
			}
		}

	} else {
		// reset battery bitfield
		_battery_failure_hysteresis.set_state_and_update(false, time_now);
		_status &= ~FAILURE_BATTERY;
	}
}

void FailureDetector::updateEscsStatus(const vehicle_status_s &vehicle_status)
{
	hrt_abstime time_now = hrt_absolute_time();

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		esc_status_s esc_status;

		if (_esc_status_sub.update(&esc_status)) {
			const int limited_esc_count = math::min(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX);
			const int all_escs_armed_mask = (1 << limited_esc_count) - 1;
			const bool is_all_escs_armed = (all_escs_armed_mask == esc_status.esc_armed_flags);

			bool is_esc_failure = !is_all_escs_armed;

			for (uint8_t i = 0; i < limited_esc_count; i++) {
				is_esc_failure |= (esc_status.esc[i].failures != esc_report_s::FAILURE_NONE);
			}

			_esc_failure_hysteresis.set_hysteresis_time_from(false, 300_ms);
			_esc_failure_hysteresis.set_state_and_update(is_esc_failure, time_now);

			if (_esc_failure_hysteresis.get_state()) {
				_status |= FAILURE_ARM_ESCS;
			}
		}

	} else {
		// reset ESC bitfield
		_esc_failure_hysteresis.set_state_and_update(false, time_now);
		_status &= ~FAILURE_ARM_ESCS;
	}
}

void FailureDetector::updateHighWindStatus()
{
	wind_estimate_s wind_estimate;

	if (_wind_estimate_sub.update(&wind_estimate)) {
		const matrix::Vector2f wind(wind_estimate.windspeed_north, wind_estimate.windspeed_east);

		const bool high_wind = wind.longerThan(_param_fd_wind_max.get());
		_status = (_status & ~FAILURE_HIGH_WIND) | (high_wind * FAILURE_HIGH_WIND);
	}
}
