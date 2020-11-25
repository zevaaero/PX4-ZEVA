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
 * @file pid_autotune_angular_rate.cpp
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include "pid_autotune_angular_rate.hpp"

using namespace matrix;
using namespace time_literals;

PidAutotuneAngularRate::PidAutotuneAngularRate() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	reset();
	_sys_id.setLpfCutoffFrequency(_filter_sample_rate, _param_imu_gyro_cutoff.get());
	_sys_id.setHpfCutoffFrequency(_filter_sample_rate, .05f);
	_sys_id.setForgettingFactor(60.f, 1.f / _filter_sample_rate);
}

PidAutotuneAngularRate::~PidAutotuneAngularRate()
{
	perf_free(_cycle_perf);
}

bool PidAutotuneAngularRate::init()
{
	if (!_actuator_controls_sub.registerCallback()) {
		PX4_ERR("actuator_controls callback registration failed!");
		return false;
	}

	return true;
}

void PidAutotuneAngularRate::reset()
{
}

void PidAutotuneAngularRate::Run()
{
	if (should_exit()) {
		_actuator_controls_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// new control data needed every iteration
	if (!_actuator_controls_sub.updated()) {
		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	actuator_controls_s controls;
	vehicle_angular_velocity_s angular_velocity;

	if (!_actuator_controls_sub.copy(&controls)
	    || !_vehicle_angular_velocity_sub.copy(&angular_velocity)) {
		return;
	}

	perf_begin(_cycle_perf);

	const hrt_abstime now = controls.timestamp_sample;

	// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
	const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);

	// collect sample interval average for filters
	if (_last_run > 0) {
		_interval_sum += dt;
		_interval_count++;

	} else {
		_interval_sum = 0.f;
		_interval_count = 0.f;
	}

	_last_run = now;

	checkFilters();

	if (_param_atune_start.get()) {
		switch (_state) {
		default:

		// fallthrough
		case state::roll_pause:

		// fallthrough
		case state::idle:
			break;

		case state::roll:
			_sys_id.update(_input_scale * controls.control[actuator_controls_s::INDEX_ROLL], angular_velocity.xyz[0]);
			break;

		case state::pitch:
			_sys_id.update(_input_scale * controls.control[actuator_controls_s::INDEX_PITCH], angular_velocity.xyz[1]);
			break;
		}

		if (hrt_elapsed_time(&_last_publish) > 100_ms || _last_publish == 0) {
			Vector<float, 5> coeff = _sys_id.getCoefficients();
			coeff(2) *= _input_scale;
			coeff(3) *= _input_scale;
			coeff(4) *= _input_scale;

			const Vector<float, 5> coeff_var = _sys_id.getVariances();

			updateStateMachine(coeff_var, now);

			const Vector3f rate_sp = getIdentificationSignal();

			const Vector3f num(coeff(2), coeff(3), coeff(4));
			const Vector3f den(1.f, coeff(0), coeff(1));
			_kid = pid_design::computePidGmvc(num, den, dt, 0.08f, 0.f, 0.4f);

			pid_autotune_angular_rate_status_s status{};
			status.timestamp = now;
			coeff.copyTo(status.coeff);
			coeff_var.copyTo(status.coeff_var);
			status.innov = _sys_id.getInnovation();
			status.u_filt = _sys_id.getFilteredInputData();
			status.y_filt = _sys_id.getFilteredOutputData();
			status.kc = _kid(0);
			status.ki = _kid(1);
			status.kd = _kid(2);
			rate_sp.copyTo(status.rate_sp);
			status.state = static_cast<int>(_state);
			_pid_autotune_angular_rate_status_pub.publish(status);

			_last_publish = now;
		}

	} else {
		_state = state::idle;
	}

	perf_end(_cycle_perf);
}

void PidAutotuneAngularRate::checkFilters()
{
	if (_interval_count > 1000) {
		// calculate sensor update rate
		const float sample_interval_avg = _interval_sum / _interval_count;
		const float update_rate_hz = 1.f / sample_interval_avg;

		// check if sample rate error is greater than 1%
		bool reset_filters = false;

		if ((fabsf(update_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
			reset_filters = true;
		}

		if (reset_filters) {
			_filter_sample_rate = update_rate_hz;
			_sys_id.setLpfCutoffFrequency(_filter_sample_rate, _param_imu_gyro_cutoff.get());
			_sys_id.setHpfCutoffFrequency(_filter_sample_rate, .05f);
			_sys_id.setForgettingFactor(60.f, sample_interval_avg);
		}
	}
}

void PidAutotuneAngularRate::updateStateMachine(const Vector<float, 5> &coeff_var, hrt_abstime now)
{
	// when identifying an axis, check if the estimate has converged
	const float converged_thr = 2.f * _input_scale;

	switch (_state) {
	case state::roll:
		if (areAllSmallerThan(coeff_var, converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains();

			// wait for the drone to stabilize
			_state = state::roll_pause;
			_state_start_time = now;
		}

		break;

	case state::roll_pause:
		if ((now - _state_start_time) > 2_s) {
			_state = state::pitch;
			_state_start_time = now;
			_sys_id.reset();
			_input_scale = 1.f / (_param_mc_pitchrate_p.get() * _param_mc_pitchrate_k.get());
			_signal_sign = 1;
			// first step needs to be shorter to keep the drone centered
			_steps_counter = 5;
			_max_steps = 10;
		}

		break;

	case state::pitch:
		if (areAllSmallerThan(coeff_var, converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains();
			_state = state::verification;
			_state_start_time = now;
		}

		break;

	case state::pitch_pause:

	// fallthrough
	case state::yaw:

	// fallthrough
	case state::yaw_pause:

	// fallthrough
	case state::verification:
		_state = areGainsGood()
			 ? state::complete
			 : state::fail;

		_state_start_time = now;
		break;

	case state::complete:
		if ((now - _state_start_time) > 2_s) {
			if (((_param_atune_apply.get() == 1) && !_armed)
			    || (_param_atune_apply.get() == 2)) {
				saveGainsToParams();
				_state = state::idle;
				_param_atune_start.set(false);
				_param_atune_start.commit();

			} else if (_param_atune_apply.get() == 0) {
				_state = state::idle;
				_param_atune_start.set(false);
				_param_atune_start.commit();
			}
		}

		break;

	// fallthrough
	case state::fail:
		if ((now - _state_start_time) > 2_s) {
			_state = state::idle;
			_param_atune_start.set(false);
			_param_atune_start.commit();
		}

		break;

	default:
	case state::idle:
		_state = state::roll;
		_state_start_time = now;
		_sys_id.reset();
		// first step needs to be shorter to keep the drone centered
		_steps_counter = 5;
		_max_steps = 10;
		_signal_sign = 1;
		_input_scale = 1.f / (_param_mc_rollrate_p.get() * _param_mc_rollrate_k.get());
		break;
	}

	// In case of convergence timeout or pilot intervention,
	// the identification sequence is aborted immediately
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	if (_state != state::complete
	    && (((now - _state_start_time) > 20_s)
		|| (fabsf(manual_control_setpoint.x) > 0.05f)
		|| (fabsf(manual_control_setpoint.y) > 0.05f))) {
		_state = state::fail;
	}
}

bool PidAutotuneAngularRate::areAllSmallerThan(Vector<float, 5> vect, float threshold) const
{
	return (vect(0) < threshold)
	       && (vect(1) < threshold)
	       && (vect(2) < threshold)
	       && (vect(3) < threshold)
	       && (vect(4) < threshold);
}

void PidAutotuneAngularRate::copyGains()
{
	int index = -1;

	switch (_state) {
	default:
		break;

	case state::roll:
		index = 0;

		break;

	case state::pitch:
		index = 1;
		break;
	}

	if (index >= 0) {
		_rate_k(index) = _kid(0);
		_rate_i(index) = _kid(1);
		_rate_d(index) = _kid(2);
	}
}

bool PidAutotuneAngularRate::areGainsGood() const
{
	// TODO: check yaw gains as well (remove Vector2f cast)
	const bool are_positive = Vector2f(_rate_k).min() > 0.f
				  && Vector2f(_rate_i).min() > 0.f
				  && Vector2f(_rate_d).min() > 0.f;

	const bool are_small_enough = Vector2f(_rate_k).max() < 0.5f
				      && Vector2f(_rate_i).max() < 10.f
				      && Vector2f(_rate_d).max() < 0.1f;

	return are_positive && are_small_enough;
}

void PidAutotuneAngularRate::saveGainsToParams()
{
	_param_mc_rollrate_p.set(1.f);
	_param_mc_rollrate_k.set(_rate_k(0));
	_param_mc_rollrate_i.set(_rate_i(0));
	_param_mc_rollrate_d.set(_rate_d(0));
	_param_mc_rollrate_p.commit_no_notification();
	_param_mc_rollrate_k.commit_no_notification();
	_param_mc_rollrate_i.commit_no_notification();
	_param_mc_rollrate_d.commit_no_notification();

	_param_mc_pitchrate_p.set(1.f);
	_param_mc_pitchrate_k.set(_rate_k(1));
	_param_mc_pitchrate_i.set(_rate_i(1));
	_param_mc_pitchrate_d.set(_rate_d(1));
	_param_mc_pitchrate_p.commit_no_notification();
	_param_mc_pitchrate_k.commit_no_notification();
	_param_mc_pitchrate_i.commit_no_notification();
	_param_mc_pitchrate_d.commit();

	//TODO: save yawrate gains
	/* _param_mc_yawrate_p.set(1.f); */
	/* _param_mc_yawrate_k.set(_rate_k(2)); */
	/* _param_mc_yawrate_i.set(_rate_i(2)); */
	/* _param_mc_yawrate_d.set(_rate_d(2)); */
}

const Vector3f PidAutotuneAngularRate::getIdentificationSignal()
{
	if (_steps_counter > _max_steps) {
		_signal_sign = (_signal_sign >= 0) ? -1 : 1;
		_steps_counter = 0;

		if (_max_steps > 1) {
			_max_steps--;

		} else {
			_max_steps = 5;
		}
	}

	_steps_counter++;

	const float signal = float(_signal_sign) * _param_atune_sysid_amp.get();

	Vector3f rate_sp{};

	switch (_state) {
	default:

	// fallthrough
	case state::idle:

	// fallthrough
	case state::roll_pause:

	// fallthrough
	case state::pitch_pause:

	// fallthrough
	case state::yaw:

	// fallthrough
	case state::yaw_pause:

	// fallthrough
	case state::verification:
		break;

	case state::roll:
		rate_sp(0) = signal;
		break;

	case state::pitch:
		rate_sp(1) = signal;
		break;
	}

	return rate_sp;
}

int PidAutotuneAngularRate::task_spawn(int argc, char *argv[])
{
	PidAutotuneAngularRate *instance = new PidAutotuneAngularRate();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int PidAutotuneAngularRate::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PidAutotuneAngularRate::print_status()
{
	perf_print_counter(_cycle_perf);

	return 0;
}

int PidAutotuneAngularRate::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pid_autotune_angular_rate", "autotune");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pid_autotune_angular_rate_main(int argc, char *argv[])
{
	return PidAutotuneAngularRate::main(argc, argv);
}
