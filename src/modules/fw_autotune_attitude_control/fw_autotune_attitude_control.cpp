/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file fw_autotune_attitude_control.cpp
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include "fw_autotune_attitude_control.hpp"

using namespace matrix;
using namespace time_literals;

FwAutotuneAttitudeControl::FwAutotuneAttitudeControl(bool is_vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_actuator_controls_sub(this, is_vtol ? ORB_ID(actuator_controls_1) : ORB_ID(actuator_controls_0))
{
	reset();
}

FwAutotuneAttitudeControl::~FwAutotuneAttitudeControl()
{
	perf_free(_cycle_perf);
}

bool FwAutotuneAttitudeControl::init()
{
	if (!_parameter_update_sub.registerCallback()) {
		PX4_ERR("parameter_update callback registration failed!");
		return false;
	}

	_signal_filter.setParameters(100e-3f, 1.f); // runs in the slow 10Hz loop

	return true;
}

void FwAutotuneAttitudeControl::reset()
{
}

void FwAutotuneAttitudeControl::Run()
{
	if (should_exit()) {
		_parameter_update_sub.unregisterCallback();
		_actuator_controls_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		updateStateMachine(hrt_absolute_time());
	}

	// new control data needed every iteration
	if (_state == state::idle
	    || !_actuator_controls_sub.updated()) {
		return;
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

	const hrt_abstime now = controls.timestamp;

	// collect sample interval average for filters
	if (_last_run > 0) {
		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_interval_sum += dt;
		_interval_count++;

	} else {
		_interval_sum = 0.f;
		_interval_count = 0.f;
	}

	_last_run = now;

	checkFilters();

	if (_state == state::roll) {
		_sys_id.update(_input_scale * controls.control[actuator_controls_s::INDEX_ROLL],
			       angular_velocity.xyz[0]);

	} else if (_state == state::pitch) {
		_sys_id.update(_input_scale * controls.control[actuator_controls_s::INDEX_PITCH],
			       angular_velocity.xyz[1]);

	} else if (_state == state::yaw) {
		_sys_id.update(_input_scale * controls.control[actuator_controls_s::INDEX_YAW],
			       angular_velocity.xyz[2]);
	}

	if (hrt_elapsed_time(&_last_publish) > 100_ms || _last_publish == 0) {
		updateStateMachine(now);

		Vector<float, 5> coeff = _sys_id.getCoefficients();
		coeff(2) *= _input_scale;
		coeff(3) *= _input_scale;
		coeff(4) *= _input_scale;

		const Vector3f num(coeff(2), coeff(3), coeff(4));
		const Vector3f den(1.f, coeff(0), coeff(1));
		_kiff(2) = (1.f + coeff(0) + coeff(1)) / (coeff(2) + coeff(3) + coeff(4)); // inverse of the static gain
		const Vector3f num_design = num * _kiff(2); // PID algorithm design works better with systems having unit static gain
		Vector3f kid = pid_design::computePidGmvc(num_design, den, _sample_interval_avg, 0.08f, 0.f, 0.4f);
		_kiff(0) = kid(0);
		_kiff(1) = kid(1);
		_attitude_p = 8.f / (M_PI_F * (_kiff(2) + _kiff(0))); // Maximum control power at an attitude error of pi/8

		const Vector<float, 5> &coeff_var = _sys_id.getVariances();
		const Vector3f &rate_sp = getIdentificationSignal();

		autotune_attitude_control_status_s status{};
		status.timestamp = now;
		coeff.copyTo(status.coeff);
		coeff_var.copyTo(status.coeff_var);
		status.fitness = _sys_id.getFitness();
		status.dt_model = _sample_interval_avg;
		status.innov = _sys_id.getInnovation();
		status.u_filt = _sys_id.getFilteredInputData();
		status.y_filt = _sys_id.getFilteredOutputData();
		status.kc = _kiff(0);
		status.ki = _kiff(1);
		status.kd = kid(2); // FW rate controller has no derivative gain
		status.kff = _kiff(2);
		status.att_p = _attitude_p;
		rate_sp.copyTo(status.rate_sp);
		status.state = static_cast<int>(_state);
		_autotune_attitude_control_status_pub.publish(status);

		_last_publish = now;
	}

	perf_end(_cycle_perf);
}

void FwAutotuneAttitudeControl::checkFilters()
{
	if (_interval_count > 1000) {
		// calculate sensor update rate
		_sample_interval_avg = _interval_sum / _interval_count;
		const float update_rate_hz = 1.f / _sample_interval_avg;

		// check if sample rate error is greater than 1%
		bool reset_filters = false;

		if ((fabsf(update_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
			reset_filters = true;
		}

		if (reset_filters) {
			_are_filters_initialized = true;
			_filter_sample_rate = update_rate_hz;
			_sys_id.setLpfCutoffFrequency(_filter_sample_rate, _param_imu_gyro_cutoff.get());
			_sys_id.setHpfCutoffFrequency(_filter_sample_rate, .05f);
			_sys_id.setForgettingFactor(60.f, _sample_interval_avg);
			_sys_id.setFitnessLpfTimeConstant(1.f, _sample_interval_avg);
		}

		// reset sample interval accumulator
		_last_run = 0;
	}
}

void FwAutotuneAttitudeControl::updateStateMachine(hrt_abstime now)
{
	// when identifying an axis, check if the estimate has converged
	const float converged_thr = 1.f;

	const float temp[5] = {0.f, 0.f, 0.f, 0.f, 0.f};
	const Vector<float, 5> sys_id_init(temp);

	switch (_state) {
	case state::idle:
		if (_param_fw_at_start.get()) {
			if (registerActuatorControlsCallback()) {
				_state = state::init;

			} else {
				_state = state::fail;
			}

			_state_start_time = now;
		}

		break;

	case state::init:
		if (_are_filters_initialized) {
			_state = state::roll;
			_state_start_time = now;
			_sys_id.reset(sys_id_init);
			// first step needs to be shorter to keep the drone centered
			_steps_counter = 5;
			_max_steps = 10;
			_signal_sign = 1;
			_input_scale = 1.f / _param_fw_rr_p.get();
			_signal_filter.reset(0.f);
		}

		break;

	case state::roll:
		if ((_sys_id.getFitness() < converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains(0);

			// wait for the drone to stabilize
			_state = state::roll_pause;
			_state_start_time = now;
		}

		break;

	case state::roll_pause:
		if ((now - _state_start_time) > 2_s) {
			_state = state::pitch;
			_state_start_time = now;
			_sys_id.reset(sys_id_init);
			_input_scale = 1.f / _param_fw_pr_p.get();
			_signal_sign = 1;
			// first step needs to be shorter to keep the drone centered
			_steps_counter = 5;
			_max_steps = 10;
			_signal_filter.reset(0.f);
		}

		break;

	case state::pitch:
		if ((_sys_id.getFitness() < converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains(1);
			_state = state::pitch_pause;
			_state_start_time = now;
		}

		break;

	case state::pitch_pause:
		if ((now - _state_start_time) > 2_s) {
			_state = state::yaw;
			_state_start_time = now;
			_sys_id.reset(sys_id_init);
			_input_scale = 1.f / _param_fw_yr_p.get();
			_signal_sign = 1;
			// first step needs to be shorter to keep the drone centered
			_steps_counter = 5;
			_max_steps = 10;

			// reset yaw signal filter states
			_signal_filter.reset(0.f);
		}

		break;

	case state::yaw:

		if ((_sys_id.getFitness() < converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains(2);
			_state = state::yaw_pause;
			_state_start_time = now;
		}

		break;

	// fallthrough
	case state::yaw_pause:
		_state = state::verification;

	// fallthrough
	case state::verification:
		_state = areGainsGood()
			 ? state::complete
			 : state::fail;

		_state_start_time = now;
		break;

	case state::complete:
		if ((now - _state_start_time) > 2_s) {
			if (((_param_fw_at_apply.get() == 1) && !_armed)
			    || (_param_fw_at_apply.get() == 2)) {
				saveGainsToParams();
				_state = state::idle;
				stopAutotune();

			} else if (_param_fw_at_apply.get() == 0) {
				_state = state::idle;
				stopAutotune();
			}
		}

		break;

	case state::fail:
		if ((now - _state_start_time) > 2_s) {
			_state = state::idle;
			stopAutotune();
		}

		break;
	}

	// In case of convergence timeout or pilot intervention,
	// the identification sequence is aborted immediately
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	if (_state != state::complete
	    && _state != state::idle
	    && (((now - _state_start_time) > 20_s)
		|| (fabsf(manual_control_setpoint.x) > 0.05f)
		|| (fabsf(manual_control_setpoint.y) > 0.05f))) {
		_state = state::fail;
		_state_start_time = now;
	}
}

bool FwAutotuneAttitudeControl::registerActuatorControlsCallback()
{
	if (!_actuator_controls_sub.registerCallback()) {
		PX4_ERR("actuator_controls callback registration failed!");
		return false;
	}

	return true;
}

void FwAutotuneAttitudeControl::copyGains(int index)
{
	if (index <= 2) {
		_rate_k(index) = _kiff(0);
		_rate_i(index) = _kiff(1);
		_rate_ff(index) = _kiff(2);
		_att_p(index) = _attitude_p;
	}
}

bool FwAutotuneAttitudeControl::areGainsGood() const
{
	const bool are_positive = _rate_k.min() > 0.f
				  && _rate_i.min() > 0.f
				  && _rate_ff.min() > 0.f
				  && _att_p.min() > 0.f;

	const bool are_small_enough = _rate_k.max() < 0.5f
				      && _rate_i.max() < 10.f
				      && _rate_ff.max() < 2.f
				      && _att_p.max() < 12.f;

	return are_positive && are_small_enough;
}

void FwAutotuneAttitudeControl::saveGainsToParams()
{
	_param_fw_rr_p.set(_rate_k(0));
	_param_fw_rr_i.set(_rate_k(0) * _rate_i(0));
	_param_fw_rr_ff.set(_rate_ff(0));
	_param_fw_r_tc.set(1.f / _att_p(0));
	_param_fw_rr_p.commit_no_notification();
	_param_fw_rr_i.commit_no_notification();
	_param_fw_rr_ff.commit_no_notification();
	_param_fw_r_tc.commit_no_notification();

	_param_fw_pr_p.set(_rate_k(1));
	_param_fw_pr_i.set(_rate_k(1) * _rate_i(1));
	_param_fw_pr_ff.set(_rate_ff(1));
	_param_fw_p_tc.set(1.f / _att_p(1));
	_param_fw_pr_p.commit_no_notification();
	_param_fw_pr_i.commit_no_notification();
	_param_fw_pr_ff.commit_no_notification();
	_param_fw_p_tc.commit_no_notification();

	_param_fw_yr_p.set(_rate_k(2));
	_param_fw_yr_i.set(_rate_k(2) * _rate_i(2));
	_param_fw_yr_ff.set(_rate_ff(2));
	_param_fw_yr_p.commit_no_notification();
	_param_fw_yr_i.commit_no_notification();
	_param_fw_yr_ff.commit();
}

void FwAutotuneAttitudeControl::stopAutotune()
{
	_param_fw_at_start.set(false);
	_param_fw_at_start.commit();
	_actuator_controls_sub.unregisterCallback();
}

const Vector3f FwAutotuneAttitudeControl::getIdentificationSignal()
{
	if (_steps_counter > _max_steps) {
		_signal_sign = (_signal_sign == 1) ? 0 : 1;
		_steps_counter = 0;

		if (_max_steps > 1) {
			_max_steps--;

		} else {
			_max_steps = 5;
		}
	}

	_steps_counter++;

	const float signal = float(_signal_sign) * _param_fw_at_sysid_amp.get();

	Vector3f rate_sp{};

	if (_state == state::roll) {
		// Scale the signal such that the attitude controller is
		// able to cancel it completely at an attitude error of pi/8
		const float signal_scaled = signal * M_PI_F / (8.f * _param_fw_r_tc.get());
		rate_sp(0) = signal_scaled - _signal_filter.getState();
		_signal_filter.update(signal_scaled);

	} else if (_state ==  state::pitch) {
		const float signal_scaled = signal * M_PI_F / (8.f * _param_fw_p_tc.get());
		rate_sp(1) = signal_scaled - _signal_filter.getState();
		_signal_filter.update(signal_scaled);

	} else if (_state ==  state::yaw) {
		rate_sp(2) = signal - _signal_filter.getState();
		_signal_filter.update(signal);
	}

	return rate_sp;
}

int FwAutotuneAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool is_vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			is_vtol = true;
		}
	}

	FwAutotuneAttitudeControl *instance = new FwAutotuneAttitudeControl(is_vtol);

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

int FwAutotuneAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FwAutotuneAttitudeControl::print_status()
{
	perf_print_counter(_cycle_perf);

	return 0;
}

int FwAutotuneAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_autotune_attitude_control", "autotune");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_autotune_attitude_control_main(int argc, char *argv[])
{
	return FwAutotuneAttitudeControl::main(argc, argv);
}
