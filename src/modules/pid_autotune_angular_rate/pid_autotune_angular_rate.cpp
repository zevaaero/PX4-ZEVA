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
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
	reset();
}

PidAutotuneAngularRate::~PidAutotuneAngularRate()
{
	perf_free(_cycle_perf);
}

bool PidAutotuneAngularRate::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void PidAutotuneAngularRate::reset()
{
}

void PidAutotuneAngularRate::updateParams()
{
	ModuleParams::updateParams();
}

void PidAutotuneAngularRate::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// new gyro data needed every iteration
	if (!_vehicle_angular_velocity_sub.updated()) {
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

	perf_begin(_cycle_perf);

	vehicle_angular_velocity_s angular_velocity;
	_vehicle_angular_velocity_sub.copy(&angular_velocity);

	if (_param_atune_start.get()) {
		actuator_controls_s controls;
		_actuator_controls_sub.copy(&controls);

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const float loop_frequency = 1.f / dt;
		_sys_id.setLpfCutoffFrequency(loop_frequency, 30.f); // TODO: use IMU_GYRO_CUTOFF
		_sys_id.setHpfCutoffFrequency(loop_frequency, .5f);
		_sys_id.setForgettingFactor(60.f, dt);

		switch (_axis) {
		default:

		// fallthrough
		case axis::wait_2_s:

		// fallthrough
		case axis::idle:
			break;

		case axis::roll:
			_sys_id.update(controls.control[actuator_controls_s::INDEX_ROLL], angular_velocity.xyz[0]);
			break;

		case axis::pitch:
			_sys_id.update(controls.control[actuator_controls_s::INDEX_PITCH], angular_velocity.xyz[1]);
			break;
		}

		if (hrt_elapsed_time(&_last_publish) > 100_ms) {
			const Vector<float, 5> coeff = _sys_id.getCoefficients();
			const Vector<float, 5> coeff_var = _sys_id.getVariances();

			updateStateMachine(coeff_var, now);

			const Vector3f rate_ff = getIdentificationSignal();

			const Vector3f num(coeff(2), coeff(3), coeff(4));
			const Vector3f den(1.f, coeff(0), coeff(1));
			const Vector3f kid = pid_design::computePidGmvc(num, den, dt);

			pid_autotune_angular_rate_status_s status{};
			status.timestamp = now;
			coeff.copyTo(status.coeff);
			coeff_var.copyTo(status.coeff_var);
			status.kc = kid(0);
			status.ki = kid(1);
			status.kd = kid(2);
			rate_ff.copyTo(status.rate_ff);
			_pid_autotune_angular_rate_status_pub.publish(status);
			_last_publish = now;
		}
	}

	perf_end(_cycle_perf);
}

void PidAutotuneAngularRate::updateStateMachine(const Vector<float, 5> &coeff_var, hrt_abstime now)
{
	// when identifying an axis, check if the estimate has converged
	constexpr float converged_thr = 0.2f;

	switch (_axis) {
	case axis::roll:
		if (areAllSmallerThan(coeff_var, converged_thr)) {
			// wait for the drone to stabilize
			_axis = axis::wait_2_s;
			_state_start_time = now;
			// first step needs to be shorter to keep the drone centered
			_steps_counter = 5;

		}

		break;

	case axis::wait_2_s:
		if (hrt_elapsed_time(&_state_start_time) > 2_s) {
			_axis = axis::pitch;
			_state_start_time = now;
			_sys_id.reset();
			_signal_sign = 1;
		}

		break;

	case axis::pitch:
		if (areAllSmallerThan(coeff_var, converged_thr)) {
			//stop
			_param_atune_start.set(false);
			_param_atune_start.commit();
			_axis = axis::idle;
			_state_start_time = now;
		}

		break;

	default:
	case axis::idle:
		_axis = axis::roll;
		_state_start_time = now;
		_sys_id.reset();
		// first step needs to be shorter to keep the drone centered
		_steps_counter = 5;
		_signal_sign = 1;
		break;
	}

	// In case of convergence timeout or pilot intervention,
	// the identification sequence is aborted immediately
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	if (hrt_elapsed_time(&_state_start_time) > 20_s
	    || (fabsf(manual_control_setpoint.x) > 0.05f)
	    || (fabsf(manual_control_setpoint.y) > 0.05f)) {
		_param_atune_start.set(false);
		_param_atune_start.commit();
		_axis = axis::idle;
	}
}

bool PidAutotuneAngularRate::areAllSmallerThan(Vector<float, 5> vect, float threshold)
{
	return (vect(0) < threshold)
	       && (vect(1) < threshold)
	       && (vect(2) < threshold)
	       && (vect(3) < threshold)
	       && (vect(4) < threshold);
}

const Vector3f PidAutotuneAngularRate::getIdentificationSignal()
{
	if (_steps_counter > 10) {
		_signal_sign = (_signal_sign >= 0) ? -1 : 1;
		_steps_counter = 0;
	}

	_steps_counter++;

	const float signal = float(_signal_sign) * _param_atune_sysid_amp.get();

	Vector3f rate_ff{};

	switch (_axis) {
	default:

	// fallthrough
	case axis::idle:

	// fallthrough
	case axis::wait_2_s:
		break;

	case axis::roll:
		rate_ff(0) = signal;
		break;

	case axis::pitch:
		rate_ff(1) = signal;
		break;
	}

	return rate_ff;
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
