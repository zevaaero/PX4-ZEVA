/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * Automatic handling power monitors
 */

#include "PowerMonitorSelectorAuterion.h"

#include <builtin/builtin.h>
#include <sys/wait.h>

PowerMonitorSelectorAuterion::PowerMonitorSelectorAuterion() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

PowerMonitorSelectorAuterion::~PowerMonitorSelectorAuterion() = default;

bool PowerMonitorSelectorAuterion::init()
{
	int32_t sens_en = 0;
	param_get(param_find("SENS_EN_INA226"), &sens_en);

	if (sens_en == 1) {

		sens_en = 0;
		param_set(param_find("SENS_EN_INA226"), &sens_en);
		const char *stop_argv[] {"ina226", "stop", NULL};
		exec_builtin("ina226", (char **)stop_argv, NULL, 0);
	}

	ScheduleNow();
	return true;
}

void PowerMonitorSelectorAuterion::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	actuator_armed_s actuator_armed{};
	_actuator_armed_sub.copy(&actuator_armed);

	if (actuator_armed.armed) {
		exit_and_cleanup();
		return;
	}

	for (uint32_t i = 0U; i <  SENSORS_NUMBER; ++i) {

		if (!_sensors[i].started) {

			const char *start_argv[] {
				_sensors[i].name,
				"-X", "-b",  _sensors[i].bus_number, "-a", _sensors[i].i2c_addr,
				"-t", "1", "-q", "start", NULL
			};

			int status = PX4_ERROR;
			int pid = exec_builtin(_sensors[i].name, (char **)start_argv, NULL, 0);

			if (pid != -1) {
				waitpid(pid, &status, WUNTRACED);
			}

			float current_shunt_value = 0.0f;
			param_get(param_find("INA226_SHUNT"), &current_shunt_value);

			if (
				(status == PX4_OK) &&
				(fabsf(current_shunt_value - _sensors[i].shunt_value) > FLT_EPSILON)
			) {

				const char *stop_argv[] {
					_sensors[i].name,
					"-X", "-b",  _sensors[i].bus_number, "-a", _sensors[i].i2c_addr,
					"-t", "1", "-q", "stop", NULL
				};

				exec_builtin(_sensors[i].name, (char **)stop_argv, NULL, 0);
				param_set(param_find("INA226_SHUNT"), &(_sensors[i].shunt_value));

				status = PX4_ERROR;
				pid =  exec_builtin(_sensors[i].name, (char **)start_argv, NULL, 0);

				if (pid != -1) {
					waitpid(pid, &status, WUNTRACED);
				}
			}

			if (status == PX4_OK) {
				_sensors[i].started = true;
			}
		}
	}

	ScheduleDelayed(RUN_INTERVAL);
}

int PowerMonitorSelectorAuterion::task_spawn(int argc, char *argv[])
{
	PowerMonitorSelectorAuterion *instance = new PowerMonitorSelectorAuterion();

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

int PowerMonitorSelectorAuterion::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PowerMonitorSelectorAuterion::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for starting and auto-detecting different power monitors.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("power_monitor_selector_auterion", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int power_monitor_selector_auterion_main(int argc, char *argv[])
{
	return PowerMonitorSelectorAuterion::main(argc, argv);
}
