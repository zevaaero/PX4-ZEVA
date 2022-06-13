/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file RunwayTakeoff.cpp
 * Runway takeoff handling for fixed-wing UAVs with steerable wheels.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "RunwayTakeoff.h"
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/events.h>

using namespace time_literals;

namespace runwaytakeoff
{

void RunwayTakeoff::init(const hrt_abstime &time_now, const float initial_yaw, const matrix::Vector2d &start_pos_global)
{
	_initial_yaw = initial_yaw;
	_start_pos_global = start_pos_global;
	_takeoff_state = RunwayTakeoffState::THROTTLE_RAMP;
	_climbout = true; // this is true until climbout is finished
	_initialized = true;
	_time_initialized = time_now;
}

void RunwayTakeoff::update(const hrt_abstime &time_now, const float calibrated_airspeed, const float vehicle_altitude,
			   const float clearance_altitude, orb_advert_t *mavlink_log_pub)
{
	switch (_takeoff_state) {
	case RunwayTakeoffState::THROTTLE_RAMP:
		if ((time_now - _time_initialized) > (_param_rwto_ramp_time.get() * 1_s)) {
			_takeoff_state = RunwayTakeoffState::CLAMPED_TO_RUNWAY;
		}

		break;

	case RunwayTakeoffState::CLAMPED_TO_RUNWAY:
		if (calibrated_airspeed > _param_fw_airspd_min.get() * _param_rwto_airspd_scl.get()) {
			_takeoff_state = RunwayTakeoffState::CLIMBOUT;
			mavlink_log_info(mavlink_log_pub, "Takeoff airspeed reached, climbout\t");
			events::send(events::ID("runway_takeoff_reached_airspeed"), events::Log::Info,
				     "Takeoff airspeed reached, climbout");
		}

		break;

	case RunwayTakeoffState::CLIMBOUT:
		if (vehicle_altitude > clearance_altitude) {
			_climbout = false;
			_takeoff_state = RunwayTakeoffState::FLY;
			mavlink_log_info(mavlink_log_pub, "Reached clearance altitude\t");
			events::send(events::ID("runway_takeoff_reached_clearance_altitude"), events::Log::Info, "Reached clearance altitude");
		}

		break;

	default:
		break;
	}
}

bool RunwayTakeoff::controlYaw()
{
	// keep controlling yaw directly until we start navigation
	return _takeoff_state < RunwayTakeoffState::CLIMBOUT;
}

float RunwayTakeoff::getPitch(float external_pitch_setpoint)
{
	if (_takeoff_state <= RunwayTakeoffState::CLAMPED_TO_RUNWAY) {
		return math::radians(_param_rwto_psp.get());
	}

	return external_pitch_setpoint;
}

float RunwayTakeoff::getRoll(float external_roll_setpoint)
{
	// until we have enough ground clearance, set roll to 0
	if (_takeoff_state < RunwayTakeoffState::CLIMBOUT) {
		return 0.0f;
	}

	return external_roll_setpoint;
}

float RunwayTakeoff::getYaw(float external_yaw_setpoint)
{
	if (_param_rwto_hdg.get() == 0 && _takeoff_state < RunwayTakeoffState::CLIMBOUT) {
		return _initial_yaw;

	} else {
		return external_yaw_setpoint;
	}
}

float RunwayTakeoff::getThrottle(const hrt_abstime &time_now, float external_throttle_setpoint)
{
	switch (_takeoff_state) {
	case RunwayTakeoffState::THROTTLE_RAMP: {
			float throttle = ((time_now - _time_initialized) / (_param_rwto_ramp_time.get() * 1_s)) * _param_rwto_max_thr.get();
			return math::min(throttle, _param_rwto_max_thr.get());
		}

	case RunwayTakeoffState::CLAMPED_TO_RUNWAY:
		return _param_rwto_max_thr.get();

	default:
		return external_throttle_setpoint;
	}
}

bool RunwayTakeoff::resetIntegrators()
{
	// reset integrators if we're still on runway
	return _takeoff_state < RunwayTakeoffState::CLIMBOUT;
}

float RunwayTakeoff::getMinPitch(float min_pitch_in_climbout, float min_pitch)
{
	if (_takeoff_state < RunwayTakeoffState::FLY) {
		return min_pitch_in_climbout;

	} else {
		return min_pitch;
	}
}

float RunwayTakeoff::getMaxPitch(const float max_pitch)
{
	// use max pitch from parameter if set
	if (_takeoff_state < RunwayTakeoffState::FLY && _param_rwto_max_pitch.get() > kMinMaxPitch) {
		return _param_rwto_max_pitch.get();

	} else {
		return max_pitch;
	}
}

void RunwayTakeoff::reset()
{
	_initialized = false;
	_takeoff_state = RunwayTakeoffState::THROTTLE_RAMP;
}

} // namespace runwaytakeoff
