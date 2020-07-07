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
 * @file FlightTaskManualAcceleration.cpp
 */

#include "FlightTaskManualAcceleration.hpp"
#include <mathlib/mathlib.h>
#include <float.h>
#include "ControlMath.hpp"
#include <ecl/geo/geo.h>

using namespace matrix;

bool FlightTaskManualAcceleration::activate(vehicle_local_position_setpoint_s last_setpoint)
{
	bool ret = FlightTaskManualAltitudeSmoothVel::activate(last_setpoint);

	if (PX4_ISFINITE(last_setpoint.vx)) {
		_velocity_setpoint.xy() = Vector2f(last_setpoint.vx, last_setpoint.vy);

	} else {
		_velocity_setpoint.xy() = Vector2f(_velocity);
	}

	if (PX4_ISFINITE(last_setpoint.acceleration[0])) {
		_acceleration_slew_rate_x.setForcedValue(last_setpoint.acceleration[0]);
		_acceleration_slew_rate_x.setForcedValue(last_setpoint.acceleration[1]);
	}

	_brake_boost_filter.reset(1.f);

	return ret;
}

bool FlightTaskManualAcceleration::update()
{
	bool ret = FlightTaskManualAltitudeSmoothVel::update();

	// maximum commanded acceleration and velocity
	Vector2f acceleration_scale(_param_mpc_acc_hor.get(), _param_mpc_acc_hor.get());
	Vector2f velocity_scale(_param_mpc_vel_manual.get(), _param_mpc_vel_manual.get());

	acceleration_scale *= 2.f; // because of drag the average aceleration is half

	// Yaw
	_position_lock.updateYawFromStick(_yawspeed_setpoint, _yaw_setpoint,
					  _sticks_expo(3) * math::radians(_param_mpc_man_y_max.get()), _yaw, _deltatime);

	// Map stick input to acceleration
	Vector2f stick_xy(&_sticks_expo(0));
	_position_lock.limitStickUnitLengthXY(stick_xy);
	_position_lock.rotateIntoHeadingFrameXY(stick_xy, _yaw, _yaw_setpoint);
	Vector2f acceleration_xy = stick_xy.emult(acceleration_scale);

	applyFeasibilityLimit(acceleration_xy);

	// Add drag to limit speed and brake again
	acceleration_xy -= calculateDrag(acceleration_scale.edivide(velocity_scale));

	_acceleration_setpoint.xy() = acceleration_xy;

	// Generate velocity setpoint by forward integrating commanded acceleration
	Vector2f velocity_xy(_velocity_setpoint);
	velocity_xy += Vector2f(_acceleration_setpoint.xy()) * _deltatime;
	_velocity_setpoint.xy() = velocity_xy;

	lockPosition();

	_constraints.want_takeoff = _checkTakeoff();
	return ret;
}

void FlightTaskManualAcceleration::applyFeasibilityLimit(Vector2f &acceleration)
{
	// Apply jerk limit - acceleration slew rate
	_acceleration_slew_rate_x.setSlewRate(_param_mpc_jerk_max.get());
	_acceleration_slew_rate_y.setSlewRate(_param_mpc_jerk_max.get());
	acceleration(0) = _acceleration_slew_rate_x.update(acceleration(0), _deltatime);
	acceleration(1) = _acceleration_slew_rate_y.update(acceleration(1), _deltatime);
}

Vector2f FlightTaskManualAcceleration::calculateDrag(Vector2f drag_coefficient)
{
	_brake_boost_filter.setParameters(_deltatime, .8f);

	if (Vector2f(&_sticks_expo(0)).norm_squared() < FLT_EPSILON) {
		_brake_boost_filter.update(3.f);

	} else {
		_brake_boost_filter.update(1.f);
	}

	drag_coefficient *= _brake_boost_filter.getState();

	return drag_coefficient.emult(_velocity_setpoint.xy());
}

void FlightTaskManualAcceleration::lockPosition()
{
	if (Vector2f(_velocity_setpoint).norm_squared() < FLT_EPSILON) {
		Vector2f position_xy(_position_setpoint);

		if (!PX4_ISFINITE(position_xy(0))) {
			position_xy = Vector2f(_position);
		}

		position_xy += Vector2f(_velocity_setpoint.xy()) * _deltatime;
		_position_setpoint.xy() = position_xy;

	} else {
		_position_setpoint.xy() = NAN;

	}
}

void FlightTaskManualAcceleration::_ekfResetHandlerPositionXY()
{
	if (PX4_ISFINITE(_position_setpoint(0))) {
		_position_setpoint(0) = _position(0);
		_position_setpoint(1) = _position(1);
	}
}

void FlightTaskManualAcceleration::_ekfResetHandlerVelocityXY()
{
	if (PX4_ISFINITE(_velocity_setpoint(0))) {
		_velocity_setpoint(0) = _velocity(0);
		_velocity_setpoint(1) = _velocity(1);
	}
}

void FlightTaskManualAcceleration::_ekfResetHandlerPositionZ()
{
	if (PX4_ISFINITE(_position_setpoint(2))) {
		_position_setpoint(2) = _position(2);
	}
}

void FlightTaskManualAcceleration::_ekfResetHandlerVelocityZ()
{
	if (PX4_ISFINITE(_velocity_setpoint(2))) {
		_velocity_setpoint(2) = _velocity(2);
	}
}

void FlightTaskManualAcceleration::_ekfResetHandlerHeading(float delta_psi)
{
	if (PX4_ISFINITE(_yaw_setpoint)) {
		_yaw_setpoint += delta_psi;
	}
}
