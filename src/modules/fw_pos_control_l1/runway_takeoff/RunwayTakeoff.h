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
 * @file RunwayTakeoff.h
 * Runway takeoff handling for fixed-wing UAVs with steerable wheels.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#ifndef RUNWAYTAKEOFF_H
#define RUNWAYTAKEOFF_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace runwaytakeoff
{

enum RunwayTakeoffState {
	THROTTLE_RAMP = 0, // ramping up throttle
	CLAMPED_TO_RUNWAY, // clamped to runway, controlling yaw directly (wheel or rudder)
	CLIMBOUT, // climbout to safe height before navigation
	FLY // hold until operator starts mission or proceed automatically
};

class __EXPORT RunwayTakeoff : public ModuleParams
{
public:
	RunwayTakeoff(ModuleParams *parent) : ModuleParams(parent) {}
	~RunwayTakeoff() = default;

	/**
	 * @brief Initializes the state machine.
	 *
	 * @param time_now Absolute time since system boot [us]
	 * @param initial_yaw Vehicle yaw angle at time of initialization [us]
	 * @param start_pos_global Vehicle global (lat, lon) position at time of initialization [deg]
	 */
	void init(const hrt_abstime &time_now, const float initial_yaw, const matrix::Vector2d &start_pos_global);

	/**
	 * @brief Updates the state machine based on the current vehicle condition.
	 *
	 * @param time_now Absolute time since system boot [us]
	 * @param calibrated_airspeed Vehicle calibrated airspeed [m/s]
	 * @param vehicle_altitude Vehicle altitude (AGL) [m]
	 * @param clearance_altitude Altitude (AGL) above which we have cleared all occlusions in the runway path [m]
	 * @param mavlink_log_pub
	 */
	void update(const hrt_abstime &time_now, const float calibrated_airspeed, const float vehicle_altitude,
		    const float clearance_altitude, orb_advert_t *mavlink_log_pub);

	/**
	 * @return Current takeoff state
	 */
	RunwayTakeoffState getState() { return _takeoff_state; }

	/**
	 * @return The state machine is initialized
	 */
	bool isInitialized() { return _initialized; }

	/**
	 * @return Runway takeoff is enabled
	 */
	bool runwayTakeoffEnabled() { return _param_rwto_tkoff.get(); }

	/**
	 * @return Scale factor for minimum indicated airspeed
	 */
	float getMinAirspeedScaling() { return _param_rwto_airspd_scl.get(); }

	/**
	 * @return Initial vehicle yaw angle [rad]
	 */
	float getInitYaw() { return _initial_yaw; }

	/**
	 * @return The vehicle should control yaw via rudder or nose gear
	 */
	bool controlYaw();

	/**
	 * @return TECS should be commanded to climbout mode
	 */
	bool climbout() { return _climbout; }

	/**
	 * @param external_pitch_setpoint Externally commanded pitch angle setpoint (usually from TECS) [rad]
	 * @return Pitch angle setpoint (limited while plane is on runway) [rad]
	 */
	float getPitch(float external_pitch_setpoint);

	/**
	 * @param external_roll_setpoint Externally commanded roll angle setpoint (usually from L1) [rad]
	 * @return Roll angle setpoint [rad]
	 */
	float getRoll(float external_roll_setpoint);

	/**
	 * @brief Returns the appropriate yaw angle setpoint.
	 *
	 * In heading hold mode (_heading_mode == 0), it returns initial yaw as long as it's on the runway.
	 * When it has enough ground clearance we start navigation towards WP.
	 *
	 * @param external_yaw_setpoint Externally commanded yaw angle setpoint [rad]
	 * @return Yaw angle setpoint [rad]
	 */
	float getYaw(float external_yaw_setpoint);

	/**
	 * @brief Returns the throttle setpoint.
	 *
	 * Ramps up over RWTO_RAMP_TIME to RWTO_MAX_THR until the aircraft lifts off the runway, then passes
	 * through the externally defined throttle setting.
	 *
	 * @param time_now Absolute time since system boot [us]
	 * @param external_throttle_setpoint Externally commanded throttle setpoint (usually from TECS), normalized [0,1]
	 * @return Throttle setpoint, normalized [0,1]
	 */
	float getThrottle(const hrt_abstime &time_now, const float external_throttle_setpoint);

	/**
	 * @return If the attitude / rate control integrators should be continually reset.
	 * This is the case during ground roll.
	 */
	bool resetIntegrators();

	/**
	 * @param min_pitch_in_climbout Minimum pitch angle during climbout [rad]
	 * @param min_pitch Externally commanded minimum pitch angle [rad]
	 * @return Minimum pitch angle [rad]
	 */
	float getMinPitch(const float min_pitch_in_climbout, const float min_pitch);

	/**
	 * @param max_pitch Externally commanded maximum pitch angle [rad]
	 * @return Maximum pitch angle [rad]
	 */
	float getMaxPitch(const float max_pitch);

	/**
	 * @return Runway takeoff starting position in global frame (lat, lon) [deg]
	 */
	const matrix::Vector2d &getStartPosition() const { return _start_pos_global; };

	/**
	 * @brief Reset the state machine.
	 */
	void reset();

	// NOTE: this is only to be used for mistaken mode transitions to takeoff while already in air
	void forceSetClimboutState() { _takeoff_state = RunwayTakeoffState::CLIMBOUT; }

private:
	/**
	 * @brief Minimum allowed maximum pitch constraint (from parameter) for runway takeoff. [rad]
	 */
	static constexpr float kMinMaxPitch = 0.1f;

	/**
	 * Current state of runway takeoff procedure
	 */
	RunwayTakeoffState _takeoff_state{THROTTLE_RAMP};

	/**
	 * True if the runway state machine is initialized
	 */
	bool _initialized{false};

	/**
	 * The absolute time since system boot at which the state machine was intialized [us]
	 */
	hrt_abstime _time_initialized{0};

	/**
	 * Initial yaw of the vehicle on first pass through the runway takeoff state machine.
	 * used for heading hold mode. [rad]
	 */
	float _initial_yaw{0.f};

	/**
	 * True if TECS should be commanded to "climbout" mode.
	 */
	bool _climbout{false};

	/**
	 * The global (lat, lon) position of the vehicle on first pass through the runway takeoff state machine. The
	 * takeoff path emanates from this point to correct for any GNSS uncertainty from the planned takeoff point. The
	 * vehicle should accordingly be set on the center of the runway before engaging the mission. [deg]
	 */
	matrix::Vector2d _start_pos_global{};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::RWTO_TKOFF>) _param_rwto_tkoff,
		(ParamInt<px4::params::RWTO_HDG>) _param_rwto_hdg,
		(ParamFloat<px4::params::RWTO_MAX_THR>) _param_rwto_max_thr,
		(ParamFloat<px4::params::RWTO_PSP>) _param_rwto_psp,
		(ParamFloat<px4::params::RWTO_MAX_PITCH>) _param_rwto_max_pitch,
		(ParamFloat<px4::params::RWTO_AIRSPD_SCL>) _param_rwto_airspd_scl,
		(ParamFloat<px4::params::RWTO_RAMP_TIME>) _param_rwto_ramp_time,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min
	)
};

} // namespace runwaytakeoff

#endif // RUNWAYTAKEOFF_H
