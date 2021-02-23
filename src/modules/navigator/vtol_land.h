/***************************************************************************
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
 * @file vtol_land.h
 *
 * Helper class to do a VTOL landing using a loiter down to altitude landing pattern.
 *
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

#include <lib/mathlib/mathlib.h>

#include <px4_platform_common/module_params.h>

class VtolLand : public MissionBlock, public ModuleParams
{
public:
	VtolLand(Navigator *navigator);
	~VtolLand() = default;

	void on_activation() override;
	void on_active() override;

	void setLandPosition(const double lat, const double lon) { _land_pos_lat_lon = matrix::Vector2<double>(lat, lon);}
	void setTransitionAltitudeAbsolute(const float alt_amsl) {_transition_alt_amsl = alt_amsl; }
	void setLoiterAltitudeAbsolute(const float alt_amsl) {_loiter_alt_amsl = alt_amsl; }
	void setSectorBitmap(uint8_t bitmap) { _sector_bitmap = bitmap; }
	void setSectorOffsetDegrees(int offset) { _offset_degrees = offset; }

private:

	enum class vtol_land_state {
		MOVE_TO_LOITER = 0,
		LOITER_DOWN,
		TRANSITION_TO_MC,
		LAND,
		IDLE
	} _land_state;

	matrix::Vector2<double> _land_pos_lat_lon;
	matrix::Vector2<double> _loiter_pos_lat_lon;

	float _transition_alt_amsl{0.f};
	float _loiter_alt_amsl{0.f};
	float 	_offset_degrees{0};

	uint8_t _sector_bitmap{0};
	static constexpr uint8_t _num_sectors = 8;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VTO_SAFE_AREA_R>) _param_safe_area_radius_m,
		(ParamFloat<px4::params::VTO_LOITER_ALT>) _param_loiter_alt,
		(ParamFloat<px4::params::RTL_RETURN_ALT>) _param_return_alt_rel_m,
		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_loiter_radius_m
	)

	void set_loiter_position();

	void generate_waypoint_from_heading(struct position_setpoint_s *setpoint, float yaw);

	bool transitionHeadingReached() {return false;}

	float getBestLandingHeading();

};
