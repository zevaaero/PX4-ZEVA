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
 * @file vtol_land.cpp
 *
 * Helper class to do a VTOL landing using a loiter down to altitude landing pattern.
 *
 */

#include "vtol_land.h"
#include "navigator.h"

using matrix::wrap_pi;

VtolLand::VtolLand(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
VtolLand::on_activation()
{
	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));
	int num_safe_points = 0;

	if (ret == sizeof(mission_stats_entry_s)) {
		num_safe_points = stats.num_items;
	}

	for (int current_seq = 1; current_seq <= num_safe_points; ++current_seq) {
		mission_safe_point_s mission_safe_point;

		if (dm_read(DM_KEY_SAFE_POINTS, current_seq, &mission_safe_point, sizeof(mission_safe_point_s)) !=
		    sizeof(mission_safe_point_s)) {
			PX4_ERR("dm_read failed");
			continue;
		}

		if (mission_safe_point.nav_cmd == NAV_CMD_VTOL_SAFE_AREA) {
			setSectorBitmap(mission_safe_point.safe_area_sector_clear_bitmap);
			setSectorOffsetDegrees(mission_safe_point.safe_area_first_sector_offset_degrees);
			setSafeAreaRadiusMeter(mission_safe_point.safe_area_radius);
			break;
		}

	}

	_land_pos_lat_lon = _navigator->getTakeoffPosition();
	set_loiter_position();
	_land_state = vtol_land_state::MOVE_TO_LOITER;

}

void
VtolLand::on_active()
{
	if (is_mission_item_reached()) {
		switch	(_land_state) {
		case vtol_land_state::MOVE_TO_LOITER: {
				_mission_item.altitude = _navigator->get_home_position()->alt + _param_descend_alt_rel_m.get();
				_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
				_mission_item.loiter_radius = _navigator->get_loiter_radius();

				_navigator->get_mission_result()->finished = false;
				_navigator->set_mission_result_updated();
				reset_mission_item_reached();

				// convert mission item to current setpoint
				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_apply_limitation(_mission_item);
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				pos_sp_triplet->next.valid = true;
				pos_sp_triplet->next.lat = _land_pos_lat_lon(0);
				pos_sp_triplet->next.lon = _land_pos_lat_lon(1);

				pos_sp_triplet->previous.valid = false;
				pos_sp_triplet->current.yaw_valid = true;

				_land_state = vtol_land_state::LOITER_DOWN;
				break;
			}

		case vtol_land_state::LOITER_DOWN: {
				_mission_item.lat = _land_pos_lat_lon(0);
				_mission_item.lon = _land_pos_lat_lon(1);

				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
				_mission_item.vtol_back_transition = true;

				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				// set previous item location to loiter location such that vehicle tracks line between loiter
				// location and land location after exiting the loiter circle
				pos_sp_triplet->previous.lat = _loiter_pos_lat_lon(0);
				pos_sp_triplet->previous.lon = _loiter_pos_lat_lon(1);
				pos_sp_triplet->previous.valid = true;

				//publish_navigator_mission_item(); // for logging
				_navigator->set_position_setpoint_triplet_updated();

				// issue_command(_mission_item);
				reset_mission_item_reached();

				_land_state = vtol_land_state::TRANSITION_TO_MC;

				break;
			}

		case vtol_land_state::TRANSITION_TO_MC: {
				set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
				_mission_item.vtol_back_transition = true;

				issue_command(_mission_item);

				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				//publish_navigator_mission_item(); // for logging
				_navigator->set_position_setpoint_triplet_updated();

				// issue_command(_mission_item);
				reset_mission_item_reached();

				_land_state = vtol_land_state::LAND;

				break;
			}

		case vtol_land_state::LAND: {
				_mission_item.nav_cmd = NAV_CMD_LAND;
				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				//publish_navigator_mission_item(); // for logging
				_navigator->set_position_setpoint_triplet_updated();

				// issue_command(_mission_item);
				reset_mission_item_reached();

				_land_state = vtol_land_state::IDLE;

				break;
			}

		default: {

				break;
			}
		}
	}
}

void VtolLand::generate_waypoint_from_heading(struct position_setpoint_s *setpoint, float yaw)
{
	waypoint_from_heading_and_distance(
		_navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
		yaw, _safe_area_radius_m,
		&(setpoint->lat), &(setpoint->lon));
	setpoint->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	setpoint->yaw = yaw;
}

void
VtolLand::set_loiter_position()
{
	double lat, lon;

	waypoint_from_heading_and_distance(
		_land_pos_lat_lon(0), _land_pos_lat_lon(1),
		getBestLandingHeading(), _safe_area_radius_m * cosf(M_PI_F / _num_sectors) -
		_param_loiter_radius_m.get(),
		&lat, &lon);

	_loiter_pos_lat_lon(0) = lat;
	_loiter_pos_lat_lon(1) = lon;

	_mission_item.lat  = _loiter_pos_lat_lon(0);
	_mission_item.lon = _loiter_pos_lat_lon(1);
	_mission_item.altitude = math::max(_navigator->get_home_position()->alt + _param_return_alt_rel_m.get(),
					   _navigator->get_global_position()->alt);
	_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
	_mission_item.force_heading = true;
	_mission_item.autocontinue = false;
	_mission_item.time_inside = _min_loiter_time_before_land;

	_mission_item.altitude_is_relative = false;

	_mission_item.loiter_radius = _navigator->get_loiter_radius();
	_mission_item.origin = ORIGIN_ONBOARD;

	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

	_navigator->set_can_loiter_at_sp(true);

	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

float VtolLand::getBestLandingHeading()
{
	wind_s *wind = _navigator->get_wind();
	const float wind_direction = atan2f(wind->windspeed_east, wind->windspeed_north);
	uint8_t min_index = 0;
	float delta_heading_prev = INFINITY;
	const float sector_angle = 2 * M_PI_F / _num_sectors;

	for (int i = 0; i < 8; i++) {
		if (_sector_bitmap & (1 << i)) {

			const float center_heading_sector = sector_angle * i + math::radians(_offset_degrees) + sector_angle * 0.5f;
			const float delta_heading = wrap_pi(wind_direction - center_heading_sector);

			if (fabsf(delta_heading) < delta_heading_prev) {
				min_index = i;
				delta_heading_prev = fabsf(delta_heading);
			}
		}
	}

	return wrap_pi(sector_angle * min_index + math::radians(_offset_degrees) + sector_angle * 0.5f);
}

bool VtolLand::hasSafeArea()
{
	return _sector_bitmap > 0;
}
