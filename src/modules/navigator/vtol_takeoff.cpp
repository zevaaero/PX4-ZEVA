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
 * @file vtol_takeoff.cpp
 *
 * Helper class to do a VTOL takeoff and transition into a loiter.
 *
 */

#include "vtol_takeoff.h"
#include "navigator.h"

using matrix::wrap_pi;

VtolTakeoff::VtolTakeoff(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	clearSafeAreaFromStorage();
}

void
VtolTakeoff::on_activation()
{
	set_takeoff_position();
	_takeoff_state = vtol_takeoff_state::TAKEOFF_HOVER;

	readSafeAreaFromStorage();
}

void VtolTakeoff::readSafeAreaFromStorage()
{

	setSectorBitmap(0);
	setSectorOffsetDegrees(0);

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
}

void VtolTakeoff::clearSafeAreaFromStorage()
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
			mission_safe_point.safe_area_sector_clear_bitmap = 0;
			mission_safe_point.safe_area_first_sector_offset_degrees = 0;
			mission_safe_point.safe_area_radius = -1;
			setSectorBitmap(mission_safe_point.safe_area_sector_clear_bitmap);
			setSectorOffsetDegrees(mission_safe_point.safe_area_first_sector_offset_degrees);
			setSafeAreaRadiusMeter(mission_safe_point.safe_area_radius);

			const bool write_failed = dm_write(DM_KEY_SAFE_POINTS, current_seq, DM_PERSIST_POWER_ON_RESET, &mission_safe_point,
							   sizeof(struct mission_safe_point_s)) != sizeof(struct mission_safe_point_s);

			if (write_failed) {
				PX4_DEBUG("Dataman write failed");
			}

			break;
		}

	}
}

void
VtolTakeoff::on_active()
{
	if (is_mission_item_reached()) {
		switch	(_takeoff_state) {
		case vtol_takeoff_state::TAKEOFF_HOVER: {

				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
				_front_trans_heading_sp_rad = getClosestTransitionHeading();
				_mission_item.yaw = _front_trans_heading_sp_rad;
				_mission_item.force_heading = true;
				mission_apply_limitation(_mission_item);
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
				pos_sp_triplet->current.disable_weather_vane = true;
				_navigator->set_position_setpoint_triplet_updated();
				reset_mission_item_reached();

				_takeoff_state = vtol_takeoff_state::ALIGN_HEADING;

				break;
			}

		case vtol_takeoff_state::ALIGN_HEADING: {

				set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				pos_sp_triplet->previous = pos_sp_triplet->current;
				generate_waypoint_from_heading(&pos_sp_triplet->current, _mission_item.yaw);
				_navigator->set_position_setpoint_triplet_updated();

				issue_command(_mission_item);

				_takeoff_state = vtol_takeoff_state::TRANSITION;

				break;
			}

		case vtol_takeoff_state::TRANSITION: {
				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
					setLoiterItemFromCurrentPositionSetpoint(&_mission_item);

				} else {
					setLoiterItemFromCurrentPosition(&_mission_item);
				}

				_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
				_mission_item.loiter_radius = _navigator->get_loiter_radius();
				_mission_item.altitude = _navigator->get_home_position()->alt + _param_loiter_alt.get();

				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
				generate_waypoint_from_heading(&pos_sp_triplet->current, _front_trans_heading_sp_rad);

				//publish_navigator_mission_item(); // for logging
				_navigator->set_position_setpoint_triplet_updated();

				reset_mission_item_reached();

				_takeoff_state = vtol_takeoff_state::CLIMB;

				break;
			}

		case vtol_takeoff_state::CLIMB: {

				// the VTOL takeoff is done, proceed loitering and upate the navigation state to LOITER
				_navigator->get_mission_result()->finished = true;
				_navigator->set_mission_result_updated();

				break;
			}

		default: {

				break;
			}
		}
	}
}

void VtolTakeoff::generate_waypoint_from_heading(struct position_setpoint_s *setpoint, float yaw)
{
	waypoint_from_heading_and_distance(
		_navigator->get_home_position()->lat, _navigator->get_home_position()->lon,
		yaw, _safe_area_radius_m * cosf(M_PI_F / _num_sectors) - _param_loiter_radius_m.get(),
		&(setpoint->lat), &(setpoint->lon));
	setpoint->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	setpoint->yaw = yaw;
}

void
VtolTakeoff::set_takeoff_position()
{
	// set current mission item to takeoff
	set_takeoff_item(&_mission_item, _transition_alt_amsl);

	_mission_item.lat = _navigator->get_home_position()->lat;
	_mission_item.lon = _navigator->get_home_position()->lon;

	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);



	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->current.yaw_valid = true;
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

float VtolTakeoff::getClosestTransitionHeading()
{
	const float vehicle_heading = _navigator->get_local_position()->heading;
	uint8_t min_index = 0;
	float delta_heading_prev = INFINITY;
	const float sector_angle = 2 * M_PI_F / _num_sectors;

	for (int i = 0; i < _num_sectors; i++) {
		if (_sector_bitmap & (1 << i)) {

			const float center_heading_sector = sector_angle * i + math::radians(_offset_degrees) + sector_angle * 0.5f;
			const float delta_heading = wrap_pi(vehicle_heading - center_heading_sector);

			if (fabsf(delta_heading) < delta_heading_prev) {
				min_index = i;
				delta_heading_prev = fabsf(delta_heading);
			}
		}
	}

	return wrap_pi(sector_angle * min_index + math::radians(_offset_degrees) + sector_angle * 0.5f);
}
