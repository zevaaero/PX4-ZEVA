/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "battery.hpp"

#include <lib/ecl/geo/geo.h>
#include <px4_defines.h>

const char *const UavcanBatteryBridge::NAME = "battery";

UavcanBatteryBridge::UavcanBatteryBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_battery", ORB_ID(battery_status)),
	ModuleParams(nullptr),
	_sub_battery(node)
{
}

int UavcanBatteryBridge::init()
{
	int res = _sub_battery.start(BatteryInfoCbBinder(this, &UavcanBatteryBridge::battery_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanBatteryBridge::battery_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &msg)
{
	_actuators_sub.copy(&_actuator_controls);

	_battery.updateBatteryStatus(
		hrt_absolute_time(),
		msg.voltage,
		msg.current,
		true,
		battery_status_s::BATTERY_SOURCE_EXTERNAL,
		0,
		_actuator_controls.control[actuator_controls_s::INDEX_THROTTLE],
		false
	);

	/* Override data that is expected to arrive from UAVCAN msg*/
	battery_status_s *battery_status = _battery.getBatteryStatus();
	battery_status->remaining = msg.state_of_charge_pct / 100.0f; // between 0 and 1
	battery_status->temperature = msg.temperature + CONSTANTS_ABSOLUTE_NULL_CELSIUS; // Kelvin to Celcius
	battery_status->capacity = msg.full_charge_capacity_wh;
	itoa(msg.model_instance_id, battery_status->serial_number, 10);
	battery_status->id = msg.getSrcNodeID().get();

	publish(msg.getSrcNodeID().get(), battery_status);
}
