/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file VTOL_att_control_main.cpp
 * Implementation of an attitude controller for VTOL airframes. This module receives data
 * from both the fixed wing- and the multicopter attitude controllers and processes it.
 * It computes the correct actuator controls depending on which vtol_state the vehicle is in (hover, forward-
 * flight or transition). It also publishes the resulting controls on the actuator controls topics.
 *
 * @author Roman Bapst 		<bapstr@ethz.ch>
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler	<thomasgubler@gmail.com>
 * @author David Vorsin		<davidvorsin@gmail.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
 */
#include "vtol_att_control_main.h"
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>

using namespace matrix;
using namespace time_literals;

VtolAttitudeControl::VtolAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "vtol_att_control: cycle"))
{
	// start vtol in rotary wing vtol_state
	_vtol_vehicle_status.vtol_state = vtol_vehicle_status_s::VTOL_STATE_MC;

	parameters_update();

	if (static_cast<vtol_type>(_param_vt_type.get()) == vtol_type::TAILSITTER) {
		_vtol_type = new Tailsitter(this);

	} else if (static_cast<vtol_type>(_param_vt_type.get()) == vtol_type::TILTROTOR) {
		_vtol_type = new Tiltrotor(this);

	} else if (static_cast<vtol_type>(_param_vt_type.get()) == vtol_type::STANDARD) {
		_vtol_type = new Standard(this);

	} else {
		exit_and_cleanup();
	}

	_vtol_vehicle_status_pub.advertise();
	_vehicle_thrust_setpoint0_pub.advertise();
	_vehicle_torque_setpoint0_pub.advertise();
	_vehicle_thrust_setpoint1_pub.advertise();
	_vehicle_torque_setpoint1_pub.advertise();
}

VtolAttitudeControl::~VtolAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
VtolAttitudeControl::init()
{
	if (!_actuator_inputs_mc.registerCallback()) {
		PX4_ERR("MC actuator controls callback registration failed!");
		return false;
	}

	if (!_actuator_inputs_fw.registerCallback()) {
		PX4_ERR("FW actuator controls callback registration failed!");
		return false;
	}

	return true;
}

void VtolAttitudeControl::action_request_poll()
{
	while (_action_request_sub.updated()) {
		action_request_s action_request;

		if (_action_request_sub.copy(&action_request)) {
			switch (action_request.action) {
			case action_request_s::ACTION_VTOL_TRANSITION_TO_MULTICOPTER:
				_transition_command = vtol_vehicle_status_s::VTOL_STATE_MC;
				_immediate_transition = false;
				break;

			case action_request_s::ACTION_VTOL_TRANSITION_TO_FIXEDWING:
				_transition_command = vtol_vehicle_status_s::VTOL_STATE_FW;
				_immediate_transition = false;
				break;
			}
		}
	}
}

void VtolAttitudeControl::vehicle_cmd_poll()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_cmd_sub.update(&vehicle_command)) {
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			uint8_t result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

			const int transition_command_param1 = int(vehicle_command.param1 + 0.5f);

			// deny transition from MC to FW in Takeoff, Land, RTL and Orbit
			if (transition_command_param1 == vtol_vehicle_status_s::VTOL_STATE_FW &&
			    (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
			     || vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
			     || vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
			     ||  vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT)) {

				result = vehicle_command_ack_s::VEHICLE_RESULT_TEMPORARILY_REJECTED;

			} else {
				_transition_command = transition_command_param1;
				_immediate_transition = (PX4_ISFINITE(vehicle_command.param2)) ? int(vehicle_command.param2 + 0.5f) : false;
			}

			if (vehicle_command.from_external) {
				vehicle_command_ack_s command_ack{};
				command_ack.timestamp = hrt_absolute_time();
				command_ack.command = vehicle_command.command;
				command_ack.result = result;
				command_ack.target_system = vehicle_command.source_system;
				command_ack.target_component = vehicle_command.source_component;

				uORB::Publication<vehicle_command_ack_s> command_ack_pub{ORB_ID(vehicle_command_ack)};
				command_ack_pub.publish(command_ack);
			}
		}
	}
}

void
VtolAttitudeControl::parameters_update()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		updateParams();
	}

	if (_vtol_type != nullptr) {
		_vtol_type->parameters_update();
	}
}

void
VtolAttitudeControl::Run()
{
	if (should_exit()) {
		_actuator_inputs_fw.unregisterCallback();
		_actuator_inputs_mc.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

#if !defined(ENABLE_LOCKSTEP_SCHEDULER)

	// prevent excessive scheduling (> 500 Hz)
	if (now - _last_run_timestamp < 2_ms) {
		return;
	}

#endif // !ENABLE_LOCKSTEP_SCHEDULER

	_last_run_timestamp = now;

	if (!_initialized) {

		if (_vtol_type->init()) {
			_initialized = true;

		} else {
			exit_and_cleanup();
			return;
		}
	}

	perf_begin(_loop_perf);

	const bool updated_mc_in = _actuator_inputs_mc.update(&_actuators_mc_in);
	const bool updated_fw_in = _actuator_inputs_fw.update(&_actuators_fw_in);

	// run update_vtol_state when either the mc or fw are updated
	if (updated_mc_in || updated_fw_in) {
		// update the vtol state machine which decides which vtol_state we are in
		_vtol_type->update_vtol_state();
	}

	// run on actuator publications corresponding to VTOL vtol_state
	bool should_run = false;

	switch (_vtol_vehicle_status.vtol_state) {

	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P1:
	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P2:
	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P3:
	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P4:
	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P1:
	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P2:
		should_run = updated_fw_in || updated_mc_in;
		break;

	case vtol_vehicle_status_s::VTOL_STATE_MC:
		should_run = updated_mc_in;
		break;

	case vtol_vehicle_status_s::VTOL_STATE_FW:
		should_run = updated_fw_in;
		break;
	}

	if (should_run) {
		parameters_update();

		_v_control_mode_sub.update(&_v_control_mode);
		_v_att_sub.update(&_v_att);
		_local_pos_sub.update(&_local_pos);
		_local_pos_sp_sub.update(&_local_pos_sp);
		_pos_sp_triplet_sub.update(&_pos_sp_triplet);
		_airspeed_validated_sub.update(&_airspeed_validated);
		_tecs_status_sub.update(&_tecs_status);
		_land_detected_sub.update(&_land_detected);
		action_request_poll();
		vehicle_cmd_poll();

		// check if mc and fw sp were updated
		const bool mc_att_sp_updated = _mc_virtual_att_sp_sub.update(&_mc_virtual_att_sp);
		const bool fw_att_sp_updated = _fw_virtual_att_sp_sub.update(&_fw_virtual_att_sp);

		// check in which vtol_state we are in and call vtol_state specific functions
		switch (_vtol_vehicle_status.vtol_state) {
		case vtol_vehicle_status_s::VTOL_STATE_MC:
			_vtol_type->update_mc_state();
			_v_att_sp_pub.publish(_v_att_sp);

			break;

		case vtol_vehicle_status_s::VTOL_STATE_FW:
			if (fw_att_sp_updated) {
				_vtol_type->update_fw_state();
				_v_att_sp_pub.publish(_v_att_sp);
			}

			break;

		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P1:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P2:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P3:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P4:

			_fw_virtual_att_sp_sub.update(&_fw_virtual_att_sp);

			if (!_vtol_type->was_in_trans_mode() || mc_att_sp_updated || fw_att_sp_updated) {
				_vtol_type->update_transition_state();
				_v_att_sp_pub.publish(_v_att_sp);
			}

			break;

		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P1:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P2:

			_fw_virtual_att_sp_sub.update(&_fw_virtual_att_sp);

			if (!_vtol_type->was_in_trans_mode() || mc_att_sp_updated || fw_att_sp_updated) {
				_vtol_type->update_transition_state();
				_v_att_sp_pub.publish(_v_att_sp);
			}

			break;

		}

		_vtol_type->fill_actuator_outputs();
		_actuators_0_pub.publish(_actuators_out_0);
		_actuators_1_pub.publish(_actuators_out_1);

		_vehicle_torque_setpoint0_pub.publish(_torque_setpoint_0);
		_vehicle_torque_setpoint1_pub.publish(_torque_setpoint_1);
		_vehicle_thrust_setpoint0_pub.publish(_thrust_setpoint_0);
		_vehicle_thrust_setpoint1_pub.publish(_thrust_setpoint_1);

		// Advertise/Publish vtol vehicle status
		_vtol_vehicle_status.timestamp = hrt_absolute_time();
		_vtol_vehicle_status_pub.publish(_vtol_vehicle_status);
	}

	perf_end(_loop_perf);
}

int
VtolAttitudeControl::task_spawn(int argc, char *argv[])
{
	VtolAttitudeControl *instance = new VtolAttitudeControl();

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

int
VtolAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
VtolAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_NAME("vtol_att_control", "controller");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int vtol_att_control_main(int argc, char *argv[])
{
	return VtolAttitudeControl::main(argc, argv);
}
