/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include "vtol_att_control_main.h"

#define PITCH_TRANSITION_FRONT_P1 -1.1f	// pitch angle to switch to TRANSITION_P2
#define PITCH_TRANSITION_BACK -0.25f	// pitch angle to switch to MC

using namespace matrix;

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	_flag_was_in_trans_mode = false;
}

void
Tailsitter::parameters_update()
{
	VtolType::updateParams();

}

void Tailsitter::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/

	float pitch = Eulerf(Quatf(_v_att->q)).theta();

	if (_vtol_vehicle_status->vtol_transition_failsafe) {
		// Failsafe event, switch to MC mode immediately
		_vtol_vehicle_status->vtol_state = vtol_vehicle_status_s::VTOL_STATE_MC;

		//reset failsafe when FW is no longer requested
		if (!_attc->is_fixed_wing_requested()) {
			_vtol_vehicle_status->vtol_transition_failsafe = false;
		}

	} else if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_vehicle_status->vtol_state) { // user switchig to MC mode
		case vtol_vehicle_status_s::VTOL_STATE_MC:
			break;

		case vtol_vehicle_status_s::VTOL_STATE_FW:
			_vtol_vehicle_status->vtol_state = vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P1;
			_transition_start = hrt_absolute_time();
			break;

		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P1:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P2:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P3:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P4:
			// failsafe into multicopter vtol_state
			_vtol_vehicle_status->vtol_state = vtol_vehicle_status_s::VTOL_STATE_MC;
			break;

		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P1:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P2:
			float time_since_trans_start = (float)(hrt_absolute_time() - _transition_start) * 1e-6f;

			// check if we have reached pitch angle to switch to MC vtol_state
			if (pitch >= PITCH_TRANSITION_BACK || time_since_trans_start > _param_vt_b_trans_dur.get()) {
				_vtol_vehicle_status->vtol_state = vtol_vehicle_status_s::VTOL_STATE_MC;
			}

			break;
		}

	} else {  // user switchig to FW vtol_state

		switch (_vtol_vehicle_status->vtol_state) {
		case vtol_vehicle_status_s::VTOL_STATE_MC:
			// initialise a front transition
			_vtol_vehicle_status->vtol_state = vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P1;
			_transition_start = hrt_absolute_time();
			break;

		case vtol_vehicle_status_s::VTOL_STATE_FW:
			break;

		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P1:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P2:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P3:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P4: {
				const bool airspeed_triggers_transition = PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)
						&& !_param_fw_arsp_mode.get() ;

				bool transition_to_fw = false;

				if (pitch <= PITCH_TRANSITION_FRONT_P1) {
					if (airspeed_triggers_transition) {
						transition_to_fw = _airspeed_validated->calibrated_airspeed_m_s >= _param_vt_arsp_trans.get() ;

					} else {
						transition_to_fw = true;
					}
				}

				transition_to_fw |= can_transition_on_ground();

				if (transition_to_fw) {
					_vtol_vehicle_status->vtol_state = vtol_vehicle_status_s::VTOL_STATE_FW;
				}

				break;
			}

		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P1:
		case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P2:
			// failsafe into fixed wing vtol_state
			_vtol_vehicle_status->vtol_state = vtol_vehicle_status_s::VTOL_STATE_FW;
			break;
		}
	}
}

void Tailsitter::update_transition_state()
{
	const float time_since_trans_start = (float)(hrt_absolute_time() - _transition_start) * 1e-6f;

	if (!_flag_was_in_trans_mode) {
		_flag_was_in_trans_mode = true;

		if (_vtol_vehicle_status->vtol_state == vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P1) {
			// calculate rotation axis for transition.
			_q_trans_start = Quatf(_v_att->q);
			Vector3f z = -_q_trans_start.dcm_z();
			_trans_rot_axis = z.cross(Vector3f(0, 0, -1));

			// as heading setpoint we choose the heading given by the direction the vehicle points
			float yaw_sp = atan2f(z(1), z(0));

			// the intial attitude setpoint for a backtransition is a combination of the current fw pitch setpoint,
			// the yaw setpoint and zero roll since we want wings level transition
			_q_trans_start = Eulerf(0.0f, _fw_virtual_att_sp->pitch_body, yaw_sp);

			// attitude during transitions are controlled by mc attitude control so rotate the desired attitude to the
			// multirotor frame
			_q_trans_start = _q_trans_start * Quatf(Eulerf(0, -M_PI_2_F, 0));

		} else if (_vtol_vehicle_status->vtol_state == vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P1) {
			// initial attitude setpoint for the transition should be with wings level
			_q_trans_start = Eulerf(0.0f, _mc_virtual_att_sp->pitch_body, _mc_virtual_att_sp->yaw_body);
			Vector3f x = Dcmf(Quatf(_v_att->q)) * Vector3f(1, 0, 0);
			_trans_rot_axis = -x.cross(Vector3f(0, 0, -1));
		}

		_q_trans_sp = _q_trans_start;
	}

	// ensure input quaternions are exactly normalized because acosf(1.00001) == NaN
	_q_trans_sp.normalize();

	// tilt angle (zero if vehicle nose points up (hover))
	float cos_tilt = _q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) - _q_trans_sp(2) *
			 _q_trans_sp(2) + _q_trans_sp(3) * _q_trans_sp(3);
	cos_tilt = cos_tilt >  1.0f ?  1.0f : cos_tilt;
	cos_tilt = cos_tilt < -1.0f ? -1.0f : cos_tilt;
	const float tilt = acosf(cos_tilt);

	switch (_vtol_vehicle_status->vtol_state) {
	case vtol_vehicle_status_s::VTOL_STATE_MC:
		break;

	case vtol_vehicle_status_s::VTOL_STATE_FW:
		break;

	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P1:
	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P2:
	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P3:
	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_FW_P4: {
			const float trans_pitch_rate = M_PI_2_F / _param_vt_f_trans_dur.get();

			if (tilt < M_PI_2_F - math::radians(_param_fw_psp_off.get())) {
				_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
							       time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
			}

			break;
		}

	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P1:
	case vtol_vehicle_status_s::VTOL_STATE_TRANSITION_TO_MC_P2: {
			const float trans_pitch_rate = M_PI_2_F / _param_vt_b_trans_dur.get();

			if (!_flag_idle_mc) {
				_flag_idle_mc = set_idle_mc();
			}

			if (tilt > 0.01f) {
				_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
							       time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
			}

			break;
		}
	}

	_v_att_sp->thrust_body[2] = _mc_virtual_att_sp->thrust_body[2];

	_v_att_sp->timestamp = hrt_absolute_time();

	const Eulerf euler_sp(_q_trans_sp);
	_v_att_sp->roll_body = euler_sp.phi();
	_v_att_sp->pitch_body = euler_sp.theta();
	_v_att_sp->yaw_body = euler_sp.psi();

	_q_trans_sp.copyTo(_v_att_sp->q_d);
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust_body[0] = _thrust_transition;
}

void Tailsitter::update_fw_state()
{
	VtolType::update_fw_state();

}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	auto &mc_in = _actuators_mc_in->control;
	auto &fw_in = _actuators_fw_in->control;

	auto &mc_out = _actuators_out_0->control;
	auto &fw_out = _actuators_out_1->control;

	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _actuators_mc_in->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _actuators_fw_in->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _actuators_mc_in->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _actuators_fw_in->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;


	mc_out[actuator_controls_s::INDEX_ROLL]  = mc_in[actuator_controls_s::INDEX_ROLL];
	mc_out[actuator_controls_s::INDEX_PITCH] = mc_in[actuator_controls_s::INDEX_PITCH];
	mc_out[actuator_controls_s::INDEX_YAW]   = mc_in[actuator_controls_s::INDEX_YAW];

	if (_vtol_vehicle_status->vtol_state == vtol_vehicle_status_s::VTOL_STATE_FW) {
		mc_out[actuator_controls_s::INDEX_THROTTLE] = fw_in[actuator_controls_s::INDEX_THROTTLE];

		// FW thrust is allocated on mc_thrust_sp[0] for tailsitter with dynamic control allocation
		_thrust_setpoint_0->xyz[2] = -fw_in[actuator_controls_s::INDEX_THROTTLE];

		/* allow differential thrust if enabled */
		if (_param_vt_fw_difthr_en.get()) {
			mc_out[actuator_controls_s::INDEX_ROLL] = fw_in[actuator_controls_s::INDEX_YAW] * _param_vt_fw_difthr_sc.get() ;
			_torque_setpoint_0->xyz[0] = fw_in[actuator_controls_s::INDEX_YAW] * _param_vt_fw_difthr_sc.get() ;
		}

	} else {
		_torque_setpoint_0->xyz[0] = mc_in[actuator_controls_s::INDEX_ROLL];
		_torque_setpoint_0->xyz[1] = mc_in[actuator_controls_s::INDEX_PITCH];
		_torque_setpoint_0->xyz[2] = mc_in[actuator_controls_s::INDEX_YAW];

		mc_out[actuator_controls_s::INDEX_THROTTLE] = mc_in[actuator_controls_s::INDEX_THROTTLE];
		_thrust_setpoint_0->xyz[2] = -mc_in[actuator_controls_s::INDEX_THROTTLE];
	}

	if (_param_vt_elev_mc_lock.get()  && _vtol_vehicle_status->vtol_state == vtol_vehicle_status_s::VTOL_STATE_MC) {
		fw_out[actuator_controls_s::INDEX_ROLL]  = 0;
		fw_out[actuator_controls_s::INDEX_PITCH] = 0;

	} else {
		fw_out[actuator_controls_s::INDEX_ROLL]  = fw_in[actuator_controls_s::INDEX_ROLL];
		fw_out[actuator_controls_s::INDEX_PITCH] = fw_in[actuator_controls_s::INDEX_PITCH];
		_torque_setpoint_1->xyz[1] = fw_in[actuator_controls_s::INDEX_PITCH];
		_torque_setpoint_1->xyz[2] = -fw_in[actuator_controls_s::INDEX_ROLL];
	}

	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	_actuators_out_0->timestamp = _actuators_out_1->timestamp = hrt_absolute_time();
}
