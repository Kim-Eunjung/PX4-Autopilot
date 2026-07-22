/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessHelicopter.hpp"
#include <lib/mathlib/mathlib.h>

#include <float.h>

using namespace matrix;
using namespace time_literals;

ActuatorEffectivenessHelicopter::ActuatorEffectivenessHelicopter(ModuleParams *parent, ActuatorType tail_actuator_type)
	: ModuleParams(parent), _tail_actuator_type(tail_actuator_type)
{
	for (int i = 0; i < NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SP0_ANG%u", i);
		_param_handles.swash_plate_servos[i].angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SP0_ARM_L%u", i);
		_param_handles.swash_plate_servos[i].arm_length = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRIM", i);
		_param_handles.swash_plate_servos[i].trim = param_find(buffer);
	}

	_param_handles.num_swash_plate_servos = param_find("CA_SP0_COUNT");

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_HELI_THR_C%u", i);
		_param_handles.throttle_curve[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_HELI_PITCH_C%u", i);
		_param_handles.pitch_curve[i] = param_find(buffer);
	}

	_param_handles.yaw_collective_pitch_scale = param_find("CA_HELI_YAW_CP_S");
	_param_handles.yaw_collective_pitch_offset = param_find("CA_HELI_YAW_CP_O");
	_param_handles.yaw_throttle_scale = param_find("CA_HELI_YAW_TH_S");
	_param_handles.yaw_ccw = param_find("CA_HELI_YAW_CCW");
	_param_handles.spoolup_time = param_find("COM_SPOOLUP_TIME");
	_param_handles.max_servo_throw = param_find("CA_MAX_SVO_THROW");
	_param_handles.sys_id_en = param_find("SYS_ID_EN");
	_param_handles.sys_id_axis = param_find("SYS_ID_AXIS");
	_param_handles.sys_id_amp = param_find("SYS_ID_AMP");
	_param_handles.sys_id_interval = param_find("SYS_ID_INTERV");
	_param_handles.sys_id_omega_min = param_find("SYS_ID_OME_MIN");
	_param_handles.sys_id_omega_max = param_find("SYS_ID_OME_MAX");
	_param_handles.sys_id_time_record = param_find("SYS_ID_T_REC");
	_param_handles.sys_id_trim = param_find("SYS_ID_Y_TRIM");

	updateParams();
}

void ActuatorEffectivenessHelicopter::updateParams()
{
	ModuleParams::updateParams();

	if (param_get(_param_handles.num_swash_plate_servos, &_geometry.num_swash_plate_servos) != PX4_OK) {
		PX4_ERR("param_get failed");
		return;
	}

	_geometry.num_swash_plate_servos = math::constrain(_geometry.num_swash_plate_servos,
					   (int32_t)3, (int32_t)NUM_SWASH_PLATE_SERVOS_MAX);

	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
		float angle_deg{};
		param_get(_param_handles.swash_plate_servos[i].angle, &angle_deg);
		_geometry.swash_plate_servos[i].angle = math::radians(angle_deg);
		param_get(_param_handles.swash_plate_servos[i].arm_length, &_geometry.swash_plate_servos[i].arm_length);
		param_get(_param_handles.swash_plate_servos[i].trim, &_geometry.swash_plate_servos[i].trim);
	}

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		param_get(_param_handles.throttle_curve[i], &_geometry.throttle_curve[i]);
		param_get(_param_handles.pitch_curve[i], &_geometry.pitch_curve[i]);
	}

	param_get(_param_handles.yaw_collective_pitch_scale, &_geometry.yaw_collective_pitch_scale);
	param_get(_param_handles.yaw_collective_pitch_offset, &_geometry.yaw_collective_pitch_offset);
	param_get(_param_handles.yaw_throttle_scale, &_geometry.yaw_throttle_scale);
	param_get(_param_handles.spoolup_time, &_geometry.spoolup_time);
	int32_t yaw_ccw = 0;
	param_get(_param_handles.yaw_ccw, &yaw_ccw);
	_geometry.yaw_sign = (yaw_ccw == 1) ? -1.f : 1.f;
	float max_servo_throw_deg = 0.f;
	param_get(_param_handles.max_servo_throw, &max_servo_throw_deg);

	if (max_servo_throw_deg > 0.f) {
		// linearization feature enabled
		_geometry.linearize_servos = 1;
		const float max_servo_throw = math::radians(max_servo_throw_deg);
		_geometry.max_servo_height = sinf(max_servo_throw);
		_geometry.inverse_max_servo_throw = 1.f / max_servo_throw;

	} else {
		// handle any undefined behaviour if disabled
		_geometry.linearize_servos = 0;
		_geometry.max_servo_height = _geometry.inverse_max_servo_throw = 0.f;
	}

	param_get(_param_handles.sys_id_en, &_sys_id.mode);
	param_get(_param_handles.sys_id_axis, &_sys_id.axis);
	param_get(_param_handles.sys_id_amp, &_sys_id.amplitude);
	param_get(_param_handles.sys_id_interval, &_sys_id.interval);
	param_get(_param_handles.sys_id_omega_min, &_sys_id.omega_min);
	param_get(_param_handles.sys_id_omega_max, &_sys_id.omega_max);
	param_get(_param_handles.sys_id_time_record, &_sys_id.time_record);
	param_get(_param_handles.sys_id_trim, &_sys_id.trim);

	_sys_id.mode = math::constrain(_sys_id.mode,
				       static_cast<int32_t>(SysIdMode::Disabled),
				       static_cast<int32_t>(SysIdMode::Sweep));
	_sys_id.axis = math::constrain(_sys_id.axis,
				       static_cast<int32_t>(SysIdAxis::Roll),
				       static_cast<int32_t>(SysIdAxis::Yaw));
	_sys_id.amplitude = math::constrain(_sys_id.amplitude, 0.f, 1.f);
	_sys_id.interval = math::max(_sys_id.interval, 0.1f);
	_sys_id.omega_min = math::max(_sys_id.omega_min, 0.01f);
	_sys_id.omega_max = math::max(_sys_id.omega_max, 0.01f);
	_sys_id.time_record = math::max(_sys_id.time_record, 1.f);
	_sys_id.trim = math::constrain(_sys_id.trim, -1.f, 1.f);
}

bool ActuatorEffectivenessHelicopter::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// As the allocation is non-linear, we use updateSetpoint() instead of the matrix
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});

	// Tail (yaw) (either ESC or Servo)
	configuration.addActuator(_tail_actuator_type, Vector3f{}, Vector3f{});

	// N swash plate servos
	_first_swash_plate_servo_index = configuration.num_actuators_matrix[0];

	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
		configuration.trim[configuration.selected_matrix](i) = _geometry.swash_plate_servos[i].trim;
	}

	return true;
}

void ActuatorEffectivenessHelicopter::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	_saturation_flags = {};

	const float spoolup_progress = throttleSpoolupProgress();
	float rpm_control_output = 0;
#if CONTROL_ALLOCATOR_RPM_CONTROL
	_rpm_control.setSpoolupProgress(spoolup_progress);
	rpm_control_output = _rpm_control.getActuatorCorrection();
#endif // CONTROL_ALLOCATOR_RPM_CONTROL
	float sys_id_excitation = 0.f;
	float sys_id_frequency = 0.f;
	float sys_id_elapsed_time = 0.f;
	float pure_servo[actuator_servos_s::NUM_CONTROLS];
	float sys_id_servo[actuator_servos_s::NUM_CONTROLS] {};
	float requested_servo[actuator_servos_s::NUM_CONTROLS];

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) {
		pure_servo[i] = NAN;
		requested_servo[i] = NAN;
	}

	const float sys_id_signal = updateSysIdSignal(sys_id_excitation, sys_id_frequency, sys_id_elapsed_time);

	// throttle/collective pitch curve
	const float throttle = (math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.throttle_curve)
				+ rpm_control_output) * spoolup_progress;
	const float collective_pitch = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.pitch_curve);
	const float pure_delta_lat = control_sp(ControlAxis::ROLL);
	const float pure_delta_lon = control_sp(ControlAxis::PITCH);
	const float pure_delta_col = collective_pitch;
	float sys_id_delta_lon = 0.f;
	float sys_id_delta_lat = 0.f;
	float sys_id_delta_col = 0.f;
	float sys_id_delta_ped = 0.f;

	// actuator mapping
	actuator_sp(0) = mainMotorEnaged() ? throttle : NAN;

	actuator_sp(1) = control_sp(ControlAxis::YAW) * _geometry.yaw_sign
			 + fabsf(collective_pitch - _geometry.yaw_collective_pitch_offset) * _geometry.yaw_collective_pitch_scale
			 + throttle * _geometry.yaw_throttle_scale;

	const float pure_delta_ped = actuator_sp(1);
	const int tail_servo_index = servoIndexFromActuatorIndex(1);

	if (tail_servo_index >= 0 && tail_servo_index < actuator_servos_s::NUM_CONTROLS) {
		pure_servo[tail_servo_index] = actuator_sp(1);
	}

	if (_sys_id.axis == static_cast<int32_t>(SysIdAxis::Yaw)) {
		const float injection = sys_id_signal * _geometry.yaw_sign;
		actuator_sp(1) += injection;
		sys_id_delta_ped = injection;

		if (tail_servo_index >= 0 && tail_servo_index < actuator_servos_s::NUM_CONTROLS) {
			sys_id_servo[tail_servo_index] = injection;
		}
	}

	if (tail_servo_index >= 0 && tail_servo_index < actuator_servos_s::NUM_CONTROLS) {
		requested_servo[tail_servo_index] = actuator_sp(1);
	}

	if (_sys_id.axis == static_cast<int32_t>(SysIdAxis::Roll)) {
		sys_id_delta_lat = sys_id_signal;

	} else if (_sys_id.axis == static_cast<int32_t>(SysIdAxis::Pitch)) {
		sys_id_delta_lon = sys_id_signal;
	}

	// Saturation check for yaw
	if (actuator_sp(1) < actuator_min(1)) {
		setSaturationFlag(_geometry.yaw_sign, _saturation_flags.yaw_neg, _saturation_flags.yaw_pos);

	} else if (actuator_sp(1) > actuator_max(1)) {
		setSaturationFlag(_geometry.yaw_sign, _saturation_flags.yaw_pos, _saturation_flags.yaw_neg);
	}

	for (int i = 0; i < _geometry.num_swash_plate_servos; i++) {
		const int actuator_index = _first_swash_plate_servo_index + i;
		const int servo_index = servoIndexFromActuatorIndex(actuator_index);
		float roll_coeff = sinf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
		float pitch_coeff = cosf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
		actuator_sp(actuator_index) = collective_pitch
				+ control_sp(ControlAxis::PITCH) * pitch_coeff
				- control_sp(ControlAxis::ROLL) * roll_coeff
				+ _geometry.swash_plate_servos[i].trim;

		// Apply linearization to the actuator setpoint if enabled
		if (_geometry.linearize_servos) {
			actuator_sp(actuator_index) = getLinearServoOutput(actuator_sp(actuator_index));
		}

		if (servo_index >= 0 && servo_index < actuator_servos_s::NUM_CONTROLS) {
			pure_servo[servo_index] = actuator_sp(actuator_index);
		}

		float injection = 0.f;

		if (_sys_id.axis == static_cast<int32_t>(SysIdAxis::Roll)) {
			injection = -sys_id_signal * roll_coeff;

		} else if (_sys_id.axis == static_cast<int32_t>(SysIdAxis::Pitch)) {
			injection = sys_id_signal * pitch_coeff;
		}

		if (fabsf(injection) > FLT_EPSILON) {
			actuator_sp(actuator_index) += injection;

			if (servo_index >= 0 && servo_index < actuator_servos_s::NUM_CONTROLS) {
				sys_id_servo[servo_index] = injection;
			}
		}

		if (servo_index >= 0 && servo_index < actuator_servos_s::NUM_CONTROLS) {
			requested_servo[servo_index] = actuator_sp(actuator_index);
		}

		// Saturation check for roll & pitch
		if (actuator_sp(actuator_index) < actuator_min(actuator_index)) {
			setSaturationFlag(roll_coeff, _saturation_flags.roll_pos, _saturation_flags.roll_neg);
			setSaturationFlag(pitch_coeff, _saturation_flags.pitch_neg, _saturation_flags.pitch_pos);

		} else if (actuator_sp(actuator_index) > actuator_max(actuator_index)) {
			setSaturationFlag(roll_coeff, _saturation_flags.roll_neg, _saturation_flags.roll_pos);
			setSaturationFlag(pitch_coeff, _saturation_flags.pitch_pos, _saturation_flags.pitch_neg);
		}
	}

	updateSysIdStatus(sys_id_signal, sys_id_excitation, sys_id_frequency, sys_id_elapsed_time,
			  pure_delta_lon, pure_delta_lat, pure_delta_col, pure_delta_ped,
			  sys_id_delta_lon, sys_id_delta_lat, sys_id_delta_col, sys_id_delta_ped,
			  pure_servo, sys_id_servo, requested_servo);
}

float ActuatorEffectivenessHelicopter::updateSysIdSignal(float &excitation, float &frequency, float &elapsed_time)
{
	excitation = 0.f;
	frequency = 0.f;
	elapsed_time = 0.f;

	const hrt_abstime now = hrt_absolute_time();

	if (_sys_id.mode == static_cast<int32_t>(SysIdMode::Disabled)) {
		_sys_id_start_time = 0;
		_sys_id_last_mode = _sys_id.mode;
		_sys_id_last_axis = _sys_id.axis;
		return 0.f;
	}

	if (!_armed) {
		_sys_id_start_time = 0;
		return 0.f;
	}

	if (_sys_id_start_time == 0 || _sys_id_last_mode != _sys_id.mode || _sys_id_last_axis != _sys_id.axis) {
		_sys_id_start_time = now;
		_sys_id_last_mode = _sys_id.mode;
		_sys_id_last_axis = _sys_id.axis;
	}

	elapsed_time = static_cast<float>(now - _sys_id_start_time) * 1e-6f;

	if (elapsed_time > _sys_id.time_record) {
		return 0.f;
	}

	if (_sys_id.mode == static_cast<int32_t>(SysIdMode::Doublet)) {
		const float interval = math::max(_sys_id.interval, 0.1f);
		const float phase_time = fmodf(elapsed_time, interval);
		const float pulse_width = 0.25f * interval;

		if (phase_time < pulse_width) {
			excitation = _sys_id.amplitude;

		} else if (phase_time < 2.f * pulse_width) {
			excitation = -_sys_id.amplitude;
		}

	} else if (_sys_id.mode == static_cast<int32_t>(SysIdMode::Sweep)) {
		const float duration = math::max(_sys_id.time_record, 1.f);
		const float omega_delta = _sys_id.omega_max - _sys_id.omega_min;
		frequency = _sys_id.omega_min + omega_delta * elapsed_time / duration;
		const float phase = _sys_id.omega_min * elapsed_time + 0.5f * omega_delta * elapsed_time * elapsed_time / duration;
		excitation = _sys_id.amplitude * sinf(phase);
	}

	return _sys_id.trim + excitation;
}

int ActuatorEffectivenessHelicopter::servoIndexFromActuatorIndex(int actuator_index) const
{
	if (_tail_actuator_type == ActuatorType::SERVOS) {
		return actuator_index - 1;
	}

	return actuator_index - 2;
}

void ActuatorEffectivenessHelicopter::updateSysIdStatus(float signal, float excitation, float frequency, float elapsed_time,
		float pure_delta_lon, float pure_delta_lat, float pure_delta_col, float pure_delta_ped,
		float sys_id_delta_lon, float sys_id_delta_lat, float sys_id_delta_col, float sys_id_delta_ped,
		const float pure_servo[actuator_servos_s::NUM_CONTROLS],
		const float sys_id_servo[actuator_servos_s::NUM_CONTROLS],
		const float requested_servo[actuator_servos_s::NUM_CONTROLS])
{
	_sys_id_actuator_status.timestamp = hrt_absolute_time();
	_sys_id_actuator_status.timestamp_sample = _sys_id_actuator_status.timestamp;
	_sys_id_actuator_status.mode = static_cast<uint8_t>(_sys_id.mode);
	_sys_id_actuator_status.axis = static_cast<uint8_t>(_sys_id.axis);
	_sys_id_actuator_status.signal = signal;
	_sys_id_actuator_status.excitation = excitation;
	_sys_id_actuator_status.frequency = frequency;
	_sys_id_actuator_status.elapsed_time = elapsed_time;
	_sys_id_actuator_status.pure_delta_lon = pure_delta_lon;
	_sys_id_actuator_status.pure_delta_lat = pure_delta_lat;
	_sys_id_actuator_status.pure_delta_col = pure_delta_col;
	_sys_id_actuator_status.pure_delta_ped = pure_delta_ped;
	_sys_id_actuator_status.sys_id_delta_lon = sys_id_delta_lon;
	_sys_id_actuator_status.sys_id_delta_lat = sys_id_delta_lat;
	_sys_id_actuator_status.sys_id_delta_col = sys_id_delta_col;
	_sys_id_actuator_status.sys_id_delta_ped = sys_id_delta_ped;
	_sys_id_actuator_status.total_delta_lon = pure_delta_lon + sys_id_delta_lon;
	_sys_id_actuator_status.total_delta_lat = pure_delta_lat + sys_id_delta_lat;
	_sys_id_actuator_status.total_delta_col = pure_delta_col + sys_id_delta_col;
	_sys_id_actuator_status.total_delta_ped = pure_delta_ped + sys_id_delta_ped;

	if (_tail_actuator_type == ActuatorType::SERVOS) {
		_sys_id_actuator_status.pure_servo_tail = pure_servo[0];
		_sys_id_actuator_status.pure_servo_swash0 = pure_servo[1];
		_sys_id_actuator_status.pure_servo_swash1 = pure_servo[2];
		_sys_id_actuator_status.pure_servo_swash2 = pure_servo[3];
		_sys_id_actuator_status.pure_servo_swash3 = pure_servo[4];
		_sys_id_actuator_status.sys_id_servo_tail = sys_id_servo[0];
		_sys_id_actuator_status.sys_id_servo_swash0 = sys_id_servo[1];
		_sys_id_actuator_status.sys_id_servo_swash1 = sys_id_servo[2];
		_sys_id_actuator_status.sys_id_servo_swash2 = sys_id_servo[3];
		_sys_id_actuator_status.sys_id_servo_swash3 = sys_id_servo[4];
		_sys_id_actuator_status.requested_servo_tail = requested_servo[0];
		_sys_id_actuator_status.requested_servo_swash0 = requested_servo[1];
		_sys_id_actuator_status.requested_servo_swash1 = requested_servo[2];
		_sys_id_actuator_status.requested_servo_swash2 = requested_servo[3];
		_sys_id_actuator_status.requested_servo_swash3 = requested_servo[4];

	} else {
		_sys_id_actuator_status.pure_servo_tail = NAN;
		_sys_id_actuator_status.pure_servo_swash0 = pure_servo[0];
		_sys_id_actuator_status.pure_servo_swash1 = pure_servo[1];
		_sys_id_actuator_status.pure_servo_swash2 = pure_servo[2];
		_sys_id_actuator_status.pure_servo_swash3 = pure_servo[3];
		_sys_id_actuator_status.sys_id_servo_tail = NAN;
		_sys_id_actuator_status.sys_id_servo_swash0 = sys_id_servo[0];
		_sys_id_actuator_status.sys_id_servo_swash1 = sys_id_servo[1];
		_sys_id_actuator_status.sys_id_servo_swash2 = sys_id_servo[2];
		_sys_id_actuator_status.sys_id_servo_swash3 = sys_id_servo[3];
		_sys_id_actuator_status.requested_servo_tail = NAN;
		_sys_id_actuator_status.requested_servo_swash0 = requested_servo[0];
		_sys_id_actuator_status.requested_servo_swash1 = requested_servo[1];
		_sys_id_actuator_status.requested_servo_swash2 = requested_servo[2];
		_sys_id_actuator_status.requested_servo_swash3 = requested_servo[3];
	}

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) {
		_sys_id_actuator_status.pure_servo[i] = pure_servo[i];
		_sys_id_actuator_status.sys_id_servo[i] = sys_id_servo[i];
		_sys_id_actuator_status.requested_servo[i] = requested_servo[i];
	}
}

void ActuatorEffectivenessHelicopter::publishSysIdActuatorStatus(const actuator_servos_s &actuator_servos)
{
	_sys_id_actuator_status.timestamp = hrt_absolute_time();
	_sys_id_actuator_status.timestamp_sample = actuator_servos.timestamp_sample;

	if (_tail_actuator_type == ActuatorType::SERVOS) {
		_sys_id_actuator_status.servo_tail = actuator_servos.control[0];
		_sys_id_actuator_status.servo_swash0 = actuator_servos.control[1];
		_sys_id_actuator_status.servo_swash1 = actuator_servos.control[2];
		_sys_id_actuator_status.servo_swash2 = actuator_servos.control[3];
		_sys_id_actuator_status.servo_swash3 = actuator_servos.control[4];

	} else {
		_sys_id_actuator_status.servo_tail = NAN;
		_sys_id_actuator_status.servo_swash0 = actuator_servos.control[0];
		_sys_id_actuator_status.servo_swash1 = actuator_servos.control[1];
		_sys_id_actuator_status.servo_swash2 = actuator_servos.control[2];
		_sys_id_actuator_status.servo_swash3 = actuator_servos.control[3];
	}

	_sys_id_actuator_pub.publish(_sys_id_actuator_status);
}

float ActuatorEffectivenessHelicopter::getLinearServoOutput(float input) const
{
	input = math::constrain(input, -1.f, 1.f);

	// make sure a the maximal input of [-1,1] maps to the maximal vertical deflection the servo can reach of sin(CA_MAX_SVO_THROW)
	float servo_height = _geometry.max_servo_height * input;

	// mulitply by 1 over max arm roation in radians to normalise
	return _geometry.inverse_max_servo_throw * asinf(servo_height);
}

bool ActuatorEffectivenessHelicopter::mainMotorEnaged()
{
	manual_control_switches_s manual_control_switches;

	if (_manual_control_switches_sub.update(&manual_control_switches)) {
		_main_motor_engaged = manual_control_switches.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_NONE
				      || manual_control_switches.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_ON;
	}

	return _main_motor_engaged;
}

float ActuatorEffectivenessHelicopter::throttleSpoolupProgress()
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_armed_time = vehicle_status.armed_time;
	}

	const float time_since_arming = (hrt_absolute_time() - _armed_time) / 1e6f;
	const float spoolup_progress = time_since_arming / _geometry.spoolup_time;

	if (_armed && spoolup_progress < 1.f) {
		return spoolup_progress;
	}

	return 1.f;
}


void ActuatorEffectivenessHelicopter::setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag)
{
	if (coeff > 0.f) {
		// A positive change in given axis will increase saturation
		positive_flag = true;

	} else if (coeff < 0.f) {
		// A negative change in given axis will increase saturation
		negative_flag = true;
	}
}

void ActuatorEffectivenessHelicopter::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_saturation_flags.roll_pos) {
		status.unallocated_torque[0] = 1.f;

	} else if (_saturation_flags.roll_neg) {
		status.unallocated_torque[0] = -1.f;

	} else {
		status.unallocated_torque[0] = 0.f;
	}

	if (_saturation_flags.pitch_pos) {
		status.unallocated_torque[1] = 1.f;

	} else if (_saturation_flags.pitch_neg) {
		status.unallocated_torque[1] = -1.f;

	} else {
		status.unallocated_torque[1] = 0.f;
	}

	if (_saturation_flags.yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_saturation_flags.yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}

	if (_saturation_flags.thrust_pos) {
		status.unallocated_thrust[2] = 1.f;

	} else if (_saturation_flags.thrust_neg) {
		status.unallocated_thrust[2] = -1.f;

	} else {
		status.unallocated_thrust[2] = 0.f;
	}
}
