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

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/sys_id_actuator.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_switches.h>

#include "RpmControl.hpp"

class ActuatorEffectivenessHelicopter : public ModuleParams, public ActuatorEffectiveness
{
public:

	static constexpr int NUM_SWASH_PLATE_SERVOS_MAX = 4;
	static constexpr int NUM_CURVE_POINTS = 5;

	struct SwashPlateGeometry {
		float angle;
		float arm_length;
		float trim;
	};

	struct Geometry {
		SwashPlateGeometry swash_plate_servos[NUM_SWASH_PLATE_SERVOS_MAX];
		int32_t num_swash_plate_servos{0};
		float throttle_curve[NUM_CURVE_POINTS];
		float pitch_curve[NUM_CURVE_POINTS];
		float yaw_collective_pitch_scale;
		float yaw_collective_pitch_offset;
		float yaw_throttle_scale;
		float yaw_sign;
		float spoolup_time;
		int linearize_servos;
		float max_servo_height;
		float inverse_max_servo_throw;
	};

	ActuatorEffectivenessHelicopter(ModuleParams *parent, ActuatorType tail_actuator_type);
	virtual ~ActuatorEffectivenessHelicopter() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	const char *name() const override { return "Helicopter"; }


	const Geometry &geometry() const { return _geometry; }

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;
private:
	float throttleSpoolupProgress();
	bool mainMotorEnaged();
	float getLinearServoOutput(float input) const;

	void updateParams() override;

	struct SaturationFlags {
		bool roll_pos;
		bool roll_neg;
		bool pitch_pos;
		bool pitch_neg;
		bool yaw_pos;
		bool yaw_neg;
		bool thrust_pos;
		bool thrust_neg;
	};
	static void setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag);

	struct ParamHandlesSwashPlate {
		param_t angle;
		param_t arm_length;
		param_t trim;
	};
	struct ParamHandles {
		ParamHandlesSwashPlate swash_plate_servos[NUM_SWASH_PLATE_SERVOS_MAX];
		param_t num_swash_plate_servos;
		param_t throttle_curve[NUM_CURVE_POINTS];
		param_t pitch_curve[NUM_CURVE_POINTS];
		param_t yaw_collective_pitch_scale;
		param_t yaw_collective_pitch_offset;
		param_t yaw_throttle_scale;
		param_t yaw_ccw;
		param_t spoolup_time;
		param_t max_servo_throw;
		param_t sys_id_en;
		param_t sys_id_axis;
		param_t sys_id_rc_mode;
		param_t sys_id_rc_axis;
		param_t sys_id_amp;
		param_t sys_id_interval;
		param_t sys_id_omega_min;
		param_t sys_id_omega_max;
		param_t sys_id_time_record;
	};
	ParamHandles _param_handles{};

	Geometry _geometry{};

	enum class SysIdMode : int32_t {
		Disabled = sys_id_actuator_s::MODE_DISABLED,
		ThreeTwoOneOne = sys_id_actuator_s::MODE_3211,
		Sweep = sys_id_actuator_s::MODE_SWEEP
	};

	enum class SysIdAxis : int32_t {
		Roll = sys_id_actuator_s::AXIS_ROLL,
		Pitch = sys_id_actuator_s::AXIS_PITCH,
		Yaw = sys_id_actuator_s::AXIS_YAW
	};

	struct SysIdConfig {
		int32_t mode_param{0};
		int32_t axis_param{0};
		int32_t mode{0};
		int32_t axis{0};
		int32_t rc_mode_channel{0};
		int32_t rc_axis_channel{0};
		float amplitude{0.f};
		float interval{0.f};
		float omega_min{0.f};
		float omega_max{0.f};
		float time_record{0.f};
	};

	SysIdConfig _sys_id{};
	int32_t _sys_id_last_mode{0};
	int32_t _sys_id_last_axis{0};
	hrt_abstime _sys_id_start_time{0};
	sys_id_actuator_s _sys_id_actuator_status{};
	uORB::Publication<sys_id_actuator_s> _sys_id_actuator_pub{ORB_ID(sys_id_actuator)};

	int _first_swash_plate_servo_index{};
	SaturationFlags _saturation_flags;

	// Throttle spoolup state
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	bool _armed{false};
	uint64_t _armed_time{0};

	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};
	bool _main_motor_engaged{true};

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s _manual_control_setpoint{};

	const ActuatorType _tail_actuator_type;

#if CONTROL_ALLOCATOR_RPM_CONTROL
	RpmControl _rpm_control {this};
#endif // CONTROL_ALLOCATOR_RPM_CONTROL

	void updateSysIdRcSelection();
	float sysIdRcAuxValue(int32_t channel) const;
	float updateSysIdSignal();
	void updateSysIdStatus(
				float pure_delta_lon, float pure_delta_lat, float pure_delta_col, float pure_delta_ped,
				float sys_id_delta_lon, float sys_id_delta_lat, float sys_id_delta_col, float sys_id_delta_ped);
};
