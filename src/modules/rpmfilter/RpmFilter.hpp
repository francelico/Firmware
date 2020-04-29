/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file RpmFilter.hpp
 *
 * @brief Implementation of multiple instances of NotchFilters to filter
 * out each motor frequencies and their harmonics on the gyro, Dterm and
 * accelerometer data.
 *
 * @author Samuel Garcin <samuel.garcin@wecorpindustries.com>
 */

#pragma once


#include <mathlib/mathlib.h> // include constrainf
//#include <mathlib/math/filter/NotchFilter.hpp>
#include <mathlib/math/filter/NotchFilterArray.hpp>
#include <lib/perf/perf_counter.h> //TODO check how to use
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/parameter_update.h> //TODO check if needed

class RpmFilter : public ModuleBase<RpmFilter>, public ModuleParams,
	public px4::WorkItem
{
public:

	typedef struct rpmNotchFilter_s {
		uint8_t harmonics;
		uint8_t motor_number;
		float   minHz;
		float   maxHz;
		float   bandwitdh;
		float   sample_freq;

		math::NotchFilter<matrix::Vector3f>
		notch_vector3f[6][3]; //TODO: replace hardcoded values by MOTOR_NUMBER, NUMBER_HARMONICS
	} rpmNotchFilter_t;

	RpmFilter();
	~RpmFilter() override;

	//TODO see if needed
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	void Run() override; //TODO see if needed? maybe it needs to be done differently here
	void updateParams() override;

	void reset();
	//void publishStatus(TODO::status &status); //TODO later

	uORB::SubscriptionCallbackWorkItem _esc_status_sub{this, ORB_ID(esc_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)}; //TODO check if needed

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover //placeholder
	)

};
