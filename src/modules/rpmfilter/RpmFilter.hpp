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
#include <matrix/matrix/math.hpp>
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

	////////////TODO Decide if splitting to separate file
	void rpmFilterInit();

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

	//TODO Decide if splitting into different files

	static const uint8_t _MAX_SUPPORTED_MOTORS = 12;
	static const uint8_t _MAX_HARMONICS = 3;

	typedef struct rpmNotchFilter_s {
		uint8_t harmonics;
		float   minHz;
		float   maxHz;
		float   bandwidth;
		float   sample_freq;

		math::NotchFilter<matrix::Vector3f>
		notch_vector3f[_MAX_SUPPORTED_MOTORS][_MAX_HARMONICS]; //TODO: replace hardcoded values by MOTOR_NUMBER, NUMBER_HARMONICS
	} rpmNotchFilter_t;

	//TODO: Only compiles by removing the static and constexpr terms, this is because those values need to be initalised outside of the function!!
	// Look into this further as it will modify how the code behaves, as those values are both used in init and update functions
	// cannot initialise them here as against ISO standards, need to initialise outside of function
	// intialising the vars in cpp gives error: ‘uint8_t RpmFilter::currentMotor’ is private within this context
	// see https://stackoverflow.com/questions/20310000/error-iso-c-forbids-in-class-initialization-of-non-const-static-member for potential solution
	// run a test, it may be just fine and how the class works by design.
	// var
	uint8_t numberFilters; //total number of low pass filters for motors
	uint8_t numberRpmNotchFilters; //counter for rpm filters
	uint8_t filterUpdatesPerIteration; // ceil(filtersPerLoopIteration)
	esc_status_s esc_status; //structure where the esc telemetry data is stored.
	static float   filteredMotorRpm[_MAX_SUPPORTED_MOTORS]; //stores filtered Erpm value for each motor

	//structures
	rpmNotchFilter_t filters[2]; // structure size.
	rpmNotchFilter_t
	*gyroFilter; //filter instance for gyro filtering !don't create instances, instead one instance that works for everything.
	rpmNotchFilter_t *dtermFilter; //filter instance for dterm filtering

	// iterators
	uint8_t currentMotor; //iterates through motors
	uint8_t currentHarmonic; //iterates through harmonics
	uint8_t currentFilterNumber; //iterates through filter index
	rpmNotchFilter_t *currentFilter = &filters[0]; //iterates through filters

	//? Make sure OK to have as private.
	void rpmNotchFilterInit(rpmNotchFilter_t *filter, int harmonics, int minHz, float bandwidth, float sample_freq);
	//! This is not OK to have as private. needs a rework
	void apply(rpmNotchFilter_t *filter, matrix::Vector3f input);

};
