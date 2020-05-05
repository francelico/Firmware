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
 * @file RpmFilter.cpp
 *
 * @author Samuel Garcin <samuel.garcin@wecorpindustries.com>
 */

#include <math.h>
#include <stdint.h>

// #include "NotchFilter.hpp" should be in hpp file?
#include "RpmFilter.hpp"

#define MOTORS_NUMBER 6 //TODO replace with esc_status.esc_count and make cleaner REPLACE by static constexpr
#define SAMPLE_FREQ 1000 //TODO replace with pid loop rate parameter

RpmFilter::RpmFilter() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	updateParams();
	reset();
}

RpmFilter::~RpmFilter()
{
	perf_free(_cycle_perf);
}

bool RpmFilter::init()
{
	if (!_esc_status_sub.registerCallback()) {
		PX4_ERR("esc_status callback registration failed!");
		return false;
	}

	return true;
}

void RpmFilter::reset()
{
	//TODO
}

void RpmFilter::updateParams()
{
	//TODO
}

void RpmFilter::Run()
{
	if (should_exit()) {
		_esc_status_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	perf_begin(_cycle_perf);

	//TODO TODO TODO

	//publishStatus(status); //TODO later

	perf_end(_cycle_perf);

}

int RpmFilter::task_spawn(int argc, char *argv[])
{
	RpmFilter *instance = new RpmFilter();

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

int RpmFilter::custom_command(int argc, char *argv[])
{
	return print_usage("unkown command");
}

int RpmFilter::print_status()
{
	perf_print_counter(_cycle_perf);

	return 0;
}

int RpmFilter::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("rpmfilter", "filter");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int rpmfilter_main(int argc, char *argv[])
{
    return RpmFilter::main(argc, argv);
}

// TODO CREATE A SPLIT CLASS FROM HERE (MAYBE)

//? consider merging this with the init function
void RpmFilter::rpmNotchFilterInit(rpmNotchFilter_t* filter, int harmonics, int minHz, float bandwidth, float sample_freq)
{
    filter->harmonics = harmonics;
    filter->minHz = minHz;
    filter->bandwidth = bandwidth;
    filter->sample_freq = sample_freq;

    for (int motor = 0; motor < MOTORS_NUMBER; motor++) {
        for (int i = 0; i < harmonics; i++) {
            filter->notch_vector3f[motor][i].setParameters(filter->sample_freq, minHz * (i + 1), filter->bandwidth);
        }
    }
}

void RpmFilter::rpmFilterInit()
{

    //TODO: hardcoded at the moment. to add as SetParam
    uint8_t gyro_rpm_notch_harmonics = 3;
    uint8_t gyro_rpm_notch_min = 20;
    uint16_t gyro_rpm_notch_bandwidth = 500;

    // //parameters for dterm config
    uint8_t dterm_rpm_notch_harmonics = 0; //number of harmonics different
    uint8_t dterm_rpm_notch_min = 100;
    uint16_t dterm_rpm_notch_bandwidth = 500;

    //uint16_t rpm_lpf = 150; //TODO low pass filter frequency cut (everything cut under 150hz == 9000rpm)

    //initialise iterators
    //currentFilter = &filters[0]; !already inetialised in hpp, shouldn't need again
    currentMotor = currentHarmonic = currentFilterNumber = 0;
    numberRpmNotchFilters = 0;

    if (gyro_rpm_notch_harmonics) {
        gyroFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(gyroFilter, gyro_rpm_notch_harmonics,
                           gyro_rpm_notch_min, gyro_rpm_notch_bandwidth, SAMPLE_FREQ);
        // don't go quite to nyquist to avoid oscillations
        gyroFilter->maxHz = 0.48f * SAMPLE_FREQ;
    } else {
        gyroFilter = NULL;
    }
    if (dterm_rpm_notch_harmonics) {
        dtermFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(dtermFilter, dterm_rpm_notch_harmonics,
                           dterm_rpm_notch_min, dterm_rpm_notch_bandwidth, SAMPLE_FREQ);
        // don't go quite to nyquist to avoid oscillations
        dtermFilter->maxHz = 0.48f * SAMPLE_FREQ;
    } else {
        dtermFilter = NULL;
    }

    //TODO lpf for rpm, implement later

    //TODO consider refining later, replace MOTORS_NUMBER by esc count, add accelerometer filters if needed.
    numberFilters = MOTORS_NUMBER * (filters[0].harmonics + filters[1].harmonics);
    filterUpdatesPerIteration = numberFilters;
}
