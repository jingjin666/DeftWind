/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file integrator.cpp
 *
 * A resettable integrator
 *
 * @author Lorenz Meier <lorenz@dp.io>
 */

#include "integrator.h"

Integrator::Integrator(uint64_t auto_reset_interval, bool coning_compensation) :
	_auto_reset_interval(auto_reset_interval),
	_last_integration(0),
	_last_auto(0),
	_integral_auto(0.0f, 0.0f, 0.0f),
	_integral_read(0.0f, 0.0f, 0.0f),
	_last_val(0.0f, 0.0f, 0.0f),
	_last_delta(0.0f, 0.0f, 0.0f),
	_auto_callback(nullptr),
	_coning_comp_on(coning_compensation)
{

}

Integrator::~Integrator()
{

}

bool
Integrator::put(uint64_t timestamp, math::Vector<3> &val, math::Vector<3> &integral, uint64_t &integral_dt)
{
	bool auto_reset = false;

	if (_last_integration == 0) {
		/* this is the first item in the integrator */
		_last_integration = timestamp;
		_last_auto = timestamp;
		_last_val = val;
		return false;
	}

	// Integrate
	double dt = (double)(timestamp - _last_integration) / 1000000.0;
	math::Vector<3> i = (val + _last_val) * dt * 0.5f;

	// Apply coning compensation if required
	if (_coning_comp_on) {
		// Coning compensation derived by Paul Riseborough and Jonathan Challinger,
		// following:
		// Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
		// Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf

		i += ((_integral_auto + _last_delta * (1.0f / 6.0f)) % i) * 0.5f;
	}

	_integral_auto += i;
	_integral_read += i;

	_last_integration = timestamp;
	_last_val = val;
	_last_delta = i;

	if ((timestamp - _last_auto) > _auto_reset_interval) {
		if (_auto_callback) {
			/* call the callback */
			_auto_callback(timestamp, _integral_auto);
		}

		integral = _integral_auto;
		integral_dt = (timestamp - _last_auto);

		auto_reset = true;
		_last_auto = timestamp;
		_integral_auto(0) = 0.0f;
		_integral_auto(1) = 0.0f;
		_integral_auto(2) = 0.0f;
	}

	return auto_reset;
}

math::Vector<3>
Integrator::read(bool auto_reset)
{
	math::Vector<3> val = _integral_read;

	if (auto_reset) {
		_integral_read(0) = 0.0f;
		_integral_read(1) = 0.0f;
		_integral_read(2) = 0.0f;
	}

	return val;
}
