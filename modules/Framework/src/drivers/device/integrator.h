/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file integrator.h
 *
 * A resettable integrator
 *
 * @author Lorenz Meier <lorenz@dp.io>
 */

#pragma once

#include <mathlib/mathlib.h>

class __EXPORT Integrator
{
public:
	Integrator(uint64_t auto_reset_interval = 4000 /* 250 Hz */, bool coning_compensation = false);
	virtual ~Integrator();

	/**
	 * Put an item into the integral.
	 *
	 * @param timestamp	Timestamp of the current value
	 * @param val		Item to put
	 * @param integral	Current integral in case the integrator did reset, else the value will not be modified
	 * @return		true if putting the item triggered an integral reset
	 *			and the integral should be published
	 */
	bool			put(uint64_t timestamp, math::Vector<3> &val, math::Vector<3> &integral, uint64_t &integral_dt);

	/**
	 * Get the current integral value
	 *
	 * @return		the integral since the last auto-reset
	 */
	math::Vector<3>		get() { return _integral_auto; }

	/**
	 * Read from the integral
	 *
	 * @param auto_reset	Reset the integral to zero on read
	 * @return		the integral since the last read-reset
	 */
	math::Vector<3>		read(bool auto_reset);

	/**
	 * Get current integral start time
	 */
	uint64_t		current_integral_start() { return _last_auto; }

private:
	uint64_t _auto_reset_interval;		/**< the interval after which the content will be published and the integrator reset */
	uint64_t _last_integration;			/**< timestamp of the last integration step */
	uint64_t _last_auto;				/**< last auto-announcement of integral value */
	math::Vector<3> _integral_auto;			/**< the integrated value which auto-resets after _auto_reset_interval */
	math::Vector<3> _integral_read;			/**< the integrated value since the last read */
	math::Vector<3> _last_val;			/**< previously integrated last value */
	math::Vector<3> _last_delta;			/**< last local delta */
	void (*_auto_callback)(uint64_t, math::Vector<3>);	/**< the function callback for auto-reset */
	bool _coning_comp_on;				/**< coning compensation */

	/* we don't want this class to be copied */
	Integrator(const Integrator &);
	Integrator operator=(const Integrator &);
};
