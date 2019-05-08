/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file pwm_limit.c
 *
 * Library for PWM output limiting
 *
 * @author Julian Oes <julian@px4.io>
 */

#ifndef PWM_LIMIT_H_
#define PWM_LIMIT_H_

#include <stdint.h>
#include <stdbool.h>

__BEGIN_DECLS

/*
 * time for the ESCs to initialize
 * (this is not actually needed if PWM is sent right after boot)
 */
#define INIT_TIME_US 500000
/*
 * time to slowly ramp up the ESCs
 */
#define RAMP_TIME_US 2500000

enum pwm_limit_state {
	PWM_LIMIT_STATE_OFF = 0,
	PWM_LIMIT_STATE_INIT,
	PWM_LIMIT_STATE_RAMP,
	PWM_LIMIT_STATE_ON
};

typedef struct {
	enum pwm_limit_state state;
	uint64_t time_armed;
} pwm_limit_t;

__EXPORT void pwm_limit_init(pwm_limit_t *limit);

__EXPORT void pwm_limit_calc(const bool armed, const bool pre_armed, const unsigned num_channels,
			     const uint16_t reverse_mask, const uint16_t *disarmed_pwm,
			     const uint16_t *min_pwm, const uint16_t *max_pwm, const float *output, uint16_t *effective_pwm, pwm_limit_t *limit);

__END_DECLS

#endif /* PWM_LIMIT_H_ */
