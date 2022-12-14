/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file drv_io_timer.h
 *
 * imxrt-specific PWM output data.
 */
#include <dp_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <drivers/drv_hrt.h>

#pragma once
__BEGIN_DECLS
/* configuration limits */
#define MAX_IO_TIMERS			4
#define MAX_TIMER_IO_CHANNELS	16

#define MAX_LED_TIMERS			2
#define MAX_TIMER_LED_CHANNELS	6

#define IO_TIMER_ALL_MODES_CHANNELS 0

typedef enum io_timer_channel_mode_t {
	IOTimerChanMode_NotUsed = 0,
	IOTimerChanMode_PWMOut  = 1,
	IOTimerChanMode_PWMIn   = 2,
	IOTimerChanMode_Capture = 3,
	IOTimerChanMode_OneShot = 4,
	IOTimerChanMode_Trigger = 5,
	IOTimerChanModeSize
} io_timer_channel_mode_t;

typedef uint16_t io_timer_channel_allocation_t; /* big enough to hold MAX_TIMER_IO_CHANNELS */

/* array of timers dedicated to PWM in and out and TBD capture use
 *** Timers are driven from QTIMER3_OUT0
 *** In PWM  mode the timer's prescaler is set to achieve a counter frequency of 1MHz
 *** In OneShot mode the timer's prescaler is set to achieve a counter frequency of 8MHz
 *** Other prescaler rates can be achieved by fore instance by setting the clock_freq = 1Mhz
 *** the resulting PSC will be one and the timer will count at it's clock frequency.
 */
typedef struct io_timers_t {
	uint32_t  base;                /* Base address of the timer */
	uint32_t  clock_register;      /* SIM_SCGCn */
	uint32_t  clock_bit;           /* SIM_SCGCn bit pos */
	uint32_t  vectorno;            /* IRQ number */
	uint32_t  first_channel_index; /* 0 based index in timer_io_channels */
	uint32_t  last_channel_index;  /* 0 based index in timer_io_channels */
	xcpt_t    handler;
} io_timers_t;

/* array of channels in logical order */
typedef struct timer_io_channels_t {
	uint32_t	gpio_out;            /* The timer valn_offset GPIO for PWM */
	uint32_t	gpio_in;             /* The timer valn_offset GPIO for Capture */
	uint8_t		timer_index;         /* 0 based index in the io_timers_t table */
	uint8_t   val_offset;          /* IMXRT_FLEXPWM_SM0VAL3_OFFSET or IMXRT_FLEXPWM_SM0VAL5_OFFSET */
	uint8_t   sub_module;          /* 0 based sub module offset */
	uint8_t   sub_module_bits;     /* LDOK and CLDOK bits */
} timer_io_channels_t;

#define SM0           0
#define SM1           1
#define SM2           2
#define SM3           3

#define PWMA_VAL      IMXRT_FLEXPWM_SM0VAL3_OFFSET
#define PWMB_VAL      IMXRT_FLEXPWM_SM0VAL5_OFFSET


typedef void (*channel_handler_t)(void *context, const io_timers_t *timer, uint32_t chan_index,
				  const timer_io_channels_t *chan,
				  hrt_abstime isrs_time, uint16_t isrs_rcnt,
				  uint16_t capture);


/* supplied by board-specific code */
__EXPORT extern const io_timers_t io_timers[MAX_IO_TIMERS];
__EXPORT extern const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS];

__EXPORT extern const io_timers_t led_pwm_timers[MAX_LED_TIMERS];
__EXPORT extern const timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS];

__EXPORT extern io_timer_channel_allocation_t allocations[IOTimerChanModeSize];
__EXPORT int io_timer_handler0(int irq, void *context, void *arg);
__EXPORT int io_timer_handler1(int irq, void *context, void *arg);
__EXPORT int io_timer_handler2(int irq, void *context, void *arg);
__EXPORT int io_timer_handler3(int irq, void *context, void *arg);

__EXPORT int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
				   channel_handler_t channel_handler, void *context);

__EXPORT int io_timer_init_timer(unsigned timer);

__EXPORT int io_timer_set_rate(unsigned timer, unsigned rate);
__EXPORT int io_timer_set_enable(bool state, io_timer_channel_mode_t mode,
				 io_timer_channel_allocation_t masks);
__EXPORT int io_timer_set_rate(unsigned timer, unsigned rate);
__EXPORT uint16_t io_channel_get_ccr(unsigned channel);
__EXPORT int io_timer_set_ccr(unsigned channel, uint16_t value);
__EXPORT uint32_t io_timer_get_group(unsigned timer);
__EXPORT int io_timer_validate_channel_index(unsigned channel);
__EXPORT int io_timer_is_channel_free(unsigned channel);
__EXPORT int io_timer_free_channel(unsigned channel);
__EXPORT int io_timer_get_channel_mode(unsigned channel);
__EXPORT int io_timer_get_mode_channels(io_timer_channel_mode_t mode);
__EXPORT extern void io_timer_trigger(void);

__END_DECLS
