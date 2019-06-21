/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/*
 * @file timer_config.c
 *
 * Configuration data for the imxrt pwm_servo, input capture and pwm input driver.
 *
 * Note that these arrays must always be fully-sized.
 */

// TODO:Stubbed out for now
#include <stdint.h>

#include <chip.h>
#include "chip/imxrt_tmr.h"
#include "chip/imxrt_flexpwm.h"
#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include "chip/imxrt_pinmux.h"
#include "imxrt_xbar.h"
#include "imxrt_periphclks.h"
#include <drivers/drv_pwm_output.h>
#include <drivers/imxrt/drv_io_timer.h>

#include "up_arch.h"
#include "board_config.h"

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Register accessors */

#define _REG(_addr) (*(volatile uint16_t *)(_addr))

/* QTimer3 register accessors */

#define REG(_reg) _REG(IMXRT_QTIMER3_BASE + IMXRT_TMR_OFFSET(IMXRT_TMR_CH0,(_reg)))

#define rCOMP1        REG(IMXRT_TMR_COMP1_OFFSET)
#define rCOMP2        REG(IMXRT_TMR_COMP2_OFFSET)
#define rCAPT         REG(IMXRT_TMR_CAPT_OFFSET)
#define rLOAD         REG(IMXRT_TMR_LOAD_OFFSET)
#define rHOLD         REG(IMXRT_TMR_HOLD_OFFSET)
#define rCNTR         REG(IMXRT_TMR_CNTR_OFFSET)
#define rCTRL         REG(IMXRT_TMR_CTRL_OFFSET)
#define rSCTRL        REG(IMXRT_TMR_SCTRL_OFFSET)
#define rCMPLD1       REG(IMXRT_TMR_CMPLD1_OFFSET)
#define rCMPLD2       REG(IMXRT_TMR_CMPLD2_OFFSET)
#define rCSCTRL       REG(IMXRT_TMR_CSCTRL_OFFSET)
#define rFILT         REG(IMXRT_TMR_FILT_OFFSET)
#define rDMA          REG(IMXRT_TMR_DMA_OFFSET)
#define rENBL         REG(IMXRT_TMR_ENBL_OFFSET)

__EXPORT const io_timers_t io_timers[MAX_IO_TIMERS] = {
	{
		.base = IMXRT_FLEXPWM2_BASE,
		.first_channel_index = 0,
		.last_channel_index  = 5,
	},
	{
		.base = IMXRT_FLEXPWM4_BASE,
		.first_channel_index = 6,
		.last_channel_index  = 9,
	},
    {
        .base = IMXRT_FLEXPWM1_BASE,
        .first_channel_index = 10,
        .last_channel_index  = 12,
    },
	{
		.base = IMXRT_FLEXPWM3_BASE,
		.first_channel_index = 13,
		.last_channel_index  = 14,
	},
};

__EXPORT const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	{
		/* FMU_CH1 : GPIO_EMC_06   GPIO4_IO6  FLEXPWM2_PWMA00 */
		.gpio_out         = PIN_FLEXPWM2_PWMA00,
		.timer_index      = 0,
		.val_offset       = PWMA_VAL,
		.sub_module       = SM0,
		.sub_module_bits  = MCTRL_LDOK(1 << SM0),
	},
	{
		/* FMU_CH2 : GPIO_EMC_10   GPIO4_IO10 FLEXPWM2_PWMA02 */
		.gpio_out         = PIN_FLEXPWM2_PWMA02,
		.timer_index      = 0,
		.val_offset       = PWMA_VAL,
		.sub_module       = SM2,
		.sub_module_bits  = MCTRL_LDOK(1 << SM2),
	},
	{
		/* FMU_CH3 : GPIO_B1_02 GPIO2_IO18  FLEXPWM2_PWMA03 */
		.gpio_out         = PIN_FLEXPWM2_PWMA03,
		.timer_index      = 0,
		.val_offset       = PWMA_VAL,
		.sub_module       = SM3,
		.sub_module_bits  = MCTRL_LDOK(1 << SM3),
	},
	{
		/* FMU_CH4 : GPIO_EMC_07   GPIO4_IO7 FLEXPWM2_PWMB00 */
		.gpio_out         = PIN_FLEXPWM2_PWMB00,
		.timer_index      = 0,
		.val_offset       = PWMB_VAL,
		.sub_module       = SM0,
		.sub_module_bits  =  MCTRL_LDOK(1 << SM0),
	},
	{
		/* FMU_CH5 : GPIO_EMC_09   GPIO4_IO9  FLEXPWM2_PWMB01 */
		.gpio_out         = PIN_FLEXPWM2_PWMB01,
		.timer_index      = 0,
		.val_offset       = PWMB_VAL,
		.sub_module       = SM1,
		.sub_module_bits  = MCTRL_LDOK(1 << SM1),
	},
	{
		/* FMU_CH6 : GPIO_B1_03   GPIO2_IO19  FLEXPWM2_PWMB03 */
		.gpio_out         = PIN_FLEXPWM2_PWMB03,
		.timer_index      = 0,
		.val_offset       = PWMB_VAL,
		.sub_module       = SM3,
		.sub_module_bits  = MCTRL_LDOK(1 << SM3),
	},
	{
		/* FMU_CH7 : GPIO_AD_B1_08   GPIO1_IO24  FLEXPWM4_PWMA00 */
		.gpio_out         = PIN_FLEXPWM4_PWMA00,
		.timer_index      = 1,
		.val_offset       = PWMA_VAL,
		.sub_module       = SM0,
		.sub_module_bits  = MCTRL_LDOK(1 << SM0),
	},
    {
        /* FMU_CH8 : GPIO_AD_B1_09   GPIO1_IO25  FLEXPWM4_PWMA01 */
        .gpio_out         = PIN_FLEXPWM4_PWMA01,
        .timer_index      = 1,
        .val_offset       = PWMA_VAL,
        .sub_module       = SM1,
        .sub_module_bits  = MCTRL_LDOK(1 << SM1),
    },
    {
        /* FMU_CH9 : GPIO_EMC_04   GPIO4_IO4    FLEXPWM4_PWMA02 */
        .gpio_out         = PIN_FLEXPWM4_PWMA02,
        .timer_index      = 1,
        .val_offset       = PWMA_VAL,
        .sub_module       = SM2,
        .sub_module_bits  = MCTRL_LDOK(1 << SM2),
    },
    {
        /* FMU_CH10 : GPIO_B1_15   GPIO2_IO31  FLEXPWM4_PWMA03 */
        .gpio_out         = PIN_FLEXPWM4_PWMA03,
        .timer_index      = 1,
        .val_offset       = PWMA_VAL,
        .sub_module       = SM3,
        .sub_module_bits  = MCTRL_LDOK(1 << SM3),
    },
    {
        /* FMU_CH11 : GPIO_EMC_23   GPIO4_IO23  FLEXPWM1_PWMA00 */
        .gpio_out         = PIN_FLEXPWM1_PWMA00,
        .timer_index      = 2,
        .val_offset       = PWMA_VAL,
        .sub_module       = SM0,
        .sub_module_bits  = MCTRL_LDOK(1 << SM0),
    },
    {
        /* FMU_CH12 : GPIO_EMC_24   GPIO4_IO24  FLEXPWM1_PWMB00 */
        .gpio_out         = PIN_FLEXPWM1_PWMB00,
        .timer_index      = 2,
        .val_offset       = PWMB_VAL,
        .sub_module       = SM0,
        .sub_module_bits  = MCTRL_LDOK(1 << SM0),
    },
    {
        /* FMU_CH13 : GPIO_EMC_39   GPIO3_IO25  FLEXPWM1_PWMB03 */
        .gpio_out         = PIN_FLEXPWM1_PWMB03,
        .timer_index      = 2,
        .val_offset       = PWMB_VAL,
        .sub_module       = SM3,
        .sub_module_bits  = MCTRL_LDOK(1 << SM3),
    },
    {
        /* FMU_CH14 : GPIO_EMC_33   GPIO3_IO19  FLEXPWM3_PWMA02 */
        .gpio_out         = PIN_FLEXPWM3_PWMA02,
        .timer_index      = 3,
        .val_offset       = PWMA_VAL,
        .sub_module       = SM2,
        .sub_module_bits  = MCTRL_LDOK(1 << SM2),
    },
    {
        /* FMU_CH15 : GPIO_EMC_34   GPIO3_IO20  FLEXPWM3_PWMB02 */
        .gpio_out         = PIN_FLEXPWM3_PWMB02,
        .timer_index      = 3,
        .val_offset       = PWMB_VAL,
        .sub_module       = SM2,
        .sub_module_bits  = MCTRL_LDOK(1 << SM2),
    },
};

__EXPORT const struct io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
};

__EXPORT const struct timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS] = {
};

__EXPORT void fmurt1052_timer_initialize(void)
{
	/* We must configure Qtimer 3 as the IPG divide by to yield 16 Mhz
	 * and deliver that clock to the eFlexPWM234 via XBAR
	 *
	 * IPG    = 144 Mhz
	 * 16Mhz  = 144 / 9
	 * COMP 1 = 5, COMP2 = 4
	 *
	 * */

	/* Enable Block Clocks for Qtimer and XBAR1 */

	imxrt_clockall_timer3();
	imxrt_clockall_xbar1();

	/* Disable Timer */

	rCTRL = 0;
	rCOMP1 = 5 - 1; // N - 1
	rCOMP2 = 4 - 1;

	rCAPT = 0;
	rLOAD = 0;
	rCNTR = 0;

	rSCTRL = TMR_SCTRL_OEN;

	rCMPLD1 = 0;
	rCMPLD2 = 0;
	rCSCTRL = 0;
	rFILT   = 0;
	rDMA    = 0;

	/* Count rising edges of primary source,
	 * Prescaler is /1
	 * Count UP until compare, then re-initialize. a successful compare occurs when the counter reaches a COMP1 value.
	 * Toggle OFLAG output using alternating compare registers
	 */
	rCTRL   = (TMR_CTRL_CM_MODE1 | TMR_CTRL_PCS_DIV1 | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE_TOG_ALT);

	/* QTIMER3_TIMER0  -> Flexpwm1ExtClk  */

	imxrt_xbar_connect(IMXRT_XBARA1_OUT_FLEXPWM1_EXT_CLK_SEL_OFFSET, IMXRT_XBARA1_IN_QTIMER3_TMR0_OUT);


	/* QTIMER3_TIMER0  -> Flexpwm234ExtClk  */

	imxrt_xbar_connect(IMXRT_XBARA1_OUT_FLEXPWM234_EXT_CLK_SEL_OFFSET, IMXRT_XBARA1_IN_QTIMER3_TMR0_OUT);
}
