/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file test_servo.c
 * Tests main file, loads individual tests.
 *
 * @author User <mail@example.com>
 */

/**
 * @file test_servo.c
 * Tests the servo outputs
 *
 */

#include <dp_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/err.h>

#include "tests.h"

/*
 * PWM_OUTPUT0_DEVICE_PATH 在fmu init中注册RCOutput模块也调用的/dev/pwm_output0而不是/dev/fmu
 *
 */
int test_servo(int argc, char *argv[])
{
	int fd, result;
	servo_position_t servos[PWM_OUTPUT_MAX_CHANNELS];
	servo_position_t value;
	unsigned pwm_value = 1000;

	fd = open(PWM_OUTPUT0_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		printf("failed opening %s\n", PWM_OUTPUT0_DEVICE_PATH);
		goto out;
	}
    
	unsigned servo_count;
	result = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
    
	if (result != OK) {
		warnx("PWM_SERVO_GET_COUNT");
		return ERROR;
	}

    printf("servo_count is %d\n", servo_count);

    // init servos pwm values
    for (unsigned i = 0; i < servo_count; i++) {
        servos[i] = pwm_value;
    }

    // use ioctl interface for one direction
	for (unsigned i = 0; i < servo_count; i++) {
		if (ioctl(fd, PWM_SERVO_SET(i), servos[i]) < 0) {
			printf("servo %u set failed\n", i);
		}
	}
    
    /* readback servo values */
	for (unsigned i = 0; i < servo_count; i++) {
		result = ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&value);
		if (result < 0) {
			printf("failed reading channel %u\n", i);
			goto out;
		}

        if(value != servos[i]) {
            printf("servo %d readback error, got %u expected %u\n", i, value, servos[i]);
        }
	}

	/* tell safety that its ok to disable it with the switch */
	result = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);

	if (result != OK) {
		warnx("FAIL: PWM_SERVO_SET_ARM_OK\n");
	}

	/* tell output device that the system is armed (it will output values if safety is off) */
	result = ioctl(fd, PWM_SERVO_ARM, 0);

	if (result != OK) {
		warnx("FAIL: PWM_SERVO_ARM\n");
	}
    
out:
	return 0;
}
