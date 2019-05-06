/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_nuttx_tasks.c
 * Implementation of existing task API for NuttX
 */

#include <dp_config.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <unistd.h>
#include <float.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>

#ifdef CONFIG_ARCH_BOARD_UAVRS_V1
#include <stm32_pwr.h>
#elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
#include <up_arch.h>
#include <chip/imxrt_snvs.h>
#endif

#include <systemlib/systemlib.h>
#include <dp_task.h>

// Didn't seem right to include up_internal.h, so direct extern instead.
extern void up_systemreset(void) noreturn_function;

void dp_systemreset(bool to_bootloader)
{
	if (to_bootloader) {
#ifdef CONFIG_ARCH_BOARD_UAVRS_V1
		stm32_pwr_enablebkp(true);
        /* XXX wow, this is evil - write a magic number into backup register zero */
        *(uint32_t *)0x40002850 = 0xb007b007;
#elif defined(CONFIG_ARCH_BOARD_UAVRS_V2)
        putreg32(0xb007b007, IMXRT_SNVS_LPGPR(3));
#endif
	}

	up_systemreset();

	/* lock up here */
	while (true);
}

int dp_task_create(const char *name, int scheduler, int priority, int stack_size, main_t entry, char *const argv[])
{
	int pid;

	sched_lock();

	/* create the task */
	pid = task_create(name, priority, stack_size, entry, argv);

	if (pid > 0) {

		/* configure the scheduler */
		struct sched_param param;

		param.sched_priority = priority;
		sched_setscheduler(pid, scheduler, &param);

		/* XXX do any other private task accounting here before the task starts */
	}

	sched_unlock();

	return pid;
}

int dp_task_delete(int pid)
{
	return task_delete(pid);
}
