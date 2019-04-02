/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_posix.h
 *
 * Includes POSIX-like functions for virtual character devices
 */

#pragma once

#include <dp_defines.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <semaphore.h>
#include <stdint.h>

#include <sys/types.h>

/* Semaphore handling */

__BEGIN_DECLS

typedef sem_t dp_sem_t;

#define dp_sem_init	 sem_init
#define dp_sem_wait	 sem_wait
#define dp_sem_post	 sem_post
#define dp_sem_getvalue sem_getvalue
#define dp_sem_destroy	 sem_destroy
#define dp_sem_timedwait	sem_timedwait

__END_DECLS

//###################################

#ifdef __DP_NUTTX

#define  DP_F_RDONLY 1
#define  DP_F_WRONLY 2

typedef struct pollfd dp_pollfd_struct_t;

#if defined(__cplusplus)
#define _GLOBAL ::
#else
#define _GLOBAL
#endif
#define dp_open 	_GLOBAL open
#define dp_close 	_GLOBAL close
#define dp_ioctl 	_GLOBAL ioctl
#define dp_write 	_GLOBAL write
#define dp_read 	_GLOBAL read
#define dp_poll 	_GLOBAL poll
#define dp_fsync 	_GLOBAL fsync
#define dp_access 	_GLOBAL access
#define dp_getpid 	_GLOBAL getpid

#elif defined(__DP_POSIX)

#define  DP_F_RDONLY O_RDONLY
#define  DP_F_WRONLY O_WRONLY
#define  DP_F_CREAT  O_CREAT

typedef short pollevent_t;

typedef struct {
	/* This part of the struct is POSIX-like */
	int		fd;       /* The descriptor being polled */
	pollevent_t 	events;   /* The input event flags */
	pollevent_t 	revents;  /* The output event flags */

	/* Required for DP compatibility */
	dp_sem_t   *sem;  	/* Pointer to semaphore used to post output event */
	void   *priv;     	/* For use by drivers */
} dp_pollfd_struct_t;

__BEGIN_DECLS

__EXPORT int 		dp_open(const char *path, int flags, ...);
__EXPORT int 		dp_close(int fd);
__EXPORT ssize_t	dp_read(int fd, void *buffer, size_t buflen);
__EXPORT ssize_t	dp_write(int fd, const void *buffer, size_t buflen);
__EXPORT int		dp_ioctl(int fd, int cmd, unsigned long arg);
__EXPORT int		dp_poll(dp_pollfd_struct_t *fds, nfds_t nfds, int timeout);
__EXPORT int		dp_fsync(int fd);
__EXPORT int		dp_access(const char *pathname, int mode);
__EXPORT unsigned long	dp_getpid(void);

__EXPORT void		dp_enable_sim_lockstep(void);
__EXPORT void		dp_sim_start_delay(void);
__EXPORT void		dp_sim_stop_delay(void);
__EXPORT bool		dp_sim_delay_enabled(void);

__END_DECLS
#else
#error "No TARGET OS Provided"
#endif

__BEGIN_DECLS
extern int dp_errno;

__EXPORT void		dp_show_devices(void);
__EXPORT void		dp_show_files(void);
__EXPORT const char 	*dp_get_device_names(unsigned int *handle);

__EXPORT void		dp_show_topics(void);
__EXPORT const char 	*dp_get_topic_names(unsigned int *handle);

#ifndef __DP_QURT
/*
 * The UNIX epoch system time following the system clock
 */
__EXPORT uint64_t	hrt_system_time(void);

__EXPORT bool		dp_exit_requested(void);
#endif

__END_DECLS
