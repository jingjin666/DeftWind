/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file visibility.h
 *
 * Definitions controlling symbol naming and visibility.
 *
 * This file is normally included automatically by the build system.
 */

#ifndef __SYSTEMLIB_VISIBILITY_H
#define __SYSTEMLIB_VISIBILITY_H

#ifdef __EXPORT
#  undef __EXPORT
#endif
#define __EXPORT __attribute__ ((visibility ("default")))

#ifdef __PRIVATE
#  undef __PRIVATE
#endif
#define __PRIVATE __attribute__ ((visibility ("hidden")))

#ifdef __cplusplus
#  define __BEGIN_DECLS		extern "C" {
#  define __END_DECLS		}
#else
#  define __BEGIN_DECLS
#  define __END_DECLS
#endif
#endif
