/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file px4_macros.h
 *
 * A set of useful macros for enhanced runtime and compile time
 * error detection and warning suppression.
 *
 * Define NO_BLOAT to reduce bloat from file name inclusion.
 *
 * The arraySize() will compute the size of an array regardless
 * it's type
 *
 * INVALID_CASE(c) should be used is case statements to ferret out
 * unintended behavior
 *
 * UNUSED(var) will suppress compile time warnings of unused
 * variables
 *
 * CCASSERT(predicate) Will generate a compile time error it the
 * predicate is false
 */
#include <assert.h>

#ifndef _DP_MACROS_H
#define _DP_MACROS_H


#if !defined(arraySize)
#define arraySize(a) (sizeof((a))/sizeof((a[0])))
#endif

#if !defined(NO_BLOAT)
#if defined(__BASE_FILE__)
#define _FILE_NAME_ __BASE_FILE__
#else
#define _FILE_NAME_ __FILE__
#endif
#else
#define _FILE_NAME_ ""
#endif

#if !defined(INVALID_CASE)
#define INVALID_CASE(c) printf("Invalid Case %d, %s:%d",(c),__BASE_FILE__,__LINE__) /* todo use PANIC */
#endif

#if !defined(UNUSED)
#define UNUSED(var) (void)(var)
#endif

#if !defined(CAT)
#if !defined(_CAT)
#define _CAT(a, b) a##b
#endif
#define CAT(a, b) _CAT(a, b)
#endif

#if !defined(CAT3)
#if !defined(CAT3_)
#define CAT3_(A, B, C)    A##B##C
#endif
#define CAT3(A, B, C)     CAT3_(A, B, C)
#endif

#if !defined(FREEZE_STR)
#  define FREEZE_STR(s) #s
#endif

#if !defined(STRINGIFY)
#define STRINGIFY(s) FREEZE_STR(s)
#endif

#if !defined(CCASSERT)
#if defined(static_assert)
#		define CCASSERT(predicate) static_assert(predicate, STRINGIFY(predicate))
#	else
#		define CCASSERT(predicate) _x_CCASSERT_LINE(predicate, __LINE__)
#		if !defined(_x_CCASSERT_LINE)
#			define _x_CCASSERT_LINE(predicate, line) typedef char CAT(constraint_violated_on_line_,line)[2*((predicate)!=0)-1] __attribute__ ((unused)) ;
#		endif
#	endif
#endif

#if !defined(DO_PRAGMA)
#  define DO_PRAGMA(x) _Pragma (#x)
#endif

#if !defined(TODO)
#  define TODO(x) DO_PRAGMA(message ("TODO - " #x))
#endif

#endif /* _DP_MACROS_H */
