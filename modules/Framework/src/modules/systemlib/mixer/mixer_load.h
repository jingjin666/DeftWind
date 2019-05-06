/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file mixer_load.h
 *
 */


#ifndef _SYSTEMLIB_MIXER_LOAD_H
#define _SYSTEMLIB_MIXER_LOAD_H value

#include <dp_config.h>

__BEGIN_DECLS

__EXPORT int load_mixer_file(const char *fname, char *buf, unsigned maxlen);

__END_DECLS

#endif
