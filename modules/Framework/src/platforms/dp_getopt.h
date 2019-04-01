/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_getopt.h
 * Thread safe version of getopt
 */

#pragma once

__BEGIN_DECLS

int dp_getopt(int argc, char *argv[], const char *options, int *myoptind, const char **myoptarg);

__END_DECLS

