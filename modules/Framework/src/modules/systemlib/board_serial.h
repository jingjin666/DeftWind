/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file board_serial.h
 * Read off the board serial
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author David "Buzz" Bussenschutt <davidbuzz@gmail.com>
 *
 */

#pragma once

__BEGIN_DECLS

__EXPORT int get_board_serial(uint8_t *serialid);

__END_DECLS
