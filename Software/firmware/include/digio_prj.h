/*
 * This file is part of the 8HP-TCU project.
 *
 * Copyright (C) 2026 Damien Maguire <info@evbmw.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

/* *** EVERY PIN BELOW IS A PLACEHOLDER (TBD) ***
 * Bob is still laying out the board. These pins only make the firmware
 * build and must be replaced with the real pinout before the board is
 * powered. Nothing here is taken from the V3 schematic.
 *
 * MAX22200 pins (data sheet "Pin Description"):
 *  max_cs0/1   CSB of chip 0 and 1, active low
 *  max_cmd     CMD: high while the command byte is clocked
 *  max_enable  ENABLE: low = all outputs off and the chip in sleep
 *  max_nfault  nFAULT: open drain, low on a fault (needs a pull-up)
 * TRIGA/TRIGB are not used (channels are SPI-controlled) and should be
 * tied low on the board.
 *
 * Speed sensors: any pin with its own EXTI line works. The ISR in
 * stm32_tcu.cpp uses lines 7, 8 and 9 for the placeholders below; change
 * it together with these pins.
 */
#define DIG_IO_LIST                                                            \
  DIG_IO_ENTRY(max_cs0, GPIOA, GPIO4, PinMode::OUTPUT)          /* TBD */      \
  DIG_IO_ENTRY(max_cs1, GPIOE, GPIO10, PinMode::OUTPUT)         /* TBD */      \
  DIG_IO_ENTRY(max_cmd, GPIOE, GPIO11, PinMode::OUTPUT)         /* TBD */      \
  DIG_IO_ENTRY(max_enable, GPIOE, GPIO12, PinMode::OUTPUT)      /* TBD */      \
  DIG_IO_ENTRY(max_nfault, GPIOE, GPIO13, PinMode::INPUT_PU)    /* TBD */      \
  DIG_IO_ENTRY(spd_in, GPIOE, GPIO7, PinMode::INPUT_FLT)        /* TBD */      \
  DIG_IO_ENTRY(spd_out, GPIOE, GPIO8, PinMode::INPUT_FLT)       /* TBD */      \
  DIG_IO_ENTRY(spd_aux, GPIOE, GPIO9, PinMode::INPUT_FLT)       /* TBD */      \
  DIG_IO_ENTRY(sens_supply, GPIOE, GPIO14, PinMode::OUTPUT)     /* TBD */      \
  DIG_IO_ENTRY(lin_nslp, GPIOD, GPIO11, PinMode::OUTPUT)        /* TBD */      \
  DIG_IO_ENTRY(led_out, GPIOE, GPIO2, PinMode::OUTPUT)          /* TBD */      \
  DIG_IO_ENTRY(dummypin, GPIOE, GPIO15, PinMode::INPUT_PD)      /* TBD */

#endif // PinMode_PRJ_H_INCLUDED
