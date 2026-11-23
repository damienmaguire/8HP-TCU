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

/* Stm32_TCU_V4 rev C pin map (final; Bob's firmware_answers.md and
 * pin map CSVs, Damien approved the header mapping).
 *
 * Two MAX22200 on SPI1 (RREF 15k on both -> IFS 1.0 A, 0.5 A with HFS = 1):
 *  max_cs0     CSB of IC2, PA4, software chip select, active low
 *  max_cs1     CSB of IC4, PE12
 *  max_cmd     CMD, PE8, shared: high while the command byte is clocked
 *  max_enable  ENABLE, PE7, shared: high = run, low = all outputs off
 *  max_nfault  nFAULT of IC2, PE10; max_nfault1 nFAULT of IC4, PE13. Open
 *              drain, active low, no pull-up fitted so the internal one
 *              is used. Polled every 10 ms.
 *  max_triga/b TRIGA PE9 / TRIGB PE11 of IC2: channels are SPI-controlled,
 *              so both are push-pull outputs held low (IC4's are tied to
 *              GND on the board).
 *
 * park_hold    PC7, park-hold magnet, NCV8402 low-side driver (Q3),
 *              active high. The general-purpose outputs are IC4 OUT1..3
 *              (valveout.h), not pins here.
 *
 * 12 V inputs, all the ZV1.3 circuit (SMF16A TVS, 4.7k series, 1.5k to
 * GND, 1 uF; 12 V -> 2.9 V, RC about 1.14 ms), active high (switch to
 * +12 V). The 1.5k is the pull-down, so INPUT_FLT:
 *  t15_in      PD0, J1.15, terminal 15 (ignition on)
 *  brake_in    PD1 (pin 82, EXTI1), J1.46, brake switch; debounced over
 *              3 samples (30 ms) in software
 *  shift_up    PD2, J1.50; shift_dn PD3, J1.49
 *  din1, din2, park_in, notpark_in  PD4..PD7, J1.54/53/52/51
 *
 * Speed sensors are on PA0 / PA1 / PA2 as analogue inputs (hwdefs.h), not
 * EXTI pins. The sensor supply is always on.
 *
 * lin_nslp     PD11, TJA1021 LIN transceiver SLP_N (high = awake).
 * led_out      PE2, active low.
 */
#define DIG_IO_LIST                                                            \
  DIG_IO_ENTRY(max_cs0, GPIOA, GPIO4, PinMode::OUTPUT)                         \
  DIG_IO_ENTRY(max_cs1, MAX_CS1_PORT, MAX_CS1_PIN, PinMode::OUTPUT)            \
  DIG_IO_ENTRY(max_cmd, GPIOE, GPIO8, PinMode::OUTPUT)                         \
  DIG_IO_ENTRY(max_enable, GPIOE, GPIO7, PinMode::OUTPUT)                      \
  DIG_IO_ENTRY(max_nfault, GPIOE, GPIO10, PinMode::INPUT_PU)                   \
  DIG_IO_ENTRY(max_nfault1, MAX_NFAULT1_PORT, MAX_NFAULT1_PIN, PinMode::INPUT_PU) \
  DIG_IO_ENTRY(max_triga, GPIOE, GPIO9, PinMode::OUTPUT)                       \
  DIG_IO_ENTRY(max_trigb, GPIOE, GPIO11, PinMode::OUTPUT)                      \
  DIG_IO_ENTRY(park_hold, GPIOC, GPIO7, PinMode::OUTPUT)                       \
  DIG_IO_ENTRY(t15_in, T15_PORT, T15_PIN, PinMode::INPUT_FLT)                  \
  DIG_IO_ENTRY(brake_in, BRAKE_IN_PORT, BRAKE_IN_PIN, PinMode::INPUT_FLT)      \
  DIG_IO_ENTRY(shift_up, SHIFT_UP_PORT, SHIFT_UP_PIN, PinMode::INPUT_FLT)      \
  DIG_IO_ENTRY(shift_dn, SHIFT_DN_PORT, SHIFT_DN_PIN, PinMode::INPUT_FLT)      \
  DIG_IO_ENTRY(din1, GPIOD, GPIO4, PinMode::INPUT_FLT)                         \
  DIG_IO_ENTRY(din2, GPIOD, GPIO5, PinMode::INPUT_FLT)                         \
  DIG_IO_ENTRY(park_in, GPIOD, GPIO6, PinMode::INPUT_FLT)                      \
  DIG_IO_ENTRY(notpark_in, GPIOD, GPIO7, PinMode::INPUT_FLT)                   \
  DIG_IO_ENTRY(lin_nslp, GPIOD, GPIO11, PinMode::OUTPUT)                       \
  DIG_IO_ENTRY(led_out, GPIOE, GPIO2, PinMode::OUTPUT) /* active low */        \
  DIG_IO_ENTRY(dummypin, GPIOE, GPIO15, PinMode::INPUT_PD)

#endif // PinMode_PRJ_H_INCLUDED
