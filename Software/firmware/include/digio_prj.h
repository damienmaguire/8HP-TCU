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

/* Stm32_TCU_V4 pin map (Bob's firmware_answers.md, Damien approved the
 * header mapping). Revision-pending items are marked REV.
 *
 * MAX22200 (one chip, RREF 15k -> IFS 1.0 A, 0.5 A with HFS = 1):
 *  max_cs0     CSB, PA4, software chip select, active low
 *  max_cmd     CMD, PE8: high while the command byte is clocked
 *  max_enable  ENABLE, PE7: high = run, low = all outputs off and sleep
 *  max_nfault  nFAULT, PE10: open drain, active low, no pull-up fitted so
 *              the internal one is used. Polled every 10 ms (EXTI10 free).
 *  max_triga/b TRIGA PE9 / TRIGB PE11: channels are SPI-controlled, so
 *              both are push-pull outputs held low.
 *
 * Discrete low-side drivers (NCV8402, active high):
 *  park_hold   PC7, park-hold magnet (moved off the MAX22200 on V4; stays
 *              on PC7 / Q3 on rev B)
 *  gp_out1/2   PC8 / PC9, spare, either can be a starter lock. V4 only:
 *              rev B removes Q1 / Q2 and drives these functions from IC4
 *              OUT1 / OUT2; PC8 / PC9 are free.
 * gp_pwm (PC6, TIM3_CH1, inverted FAN3121) is set up in hwinit.cpp. V4
 * only: on rev B the GP PWM function is IC4 OUT3 and PC6 is free.
 *
 * Digital inputs D1, D2, PARK, NOT_PARK (J1.54/53/52/51): active high
 * (switch to +12 V), 4.7k / 1.5k divider, the 1.5k is the pull-down, so
 * INPUT_FLT. RC about 1.1 ms.
 *
 * Speed sensors are on PA0 / PA1 as analogue inputs (anain_prj.h), not
 * EXTI pins. The sensor supply is always on (no switch on V4).
 *
 * lin_nslp  PD11, LIN transceiver SLP_N (high = awake). No LIN
 *           transceiver on V4 (UART4 PC10/PC11 kept free); rev B: TJA1021.
 * led_out   PE2, active low.
 *
 * Rev B (HW_REV 5, hwdefs.h):
 *  max_cs1     CSB of the second MAX22200 (IC4), PE12; same SPI1, CMD and
 *              ENABLE as IC2
 *  max_nfault1 nFAULT of IC4, PE13, open drain, internal pull-up
 *  shift_up    PD2 / shift_dn PD3: 12 V shift up / down inputs, active
 *              high (divider with pull-down: INPUT_FLT)
 *  t15_in      PD0, terminal 15 (ignition on), active high
 *  lin_nslp    PD11 drives the TJA1021 SLP_N (fitted on rev B)
 */
#if HW_REV >= 5
#define DIG_IO_GP
#define DIG_IO_NEXT                                                            \
  DIG_IO_ENTRY(max_cs1, MAX_CS1_PORT, MAX_CS1_PIN, PinMode::OUTPUT)            \
  DIG_IO_ENTRY(max_nfault1, MAX_NFAULT1_PORT, MAX_NFAULT1_PIN, PinMode::INPUT_PU) \
  DIG_IO_ENTRY(shift_up, SHIFT_UP_PORT, SHIFT_UP_PIN, PinMode::INPUT_FLT)      \
  DIG_IO_ENTRY(shift_dn, SHIFT_DN_PORT, SHIFT_DN_PIN, PinMode::INPUT_FLT)      \
  DIG_IO_ENTRY(t15_in, T15_PORT, T15_PIN, PinMode::INPUT_FLT)
#else
#define DIG_IO_GP                                                              \
  DIG_IO_ENTRY(gp_out1, GPIOC, GPIO8, PinMode::OUTPUT)                         \
  DIG_IO_ENTRY(gp_out2, GPIOC, GPIO9, PinMode::OUTPUT)
#define DIG_IO_NEXT
#endif
#define DIG_IO_LIST                                                            \
  DIG_IO_ENTRY(max_cs0, GPIOA, GPIO4, PinMode::OUTPUT)                         \
  DIG_IO_ENTRY(max_cmd, GPIOE, GPIO8, PinMode::OUTPUT)                         \
  DIG_IO_ENTRY(max_enable, GPIOE, GPIO7, PinMode::OUTPUT)                      \
  DIG_IO_ENTRY(max_nfault, GPIOE, GPIO10, PinMode::INPUT_PU)                   \
  DIG_IO_ENTRY(max_triga, GPIOE, GPIO9, PinMode::OUTPUT)                       \
  DIG_IO_ENTRY(max_trigb, GPIOE, GPIO11, PinMode::OUTPUT)                      \
  DIG_IO_ENTRY(park_hold, GPIOC, GPIO7, PinMode::OUTPUT)                       \
  DIG_IO_GP                                                                    \
  DIG_IO_ENTRY(din1, GPIOD, GPIO4, PinMode::INPUT_FLT)                         \
  DIG_IO_ENTRY(din2, GPIOD, GPIO5, PinMode::INPUT_FLT)                         \
  DIG_IO_ENTRY(park_in, GPIOD, GPIO6, PinMode::INPUT_FLT)                      \
  DIG_IO_ENTRY(notpark_in, GPIOD, GPIO7, PinMode::INPUT_FLT)                   \
  DIG_IO_ENTRY(lin_nslp, GPIOD, GPIO11, PinMode::OUTPUT)                       \
  DIG_IO_ENTRY(led_out, GPIOE, GPIO2, PinMode::OUTPUT)   /* active low */      \
  DIG_IO_NEXT                                                                  \
  DIG_IO_ENTRY(dummypin, GPIOE, GPIO15, PinMode::INPUT_PD)

#endif // PinMode_PRJ_H_INCLUDED
