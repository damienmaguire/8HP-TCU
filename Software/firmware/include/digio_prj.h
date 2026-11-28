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

/* Stm32_TCU_V5 pin map (Bob's for_sam_v5_1349 digio CSV, checked against
 * the netlist). Header pins are J5 (netlist name).
 *
 * Two TLE92466ED on SPI1 (tle92466.h):
 *  tle_cs0     CSN of IC10, PA4, software chip select, active low
 *  tle_cs1     CSN of IC11, PE9
 *  tle_en      EN, PE7, shared. Low = all outputs off. Only the chips'
 *              pull-downs, so it is low until the firmware drives it.
 *  tle_resn    RESN, PE11, shared. Low = chips held in reset (outputs off).
 *  tle_nfault  FAULTN, PE10, shared open drain, 10k pull-up on the board.
 *  SAFETY: tle_en and tle_resn are only ever driven low before the drivers
 *  are started, or when the park-lock rule allows it (parksafe.h): pulling
 *  either low drops PZYL and PHOLD, which lets the park lock in.
 *
 * out1, out2   PE8 / PE12, NCV8402 low-side switches, active high
 *              (GpOut1Func / GpOut2Func). OUT3 (PC6) is the TIM3 PWM
 *              output (hwinit.cpp), so not in this list.
 *
 * 12 V inputs, all the ZV1.3 circuit (SMF16A TVS, 4.7k series, 1.5k to
 * GND, 1 uF; 12 V -> 2.9 V, RC about 1.14 ms), active high (switch to
 * +12 V). The 1.5k is the pull-down, so INPUT_FLT:
 *  t15_in      PD0, terminal 15 (ignition on)
 *  brake_in    PD1 (EXTI1), brake switch; debounced over 3 samples
 *              (30 ms) in software
 *  shift_up    PD2; shift_dn PD3
 *  din1, din2, park_in, notpark_in  PD4..PD7
 *  clutch_in   PA15, J5.41 CL_IN (needs JTAG off, afio_setup). Read into
 *              dinBits only; nothing acts on it.
 *
 * Speed sensors are on PA0 / PA1 / PA2 as analogue inputs (hwdefs.h), not
 * EXTI pins. The sensor supply is always on.
 *
 * lin_nslp     PD11, TJA1021 LIN transceiver SLP_N (high = awake; the chip
 *              pulls it low inside, so it sleeps until we drive it high).
 * led_out      PE2, active low.
 */
#define DIG_IO_LIST                                                            \
  DIG_IO_ENTRY(tle_cs0, GPIOA, GPIO4, PinMode::OUTPUT)                         \
  DIG_IO_ENTRY(tle_cs1, TLE_CS1_PORT, TLE_CS1_PIN, PinMode::OUTPUT)            \
  DIG_IO_ENTRY(tle_en, GPIOE, GPIO7, PinMode::OUTPUT)                          \
  DIG_IO_ENTRY(tle_resn, GPIOE, GPIO11, PinMode::OUTPUT)                       \
  DIG_IO_ENTRY(tle_nfault, GPIOE, GPIO10, PinMode::INPUT_FLT)                  \
  DIG_IO_ENTRY(out1, GPIOE, GPIO8, PinMode::OUTPUT)                            \
  DIG_IO_ENTRY(out2, GPIOE, GPIO12, PinMode::OUTPUT)                           \
  DIG_IO_ENTRY(t15_in, T15_PORT, T15_PIN, PinMode::INPUT_FLT)                  \
  DIG_IO_ENTRY(brake_in, BRAKE_IN_PORT, BRAKE_IN_PIN, PinMode::INPUT_FLT)      \
  DIG_IO_ENTRY(shift_up, SHIFT_UP_PORT, SHIFT_UP_PIN, PinMode::INPUT_FLT)      \
  DIG_IO_ENTRY(shift_dn, SHIFT_DN_PORT, SHIFT_DN_PIN, PinMode::INPUT_FLT)      \
  DIG_IO_ENTRY(din1, GPIOD, GPIO4, PinMode::INPUT_FLT)                         \
  DIG_IO_ENTRY(din2, GPIOD, GPIO5, PinMode::INPUT_FLT)                         \
  DIG_IO_ENTRY(park_in, GPIOD, GPIO6, PinMode::INPUT_FLT)                      \
  DIG_IO_ENTRY(notpark_in, GPIOD, GPIO7, PinMode::INPUT_FLT)                   \
  DIG_IO_ENTRY(clutch_in, GPIOA, GPIO15, PinMode::INPUT_FLT)                   \
  DIG_IO_ENTRY(lin_nslp, GPIOD, GPIO11, PinMode::OUTPUT)                       \
  DIG_IO_ENTRY(led_out, GPIOE, GPIO2, PinMode::OUTPUT) /* active low */        \
  DIG_IO_ENTRY(dummypin, GPIOE, GPIO15, PinMode::INPUT_PD)

#endif // PinMode_PRJ_H_INCLUDED
