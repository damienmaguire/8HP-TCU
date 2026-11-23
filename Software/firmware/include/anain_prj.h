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
#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

#define NUM_SAMPLES 12
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC

/* Stm32_TCU_V4 analogue inputs on ADC1 (libopeninv AnaIn, DMA scan).
 * VDDA / VREF+ = +3V3 through L2, so 4095 counts = 3.3 V (not a
 * precision reference). Conversions in anaconv.h.
 *  vbat       PC0, ADC12_IN10: 8.2k / 1.8k = 0.18, V = Vadc x 5.556,
 *             full scale 18.33 V (VbatScale)
 *  atf_temp1  PC1, ADC12_IN11: TempIn1 J1.31, 2.6k pull-up to 5 V, /2
 *  atf_temp2  PC2, ADC12_IN12: TempIn2 J1.33, 470R pull-up to 5 V, /2
 *             Which input the 8HP ATF sensor uses is open (Damien).
 *  ana_in     PC3, ADC12_IN13: J1.47 spare, /2, 0..6.6 V
 *
 * The speed sensor pins spd_in_ana (PA0, ADC12_IN0) and spd_out_ana
 * (PA1, ADC12_IN1) are sampled by ADC2 at SPEED_TIMER_HZ, not by this
 * slow scan, so the same channel is never converted by both ADCs at
 * once. REV: the RC filter on these pins comes with the next revision.
 */
#define ANA_IN_LIST                                                            \
  ANA_IN_ENTRY(vbat, GPIOC, 0)                                                 \
  ANA_IN_ENTRY(atf_temp1, GPIOC, 1)                                            \
  ANA_IN_ENTRY(atf_temp2, GPIOC, 2)                                            \
  ANA_IN_ENTRY(ana_in, GPIOC, 3)

/* ADC2 injected inputs for the speed sensors (hwinit.cpp speed_adc_setup).
 * Next revision: the third input (n6, an internal shaft, safety cross-check
 * only) probably on PA2 = ADC12_IN2 (Bob, not final). */
#if HW_HAS_SPD3
#define SPEED_ANA_LIST                                                         \
  SPEED_ANA_ENTRY(spd_in_ana, GPIOA, 0)                                        \
  SPEED_ANA_ENTRY(spd_out_ana, GPIOA, 1)                                       \
  SPEED_ANA_ENTRY(spd_aux_ana, GPIOA, 2)
#else
#define SPEED_ANA_LIST                                                         \
  SPEED_ANA_ENTRY(spd_in_ana, GPIOA, 0)                                        \
  SPEED_ANA_ENTRY(spd_out_ana, GPIOA, 1)
#endif

#endif // ANAIN_PRJ_H_INCLUDED
