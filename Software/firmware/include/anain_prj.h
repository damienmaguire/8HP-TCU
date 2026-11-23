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

#define NUM_SAMPLES 12 // ADC samples kept per input
/* 239.5 cycles: the internal temperature sensor needs >= 17.1 us of
 * sampling (12 MHz ADC clock: 20 us). The other inputs are resistive
 * dividers and read better with the longer time too. Scan of 5 channels
 * x 12 samples = 1.3 ms. */
#define SAMPLE_TIME ADC_SMPR_SMP_239DOT5CYC

/* Stm32_TCU_V4 analogue inputs on ADC1 (libopeninv AnaIn, DMA scan).
 * V4 only: on rev B ADC1 and DMA1 channel 1 run the speed scan, and these
 * inputs are read through ADC2 by slowadc.cpp (same pins, same order).
 * The firmware reads them through SlowAdc on both boards.
 * VDDA / VREF+ = +3V3 through L2, so 4095 counts = 3.3 V (not a
 * precision reference). Conversions in anaconv.h.
 *  vbat       PC0, ADC12_IN10: 8.2k / 1.8k = 0.18, V = Vadc x 5.556,
 *             full scale 18.33 V (VbatScale)
 *  atf_temp1  PC1, ADC12_IN11: TempIn1 J1.31, 2.6k pull-up to 5 V, /2
 *             (rev B: 2.7k)
 *  atf_temp2  PC2, ADC12_IN12: TempIn2 J1.33, 470R pull-up to 5 V, /2
 *             (rev B: 2.7k). Which input the 8HP ATF sensor uses is open
 *             (Damien).
 *  ana_in     PC3, ADC12_IN13: J1.47 spare, /2, 0..6.6 V
 *  mcu_temp   ADC1_IN16, the STM32 internal temperature sensor (tBoard).
 *             libopeninv maps any non-ADC pin to IN16 and switches the
 *             sensor on. PE15 is named here because it is the unused
 *             dummypin; it is set to analogue input and still reads 0.
 *  board_ntc  rev B only: TH1 on PB0 (ADC12_IN8), 10k B3950, 10k pull-up
 *             to 3V3; AnaConv::BoardNtcC. Read by slowadc.cpp.
 *
 * The speed sensor pins spd_in_ana (PA0, ADC12_IN0) and spd_out_ana
 * (PA1, ADC12_IN1), and on rev B spd_aux_ana (n6, PA2, ADC12_IN2), are
 * sampled by the speed ADC at SPEED_TIMER_HZ (V4: ADC2 injected, rev B:
 * ADC1 continuous scan + DMA), never by the slow inputs' ADC, so the same
 * channel is never converted by both ADCs at once. Rev B fits a 10k / 1 nF
 * RC on each speed pin.
 */
#define ANA_IN_LIST                                                            \
  ANA_IN_ENTRY(vbat, GPIOC, 0)                                                 \
  ANA_IN_ENTRY(atf_temp1, GPIOC, 1)                                            \
  ANA_IN_ENTRY(atf_temp2, GPIOC, 2)                                            \
  ANA_IN_ENTRY(ana_in, GPIOC, 3)                                               \
  ANA_IN_ENTRY(mcu_temp, GPIOE, 15)

/* Speed sensor pins (hwinit.cpp speed_adc_setup). n6 on rev B only: an
 * internal shaft, a safety cross-check only. */
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
