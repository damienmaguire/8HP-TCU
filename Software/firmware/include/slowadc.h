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
#ifndef SLOWADC_H_INCLUDED
#define SLOWADC_H_INCLUDED

#include <stdint.h>

/** The slow analogue inputs: battery, temperature inputs, spare input,
 * board NTC and die temperature. ADC1 and its DMA belong to the speed
 * inputs, so these are on ADC2, which has no DMA on the F1 (libopeninv
 * AnaIn is not used). Tick10ms() reads the last results and starts the
 * next conversions (injected group of 4 + one regular conversion for the
 * board NTC); values are averaged over about 4 readings. The die sensor
 * exists on ADC1 only and is read with one injected conversion every
 * 100 ms between the speed scans. Values are raw 12 bit counts.
 */
namespace SlowAdc
{
enum Ch
{
   VBAT = 0,  // PC0, ADC12_IN10
   ATF1,      // PC1, ADC12_IN11, TempIn1
   ATF2,      // PC2, ADC12_IN12, TempIn2
   ANA,       // PC3, ADC12_IN13, spare analogue input
   MCU_TEMP,  // ADC1_IN16, die sensor
   BOARD_NTC, // PB0, ADC12_IN8, TH1
   COUNT
};
/// Pins and ADC set-up, first conversions started.
void Start();
/// 10 ms task: collect results, start the next round.
void Tick10ms();
uint16_t Get(Ch ch);
} // namespace SlowAdc

#endif // SLOWADC_H_INCLUDED
