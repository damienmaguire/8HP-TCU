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
#ifndef ANACONV_H
#define ANACONV_H

#include <stdint.h>

/* Analogue front ends on the V4 board (Bob's firmware_answers.md, 5),
 * as plain integer maths so it runs in the host tests. The ADC reference
 * is the 3.3 V supply (VDDA through L2), not a precision reference.
 */

/* Speed sensors on PA0 (ISS) / PA1 (OSS), rev B also PA2 (n6): two-wire
 * current sensors into 100 ohm, 7 mA / 14 mA = about 0.7 V / 1.4 V. That
 * is below the digital VIH, so the pins are sampled by the ADC (Damien's
 * choice; V4 ADC2, rev B ADC1 + DMA) and an edge is a crossing of the high
 * threshold after the signal has been below the low one (hysteresis).
 * Thresholds are parameters until the rev B RC filter (10k / 1 nF) is
 * fitted and measured (Bob: about 1.05 V).
 */
class AdcEdge
{
public:
   AdcEdge() : lo(0), hi(4095), high(false) {}
   void Configure(uint16_t loCounts, uint16_t hiCounts) {
      lo = loCounts;
      hi = hiCounts;
   }
   /// Feed one sample; true on a rising edge.
   bool Sample(uint16_t v) {
      if (!high && v >= hi) {
         high = true;
         return true;
      }
      if (high && v <= lo)
         high = false;
      return false;
   }
   bool High() const { return high; }
   uint16_t Lo() const { return lo; }
   uint16_t Hi() const { return hi; }
   /// For block processing (ScanEdges): carry the state across blocks.
   void SetHigh(bool h) { high = h; }

private:
   uint16_t lo, hi;
   bool high;
};

namespace AnaConv
{
const int32_t ADC_FS = 4095;
const int32_t VREF_MV = 3300;
/// Pull-ups to 5 V on the temperature inputs (R22, R23), ohm. V4: 2.6k
/// and 470R; rev B: 2.7k on both (Bob, rev B BOM). HW_REV comes from the
/// Makefile (hwdefs.h).
#if defined(HW_REV) && HW_REV >= 5
const int32_t TEMPIN1_PULLUP = 2700;
const int32_t TEMPIN2_PULLUP = 2700;
#else
const int32_t TEMPIN1_PULLUP = 2600;
const int32_t TEMPIN2_PULLUP = 470;
#endif

/// ADC counts -> mV at the pin.
int32_t ToMv(int32_t counts);
/// mV at the pin -> ADC counts (for thresholds set in mV).
int32_t ToCounts(int32_t mv);
/// Battery: divider 1.8k / (8.2k + 1.8k) = 0.18, so V = Vadc x 5.556 and
/// 3.3 V at the pin = 18.33 V. fsMv is the VbatScale parameter.
int32_t VbatMv(int32_t counts, int32_t fsMv);

/** Sensor resistance on a temperature input, ohm. The input has a pull-up
 * rPullUp to 5 V at the connector (V4: 2.6k on TempIn1, 470R on TempIn2;
 * rev B: 2.7k on both), then
 * a 10k/10k divider to the ADC, so the sensor sees Rs || 20k and
 *   Vadc = 1/2 x 5 V x (Rs || 20k) / (rPullUp + Rs || 20k).
 * Returns -1 if the reading means open circuit (Rs not finite).
 */
int32_t TempInputOhms(int32_t counts, int32_t rPullUp);

/** Temperature from a resistance table (any monotonic order, n points).
 * Clamped at the ends. */
int32_t TempFromOhms(int32_t ohms, const int32_t *rPts, const int32_t *tPts, int n);

/** STM32F107 internal temperature sensor (ADC1_IN16), degC. Data sheet
 * typical values: V25 = 1.43 V, slope 4.3 mV/degC, T = (V25 - V) / slope
 * + 25. Not calibrated per part: V25 spreads 1.34..1.52 V, so the reading
 * can be some 20 degC off. A die temperature, used as the board
 * temperature (V4 has no board sensor). */
int32_t McuTempC(int32_t counts);

/* Board NTC TH1 (rev B, HW_REV 5), in the libopeninv / ZombieVerter
 * TempMeas style: a table of ADC counts at fixed temperature steps,
 * interpolated. Rev B circuit (Bob): 10k NTC at 25 degC, B 3950 K, to GND,
 * 10k pull-up to 3V3, so counts = 4095 x R / (R + 10k) with
 * R = 10k x exp(3950 x (1/T - 1/298.15)), -40..125 degC in 5 degC steps.
 */
#define BOARD_NTC_TMIN -40 // degC, first table entry
#define BOARD_NTC_STEP 5 // degC between table entries
#define BOARD_NTC_TABLE                                                        \
  3996, 3955, 3900, 3830, 3740, 3629, 3495, 3337, 3156, 2955, 2738, 2510,      \
      2278, 2048, 1825, 1614, 1419, 1241, 1081, 940, 815, 707, 613, 532, 462,  \
      401, 350, 305, 267, 234, 206, 181, 160, 142
/// Counts outside this window mean open (pulled up) or shorted: not valid.
#define BOARD_NTC_OPEN 4050 // ADC counts (12 bit)
#define BOARD_NTC_SHORT 40 // ADC counts (12 bit)
/** Board NTC temperature, degC, clamped to the table ends. Returns false
 * (and leaves *degC alone) for an open or shorted sensor. */
bool BoardNtcC(int32_t counts, int32_t *degC);
/** tBoard source: the NTC if fitted and valid, else the die sensor.
 * Returns 1 when the NTC was used, 0 for the die. */
int BoardTempC(bool ntcFitted, int32_t ntcCounts, int32_t dieCounts, int32_t *degC);
} // namespace AnaConv

#endif // ANACONV_H
