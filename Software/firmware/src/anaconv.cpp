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
#include "anaconv.h"

namespace AnaConv {

// ADC counts to millivolts at the pin (12-bit, 3.3 V reference).
int32_t ToMv(int32_t c) { return (c * VREF_MV + ADC_FS / 2) / ADC_FS; }

// STM32 internal temperature sensor counts to degC (data sheet typical values).
int32_t McuTempC(int32_t counts) {
  // (1430 mV - V) / 4.3 mV + 25, rounded
  int32_t num = (1430 - ToMv(counts)) * 10;
  return (num >= 0 ? (num + 21) / 43 : (num - 21) / 43) + 25;
}

static const uint16_t boardNtc[] = {BOARD_NTC_TABLE};

// Board NTC counts to degC; false if the reading is open or shorted.
bool BoardNtcC(int32_t c, int32_t *degC) {
  if (c >= BOARD_NTC_OPEN || c <= BOARD_NTC_SHORT)
    return false;
  const int n = sizeof(boardNtc) / sizeof(boardNtc[0]);
  // NTC: counts fall as the temperature rises (TempMeas::Lookup, NTC).
  if (c >= boardNtc[0]) {
    *degC = BOARD_NTC_TMIN;
    return true;
  }
  for (int i = 1; i < n; i++) {
    if (c >= boardNtc[i]) {
      int32_t a = boardNtc[i - 1] - c, b = boardNtc[i - 1] - boardNtc[i];
      *degC = BOARD_NTC_TMIN + BOARD_NTC_STEP * (i - 1) +
              (BOARD_NTC_STEP * a + b / 2) / b;
      return true;
    }
  }
  *degC = BOARD_NTC_TMIN + BOARD_NTC_STEP * (n - 1);
  return true;
}

// Board temperature from the NTC if fitted and sane, else the chip's own sensor.
int BoardTempC(bool ntcFitted, int32_t ntcCounts, int32_t dieCounts, int32_t *degC) {
  if (ntcFitted && BoardNtcC(ntcCounts, degC))
    return 1;
  *degC = McuTempC(dieCounts);
  return 0;
}

// Millivolts at the pin to ADC counts (inverse of ToMv).
int32_t ToCounts(int32_t mv) {
  int32_t c = (mv * ADC_FS + VREF_MV / 2) / VREF_MV;
  return c < 0 ? 0 : (c > ADC_FS ? ADC_FS : c);
}

// Battery voltage from its divider: fsMv = volts at full ADC scale.
int32_t VbatMv(int32_t c, int32_t fsMv) {
  return (int32_t)(((int64_t)c * fsMv) / ADC_FS);
}

// Sensor resistance on a temperature input from the pull-up divider, ohm.
int32_t TempInputOhms(int32_t counts, int32_t rPullUp) {
  // Voltage at the connector, before the /2 divider, in mV.
  int32_t vin = 2 * ToMv(counts);
  if (vin >= 5000)
    return -1; // at or above the pull-up supply: open
  if (vin <= 0)
    return 0;
  // Rp = Rs || 20k = vin x Rpu / (5 V - vin)
  int64_t rp = ((int64_t)vin * rPullUp) / (5000 - vin);
  if (rp >= 20000)
    return -1; // the divider alone: sensor open
  // Rs = Rp x 20k / (20k - Rp)
  return (int32_t)((rp * 20000) / (20000 - rp));
}

// Resistance to degC through the user's R / T table (interpolated).
int32_t TempFromOhms(int32_t r, const int32_t *rp, const int32_t *tp, int n) {
  if (n <= 0)
    return 0;
  if (n == 1)
    return tp[0];
  bool desc = rp[n - 1] < rp[0];
  // Clamp at the ends.
  if (desc ? r >= rp[0] : r <= rp[0])
    return tp[0];
  if (desc ? r <= rp[n - 1] : r >= rp[n - 1])
    return tp[n - 1];
  for (int i = 1; i < n; i++) {
    bool in = desc ? r >= rp[i] : r <= rp[i];
    if (in) {
      int32_t dr = rp[i] - rp[i - 1];
      if (dr == 0)
        return tp[i];
      return tp[i - 1] +
             (int32_t)(((int64_t)(r - rp[i - 1]) * (tp[i] - tp[i - 1])) / dr);
    }
  }
  return tp[n - 1];
}

} // namespace AnaConv
