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

int32_t ToMv(int32_t c) { return (c * VREF_MV + ADC_FS / 2) / ADC_FS; }

int32_t ToCounts(int32_t mv) {
  int32_t c = (mv * ADC_FS + VREF_MV / 2) / VREF_MV;
  return c < 0 ? 0 : (c > ADC_FS ? ADC_FS : c);
}

int32_t VbatMv(int32_t c, int32_t fsMv) {
  return (int32_t)(((int64_t)c * fsMv) / ADC_FS);
}

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
