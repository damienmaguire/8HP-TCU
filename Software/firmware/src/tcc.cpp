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
#include "tcc.h"
#include "caltypes.h"

/* PLACEHOLDER lock-up-on schedule: input rpm vs pedal (x) and gear (y).
 * Not measured and not OEM; tune on the car. */
static const int16_t tccPedal[5] = {0, 25, 50, 75, 100};
static const int16_t tccGear[8] = {1, 2, 3, 4, 5, 6, 7, 8};
static const int32_t tccOn[40] = {
    3000, 3200, 3500, 4000, 4500, // 1st: lock late, if at all
    2200, 2400, 2800, 3300, 3800, // 2nd
    1600, 1800, 2200, 2700, 3200, // 3rd
    1400, 1600, 1900, 2400, 2900, // 4th
    1300, 1450, 1700, 2200, 2700, // 5th
    1200, 1350, 1600, 2000, 2500, // 6th
    1150, 1300, 1500, 1900, 2400, // 7th
    1100, 1250, 1450, 1800, 2300, // 8th
};
static const CalMap tccOnMap = {5, 8, tccPedal, tccGear, tccOn};

int32_t Tcc::OnRpm(int gear, int32_t pedalPct) {
  return Cal::Map(tccOnMap, pedalPct, gear);
}

int32_t Tcc::Target(const TccConfig &c, int gear, int32_t nT) const {
  if (c.slipTgt > 0 && gear <= c.slipGear && nT < c.slipRpm)
    return c.slipTgt;
  return 0;
}

int32_t Tcc::Tick10ms(const TccConfig &c, bool allowed, int gear, int32_t pedal,
                      int32_t nT, int32_t nMot) {
  int32_t on = OnRpm(gear, pedal);
  if (!allowed || nT < c.minRpm)
    closed = false;
  else if (!closed && nT >= on)
    closed = true;
  else if (closed && nT < on - c.hystRpm)
    closed = false;
  if (!closed) {
    integ = 0;
    p = 0;
    return 0;
  }
  // PI on slip: positive error (too much slip) = more pressure.
  int32_t err = (nMot - nT) - Target(c, gear, nT);
  integ += c.ki10 * err; // 0.1 mbar/(rpm s) x rpm x 10 ms = 0.001 mbar
  int32_t iMax = c.maxP * 1000;
  if (integ > iMax) integ = iMax;
  if (integ < -iMax) integ = -iMax;
  int32_t out = c.startP + c.kp10 * err / 10 + integ / 1000;
  if (out < 0) out = 0;
  if (out > c.maxP) out = c.maxP;
  p = out;
  return p;
}
