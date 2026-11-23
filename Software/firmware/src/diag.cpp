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
#include "diag.h"
#include "valveout.h"

namespace Diag {

// Coil + wiring resistance expected at this temperature.
int32_t ExpectedMilliOhm(int32_t tempC, int32_t wire) {
  // R(T) = R20 x (1 + tc x (T - 20)) + wiring
  int64_t r = (int64_t)DIAG_R20_MOHM * (1000000 + (int64_t)DIAG_TC_PPM * (tempC - 20)) / 1000000;
  return (int32_t)r + wire;
}

// Compare a measured V / I resistance with the expected band.
uint8_t Resistance(int32_t mV, int32_t mA, int32_t tempC, int32_t bandPct, int32_t wire) {
  if (mA < 50)
    return DIAG_OK; // too little current to judge (coil off or just switched)
  int32_t r = (int32_t)((int64_t)mV * 1000 / mA); // milliohm
  if (r < DIAG_RSHORT_MOHM)
    return DIAG_SHORT_GND;
  int32_t e = ExpectedMilliOhm(tempC, wire);
  int32_t band = e * bandPct / 100;
  if (r > e + band)
    return DIAG_R_HIGH;
  if (r < e - band)
    return DIAG_R_LOW;
  return DIAG_OK;
}

// Fault code of one valve channel: driver flags first, then the resistance check.
uint8_t Channel(uint16_t f, uint8_t rCheck) {
  if (f & VALVE_F_OCP)
    return DIAG_SHORT_BAT;
  if (f & VALVE_F_OLF)
    return DIAG_OPEN;
  if (f & VALVE_F_HHF)
    return DIAG_R_HIGH;
  return rCheck;
}

// Pairs of chip channels that are both faulted (shared wiring suspected).
uint8_t Pairs(const uint8_t *codes, int n) {
  uint8_t m = 0;
  for (int k = 0; 2 * k + 1 < n && k < 8; k++)
    if (codes[2 * k] != DIAG_OK && codes[2 * k + 1] != DIAG_OK)
      m |= (uint8_t)(1u << k);
  return m;
}

// Speed sensor state from its signal levels and edges.
uint8_t Speed(int32_t minMv, int32_t maxMv, bool edges, bool otherMoving,
              int32_t openMv, int32_t shortMv) {
  bool flat = !edges;
  if (flat && openMv > 0 && minMv >= openMv)
    return SPD_OPEN;
  if (flat && shortMv > 0 && maxMv <= shortMv)
    return SPD_SHORT;
  if (flat && otherMoving)
    return SPD_NOSIGNAL;
  return SPD_OK;
}

// Calibration signature check (0 expected = not checked).
bool CalOk(uint32_t expected, uint32_t actual) { return expected == 0 || expected == actual; }

} // namespace Diag

// Watch engine vs turbine speed while the lock-up clutch should be closed.
bool SlipMonitor::Tick10ms(bool locked, int32_t nMot, int32_t nT, int32_t maxRpm, int32_t ms) {
  if (maxRpm <= 0 || !locked) {
    t = 0;
    return fault;
  }
  int32_t d = nMot - nT;
  if (d < 0)
    d = -d;
  t = d > maxRpm ? t + 10 : 0;
  if (t > ms)
    fault = true; // latched until Clear() (fault clear)
  return fault;
}
