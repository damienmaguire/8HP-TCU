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
#include "torquein.h"
#include "caltypes.h"

/* PLACEHOLDER torque shape (not measured, not from any OEM): Nm for a
 * 300 Nm peak engine, x = rpm, y = MAP kPa (absolute), z[iy * nx + ix].
 * About 15 Nm (friction) at 20-30 kPa, full torque at 100 kPa. */
static const int16_t tqRpm[6] = {800, 1500, 2500, 3500, 4500, 6000};
static const int16_t tqMap[5] = {20, 40, 60, 80, 100};
static const int32_t tqNm[30] = {
    10,  12,  15,  15,  12,  8,   // 20 kPa: friction, slightly negative feel
    50,  65,  75,  75,  70,  55,  // 40 kPa
    110, 135, 150, 150, 140, 115, // 60 kPa
    165, 200, 225, 225, 210, 175, // 80 kPa
    220, 265, 300, 300, 280, 235, // 100 kPa: full load
};
static const CalMap tqModelMap = {6, 5, tqRpm, tqMap, tqNm};

namespace TorqueIn {

// 0x8000 and 0x7FFF are "signal not available" codes, never a torque.
bool RawValid(int16_t raw) { return raw != (int16_t)0x8000 && raw != 0x7FFF; }

// Placeholder torque estimate from engine speed and manifold pressure.
int32_t Model(const TorqueModelIn &m) {
  int32_t map = m.mapKpa;
  int32_t base = Cal::Map(tqModelMap, m.rpm, map > 100 ? 100 : map);
  int64_t nm = (int64_t)base * m.scalePct / 100;
  if (map > 100)
    nm = nm * map / 100 * m.boostPct / 100; // boost: MAP ratio x multiplier
  return (int32_t)nm;
}

// Torque to use: CAN if usable, else the model if on, else the substitute.
uint8_t Select(int32_t canNm, bool canUsable, bool modelOn,
               const TorqueModelIn &m, int32_t substituteNm, int32_t *nm) {
  if (canUsable) {
    *nm = canNm;
    return TQSRC_CAN;
  }
  if (modelOn) {
    *nm = Model(m);
    return TQSRC_MODEL;
  }
  *nm = substituteNm;
  return TQSRC_SUBSTITUTE;
}

} // namespace TorqueIn

// True once the value has stood still while the engine speed moved.
bool FrozenCheck::Tick(int32_t value, int32_t rpm, int32_t timeMs, int32_t rpmDelta) {
  if (timeMs <= 0) {
    frozen = false;
    init = false;
    return false;
  }
  if (!init || value != last) {
    // The value moved: restart the window.
    init = true;
    last = value;
    rpmMin = rpmMax = rpm;
    ms = 0;
    frozen = false;
    return false;
  }
  if (rpm < rpmMin) rpmMin = rpm;
  if (rpm > rpmMax) rpmMax = rpm;
  if (ms < timeMs)
    ms += 10;
  frozen = ms >= timeMs && rpmMax - rpmMin > rpmDelta;
  return frozen;
}
