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
#include "interlock.h"

namespace Interlock {

bool NormallyLow(int cl) { return cl == CL_A || cl == CL_B; }

bool IsOpen(int cl, int32_t i, int32_t openLow, int32_t openHigh) {
  return NormallyLow(cl) ? i < openLow : i > openHigh;
}

uint8_t AppliedMask(const InterlockInput &in) {
  uint8_t m = 0;
  for (int c = 0; c < CL_COUNT; c++)
    if (!IsOpen(c, in.current[c], in.openLow, in.openHigh))
      m |= CLUTCH_BIT(c);
  return m;
}

static bool IsGearMask(uint8_t m) {
  for (int g = GEAR_1; g <= GEAR_R; g++)
    if (Gears::ClutchMask(g) == m)
      return true;
  return false;
}

InterlockResult Check(const InterlockInput &in) {
  uint8_t applied = AppliedMask(in);
  int n = Gears::CountClutches(applied);

  if (n >= 5)
    return IL_TOO_MANY;
  if (applied & ~in.allowedMask)
    return IL_NOT_ALLOWED;
  if (in.shifting)
    return n > 4 ? IL_TOO_MANY : IL_OK;
  if (n > 3)
    return IL_TOO_MANY;
  if (n == 3 && !IsGearMask(applied))
    return IL_NOT_A_GEAR;
  return IL_OK;
}

} // namespace Interlock
