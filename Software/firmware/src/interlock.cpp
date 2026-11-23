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
/*
 * Hard clutch interlock: the last check before any current reaches the
 * valves. Rules and their sources are in interlock.h.
 */
#include "interlock.h"

namespace Interlock {

// A and B are open at low current; C, D and E are open at high current.
bool NormallyLow(int cl) { return cl == CL_A || cl == CL_B; }

// Is this clutch open at this commanded current?
bool IsOpen(int cl, int32_t i, int32_t openLow, int32_t openHigh) {
  return NormallyLow(cl) ? i < openLow : i > openHigh;
}

// Clutches that are not open at the commanded currents.
uint8_t AppliedMask(const InterlockInput &in) {
  uint8_t m = 0;
  for (int c = 0; c < CL_COUNT; c++)
    if (!IsOpen(c, in.current[c], in.openLow, in.openHigh))
      m |= CLUTCH_BIT(c);
  return m;
}

// Does this set of applied clutches form a real gear?
static bool IsGearMask(uint8_t m) {
  for (int g = GEAR_1; g <= GEAR_R; g++)
    if (Gears::ClutchMask(g) == m)
      return true;
  return false;
}

// The hard safety check run before any output (rules in interlock.h).
// In plain words: with too many clutches closed the gearbox locks up (two
// gears fighting each other), which can lock the wheels. So whatever the
// shift logic asks for, if the currents would close a clutch it should not,
// more than one gear's worth, or a mix that is not a gear, the TCU goes to
// its safe state instead of sending them.
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
