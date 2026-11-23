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
#include "test.h"

static InterlockInput Make(uint8_t applied, uint8_t allowed, bool shifting) {
  InterlockInput in;
  for (int c = 0; c < CL_COUNT; c++) {
    in.fillLevel[c] = 1000;
    in.pressure[c] = (applied & CLUTCH_BIT(c)) ? 10000 : 500;
  }
  in.allowedMask = allowed;
  in.shifting = shifting;
  return in;
}

void TestInterlock() {
  const uint8_t ALL = 0x1F;
  // Every gear on its own is fine.
  for (int g = 1; g <= 9; g++) {
    uint8_t m = Gears::ClutchMask(g);
    CHECK_EQ(Interlock::Check(Make(m, m, false)), IL_OK);
  }
  // Four applied outside a shift = locked gearbox.
  uint8_t four = CLUTCH_BIT(CL_A) | CLUTCH_BIT(CL_B) | CLUTCH_BIT(CL_C) |
                 CLUTCH_BIT(CL_E);
  CHECK_EQ(Interlock::Check(Make(four, ALL, false)), IL_TOO_MANY);
  CHECK_EQ(Interlock::Check(Make(ALL, ALL, false)), IL_TOO_MANY);
  // Three that are not a gear (A C E).
  uint8_t ace = CLUTCH_BIT(CL_A) | CLUTCH_BIT(CL_C) | CLUTCH_BIT(CL_E);
  CHECK_EQ(Interlock::Check(Make(ace, ALL, false)), IL_NOT_A_GEAR);
  // Outside the allowed set.
  CHECK_EQ(Interlock::Check(Make(Gears::ClutchMask(2), Gears::ClutchMask(1), false)),
           IL_NOT_ALLOWED);
  // During a 1->2 swap the union ABCE may be applied for the overlap...
  uint8_t u12 = Gears::ClutchMask(1) | Gears::ClutchMask(2);
  CHECK_EQ(Interlock::Check(Make(u12, u12, true)), IL_OK);
  // ...but never D, and never all five.
  CHECK_EQ(Interlock::Check(Make(u12 | CLUTCH_BIT(CL_D), u12, true)), IL_NOT_ALLOWED);
  CHECK_EQ(Interlock::Check(Make(ALL, ALL, true)), IL_TOO_MANY);
  // Pressure exactly at the fill level does not count as applied.
  InterlockInput in = Make(0, 0, false);
  in.pressure[CL_B] = in.fillLevel[CL_B];
  CHECK_EQ(Interlock::AppliedMask(in), 0);
  CHECK_EQ(Interlock::Check(in), IL_OK);
}
