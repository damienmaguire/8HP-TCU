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
#include "eds.h"
#include "test.h"

#define OPEN_LOW 600   // 60 mA
#define OPEN_HIGH 7900 // 790 mA

/* Build an input with the clutches in "applied" at a mid-range current and
 * the rest at their open end stop (A/B 50 mA, C/D/E 850 mA). */
static InterlockInput Make(uint8_t applied, uint8_t allowed, bool shifting) {
  InterlockInput in;
  for (int c = 0; c < CL_COUNT; c++) {
    bool low = Interlock::NormallyLow(c);
    if (applied & CLUTCH_BIT(c))
      in.current[c] = 4000;
    else
      in.current[c] = low ? 500 : 8500;
  }
  in.openLow = OPEN_LOW;
  in.openHigh = OPEN_HIGH;
  in.allowedMask = allowed;
  in.shifting = shifting;
  return in;
}

void TestInterlock() {
  const uint8_t ALL = 0x1F;
  // Valve types: A and B normally low, C D E normally high.
  CHECK(Interlock::NormallyLow(CL_A));
  CHECK(Interlock::NormallyLow(CL_B));
  CHECK(!Interlock::NormallyLow(CL_C));
  CHECK(!Interlock::NormallyLow(CL_D));
  CHECK(!Interlock::NormallyLow(CL_E));
  // Thresholds are strict: A/B open below 60 mA, C/D/E open above 790 mA.
  CHECK(Interlock::IsOpen(CL_A, 599, OPEN_LOW, OPEN_HIGH));
  CHECK(!Interlock::IsOpen(CL_A, 600, OPEN_LOW, OPEN_HIGH));
  CHECK(Interlock::IsOpen(CL_B, 0, OPEN_LOW, OPEN_HIGH));
  CHECK(Interlock::IsOpen(CL_C, 7901, OPEN_LOW, OPEN_HIGH));
  CHECK(!Interlock::IsOpen(CL_C, 7900, OPEN_LOW, OPEN_HIGH));
  CHECK(!Interlock::IsOpen(CL_D, 500, OPEN_LOW, OPEN_HIGH)); // 0 A on D = applied
  CHECK(!Interlock::IsOpen(CL_E, 0, OPEN_LOW, OPEN_HIGH));

  // All coils at 0 A is C D E = 6th, which is a gear but not "neutral".
  InterlockInput zero = Make(0, 0, false);
  for (int c = 0; c < CL_COUNT; c++)
    zero.current[c] = 0;
  CHECK_EQ(Interlock::AppliedMask(zero), Gears::ClutchMask(6));
  CHECK_EQ(Interlock::Check(zero), IL_NOT_ALLOWED);

  // Every gear on its own is fine.
  for (int g = 1; g <= 9; g++) {
    uint8_t m = Gears::ClutchMask(g);
    CHECK_EQ(Interlock::Check(Make(m, m, false)), IL_OK);
  }
  // Nothing applied is fine.
  CHECK_EQ(Interlock::Check(Make(0, 0, false)), IL_OK);
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
  // Every swap: the union of from and to may be applied (4 clutches)...
  for (int g = 1; g < 8; g++) {
    for (int d = 0; d < 2; d++) {
      int from = d ? g + 1 : g, to = d ? g : g + 1;
      uint8_t u = Gears::ClutchMask(from) | Gears::ClutchMask(to);
      CHECK_EQ(Gears::CountClutches(u), 4);
      CHECK_EQ(Interlock::Check(Make(u, u, true)), IL_OK);
      // ...and the fifth clutch must stay open.
      uint8_t fifth = (uint8_t)(ALL & ~u);
      CHECK_EQ(Interlock::Check(Make(u | fifth, u, true)), IL_TOO_MANY);
      CHECK_EQ(Interlock::Check(Make(Gears::ClutchMask(from) | fifth, u, true)),
               IL_NOT_ALLOWED);
    }
  }
  // Five is refused even if a caller wrongly allowed everything.
  CHECK_EQ(Interlock::Check(Make(ALL, ALL, true)), IL_TOO_MANY);

  // A clutch just short of its open threshold counts as applied.
  InterlockInput in = Make(0, 0, false);
  in.current[CL_A] = OPEN_LOW;
  CHECK_EQ(Interlock::AppliedMask(in), CLUTCH_BIT(CL_A));
  CHECK_EQ(Interlock::Check(in), IL_NOT_ALLOWED);
  in = Make(0, 0, false);
  in.current[CL_E] = OPEN_HIGH;
  CHECK_EQ(Interlock::Check(in), IL_NOT_ALLOWED);

  // With the real OEM curves, 0 mbar is open and any fill pressure is not.
  for (int c = 0; c < CL_COUNT; c++) {
    int32_t i0 = Eds::CommandCurrent((EdsValve)c, 0, 40, true, 8500);
    int32_t iFill = Eds::CommandCurrent((EdsValve)c, 300, 40, true, 8500);
    CHECK(Interlock::IsOpen(c, i0, OPEN_LOW, OPEN_HIGH));
    CHECK(!Interlock::IsOpen(c, iFill, OPEN_LOW, OPEN_HIGH));
  }
}
