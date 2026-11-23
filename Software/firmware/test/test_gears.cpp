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
#include "gears.h"
#include "test.h"
#include <string.h>

/* Ian's table, 330E_OPEN_TCU_CONTROL_SPEC.md section 5.1.
 * Row = from gear, column = to gear. "XY" = X released, Y applied.
 * "--" = 2-swap (no single swap), ".." = same gear. */
static const char *kSwap[8][8] = {
    {"..", "CE", "AE", "--", "AD", "--", "BD", "--"},
    {"EC", "..", "AC", "AD", "--", "--", "--", "BD"},
    {"EA", "CA", "..", "CD", "ED", "BD", "--", "--"},
    {"--", "DA", "DC", "..", "EC", "BC", "--", "BA"},
    {"DA", "--", "DE", "CE", "..", "BE", "BA", "--"},
    {"--", "--", "DB", "CB", "EB", "..", "EA", "CA"},
    {"DB", "--", "--", "--", "AB", "AE", "..", "CE"},
    {"--", "DB", "--", "AB", "--", "AC", "EC", ".."},
};

static void TestClutchTable() {
  // Section 4: R=ABD 1=ABC 2=ABE 3=BCE 4=BDE 5=BCD 6=CDE 7=ACD 8=ADE
  const char *t[10] = {"", "ABC", "ABE", "BCE", "BDE", "BCD",
                       "CDE", "ACD", "ADE", "ABD"};
  for (int g = 1; g <= 9; g++) {
    uint8_t m = 0;
    for (const char *p = t[g]; *p; p++)
      m |= CLUTCH_BIT(*p - 'A');
    CHECK_EQ(Gears::ClutchMask(g), m);
    CHECK_EQ(Gears::CountClutches(Gears::ClutchMask(g)), 3);
  }
  CHECK_EQ(Gears::ClutchMask(GEAR_N), 0);
  // Power off = C+D+E = 6th
  CHECK_EQ(Gears::ClutchMask(GEAR_LIMP),
           CLUTCH_BIT(CL_C) | CLUTCH_BIT(CL_D) | CLUTCH_BIT(CL_E));
}

static void TestSwapTable() {
  for (int f = 1; f <= 8; f++) {
    for (int t = 1; t <= 8; t++) {
      int off = -1, on = -1;
      bool ok = Gears::SingleSwap(f, t, off, on);
      const char *e = kSwap[f - 1][t - 1];
      if (e[0] == '.' || e[0] == '-') {
        CHECK(!ok);
      } else {
        CHECK(ok);
        CHECK_EQ(Gears::ClutchName(off), e[0]);
        CHECK_EQ(Gears::ClutchName(on), e[1]);
      }
    }
  }
  int off, on;
  CHECK(Gears::SingleSwap(1, 2, off, on));
  CHECK_EQ(off, CL_C);
  CHECK_EQ(on, CL_E);
  CHECK(Gears::SingleSwap(2, 1, off, on));
  CHECK_EQ(off, CL_E);
  CHECK_EQ(on, CL_C);
}

static void TestRatiosAndSteps() {
  CHECK_EQ(Gears::RatioX1000(1), 4714);
  CHECK_EQ(Gears::RatioX1000(6), 1000);
  CHECK_EQ(Gears::RatioX1000(8), 667);
  CHECK_EQ(Gears::RatioX1000(GEAR_R), 3306);
  CHECK_EQ(Gears::SyncSpeed(1, 1000), 4714);
  CHECK_EQ(Gears::SyncSpeed(8, 3000), 2001);
  // Adjacent steps only (v1)
  CHECK_EQ(Gears::NextStep(1, 2), 2);
  CHECK_EQ(Gears::NextStep(1, 8), 2);
  CHECK_EQ(Gears::NextStep(8, 1), 7);
  CHECK_EQ(Gears::NextStep(4, 4), 4);
  CHECK(Gears::IsDriveGear(GEAR_R));
  CHECK(!Gears::IsDriveGear(GEAR_N));
}

void TestGears() {
  TestClutchTable();
  TestSwapTable();
  TestRatiosAndSteps();
}
