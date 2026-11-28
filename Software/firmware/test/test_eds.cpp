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
#include "eds.h"
#include "test.h"

static void TestCurves() {
  // KL_EDSKVA end points and one point from the export: 4000 mbar -> 431.2 mA
  CHECK_EQ(Eds::PressureToCurrent(EDS_A, 0), 500);
  CHECK_EQ(Eds::PressureToCurrent(EDS_A, 4000), 4312);
  CHECK_EQ(Eds::PressureToCurrent(EDS_A, 11500), 8500);
  CHECK_EQ(Eds::PressureToCurrent(EDS_A, 30000), 8500);
  CHECK_EQ(Eds::PressureToCurrent(EDS_A, -10), 500);
  // A rises with pressure (normally low)...
  CHECK(Eds::PressureToCurrent(EDS_A, 8000) > Eds::PressureToCurrent(EDS_A, 2000));
  // ...C, D, E and SYS fall (normally high: 0 A = full pressure)
  CHECK(Eds::PressureToCurrent(EDS_C, 8000) < Eds::PressureToCurrent(EDS_C, 2000));
  CHECK(Eds::PressureToCurrent(EDS_D, 8000) < Eds::PressureToCurrent(EDS_D, 2000));
  CHECK(Eds::PressureToCurrent(EDS_E, 8000) < Eds::PressureToCurrent(EDS_E, 2000));
  CHECK(Eds::PressureToCurrent(EDS_SYS, 15000) < Eds::PressureToCurrent(EDS_SYS, 5000));
  // No curve for the current-command outputs
  CHECK_EQ(Eds::PressureToCurrent(EDS_PZYL, 5000), 0);
}

static void TestSlewAndSafe() {
  Eds e;
  EdsConfig c = e.Config();
  c.tempCorrection = false;
  e.Configure(c);
  e.SetCurrent(EDS_A, 3000);
  e.Tick10ms();
  CHECK_EQ(e.Setpoint(EDS_A), 600); // 0.03 A per 5 ms = 60 mA per 10 ms
  for (int i = 0; i < 10; i++)
    e.Tick10ms();
  CHECK_EQ(e.Setpoint(EDS_A), 3000);
  e.SetCurrent(EDS_A, 99999);
  CHECK_EQ(e.Target(EDS_A), 8500); // clamped to iMax
  // POS (current command) is slew limited like the rest
  e.SetCurrent(EDS_POS, 5000);
  e.Tick10ms();
  CHECK_EQ(e.Setpoint(EDS_POS), 600);
  // PZYL is on / off: not slewed, and its pull-in may exceed iMax
  e.SetCurrent(EDS_PZYL, 9500);
  e.Tick10ms();
  CHECK_EQ(e.Output(EDS_PZYL), 9500);
  e.SetCurrent(EDS_PZYL, 99999);
  CHECK_EQ(e.Target(EDS_PZYL), EDS_PZYL_IMAX);
  e.SetCurrent(EDS_PZYL, 4000);
  // Safe state drops every valve at once, but never PZYL (park lock)
  e.SafeState();
  for (int v = 0; v < EDS_COUNT; v++)
    if (v != EDS_PZYL)
      CHECK_EQ(e.Output((EdsValve)v), 0);
  e.Tick10ms();
  CHECK_EQ(e.Output(EDS_PZYL), 4000);
  e.SetCurrent(EDS_PZYL, 0); // the caller decides when PZYL goes off
  e.Tick10ms();
  CHECK_EQ(e.Output(EDS_PZYL), 0);
  e.SetCurrent(EDS_A, 3000);
  e.Tick10ms();
  CHECK_EQ(e.Setpoint(EDS_A), 0); // still safe until Release()
  e.Release();
  e.SetCurrent(EDS_A, 3000);
  e.Tick10ms();
  CHECK_EQ(e.Setpoint(EDS_A), 600);
}

void TestEds() {
  TestCurves();
  TestSlewAndSafe();
}
