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
  c.ditherAmp = 0;
  e.Configure(c);
  e.SetCurrent(EDS_A, 3000);
  e.Tick10ms();
  CHECK_EQ(e.Setpoint(EDS_A), 600); // 0.03 A per 5 ms = 60 mA per 10 ms
  for (int i = 0; i < 10; i++)
    e.Tick10ms();
  CHECK_EQ(e.Setpoint(EDS_A), 3000);
  e.SetCurrent(EDS_A, 99999);
  CHECK_EQ(e.Target(EDS_A), 8500); // clamped to iMax
  // Current-command outputs are slew limited like the rest
  e.SetCurrent(EDS_PZYL, 5000);
  e.Tick10ms();
  CHECK_EQ(e.Setpoint(EDS_PZYL), 600);
  // Safe state drops everything at once
  e.SafeState();
  e.Tick1ms();
  for (int v = 0; v < EDS_COUNT; v++)
    CHECK_EQ(e.Output((EdsValve)v), 0);
  e.SetCurrent(EDS_A, 3000);
  e.Tick10ms();
  CHECK_EQ(e.Setpoint(EDS_A), 0); // still safe until Release()
  e.Release();
  e.SetCurrent(EDS_A, 3000);
  e.Tick10ms();
  CHECK_EQ(e.Setpoint(EDS_A), 600);
}

static void TestDither() {
  // Triangle, 8 ms (125 Hz), +-30 mA
  const int32_t amp = 300;
  int32_t lo = 99999, hi = -99999, sum = 0;
  for (uint32_t t = 0; t < 8; t++) {
    int32_t d = Eds::DitherValue(t, amp, 8, DITHER_TRIANGLE);
    lo = d < lo ? d : lo;
    hi = d > hi ? d : hi;
    sum += d;
  }
  CHECK_EQ(lo, -amp);
  CHECK_EQ(hi, amp);
  CHECK_EQ(sum, 0); // no DC offset over one period
  CHECK_EQ(Eds::DitherValue(0, amp, 8, DITHER_TRIANGLE),
           Eds::DitherValue(8, amp, 8, DITHER_TRIANGLE)); // periodic
  CHECK_EQ(Eds::DitherValue(0, amp, 8, DITHER_SQUARE), amp);
  CHECK_EQ(Eds::DitherValue(4, amp, 8, DITHER_SQUARE), -amp);
  CHECK_EQ(Eds::DitherValue(3, 0, 8, DITHER_SQUARE), 0);

  Eds e;
  EdsConfig c = e.Config();
  c.tempCorrection = false;
  c.slewPer10ms = 0; // jump straight to the target
  e.Configure(c);
  e.SetCurrent(EDS_A, 4000);
  e.SetCurrent(EDS_PZYL, 4000);
  e.Tick10ms();
  int32_t mn = 99999, mx = 0;
  for (int i = 0; i < 16; i++) {
    e.Tick1ms();
    int32_t o = e.Output(EDS_A);
    mn = o < mn ? o : mn;
    mx = o > mx ? o : mx;
    CHECK_EQ(e.Output(EDS_PZYL), 4000); // current-command outputs never dither
  }
  CHECK_EQ(mn, 3700);
  CHECK_EQ(mx, 4300);
  // No dither near the end stops
  e.SetCurrent(EDS_A, 8500);
  e.Tick10ms();
  for (int i = 0; i < 8; i++) {
    e.Tick1ms();
    CHECK_EQ(e.Output(EDS_A), 8500);
  }
  e.SetCurrent(EDS_A, 600);
  e.Tick10ms();
  e.Tick1ms();
  CHECK_EQ(e.Output(EDS_A), 600);
}

void TestEds() {
  TestCurves();
  TestSlewAndSafe();
  TestDither();
}
