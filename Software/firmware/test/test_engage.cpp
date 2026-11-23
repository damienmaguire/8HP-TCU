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
#include "engage.h"
#include "gears.h"
#include "test.h"

static EngageConfig Cfg() {
  EngageConfig c;
  c.pMax = 12750;
  c.fillEqMs = 50;
  c.rampMs = 300;
  c.syncBand = 30;
  c.syncCloseGrad = 20000;
  return c;
}

static void RunEngage(int gear) {
  Engage e;
  CHECK(e.Start(gear, Cfg()));
  int pc = Engage::PowerFlowClutch(gear);
  uint8_t mask = Gears::ClutchMask(gear);
  int lastPhase = ENG_FILL;
  bool order = true;
  int ms = 0;
  EngageInputs in = {20, 0, 0, 40}; // standstill, motor stopped
  while (!e.Done() && ms < 10000) {
    e.Tick(in);
    if (e.Phase() < lastPhase)
      order = false;
    lastPhase = e.Phase();
    // Only the elements of the target gear ever get pressure
    for (int c = 0; c < CL_COUNT; c++)
      if (!(mask & CLUTCH_BIT(c)))
        CHECK_EQ(e.Pressure(c), 0);
    // The power-flow element waits until the other two are filled
    if (e.Phase() == ENG_FILL)
      CHECK_EQ(e.Pressure(pc), 0);
    ms += 10;
  }
  CHECK(e.Done());
  CHECK(order);
  CHECK(ms < 5000);
  for (int c = 0; c < CL_COUNT; c++)
    if (mask & CLUTCH_BIT(c))
      CHECK(e.Pressure(c) >= 12750);
}

void TestEngage() {
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_1), CL_C);
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_R), CL_D);
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_2), -1);
  Engage e;
  CHECK(!e.Start(GEAR_2, Cfg()));
  RunEngage(GEAR_1);
  RunEngage(GEAR_R);
}
