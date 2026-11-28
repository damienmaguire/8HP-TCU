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
#include "parksafe.h"
#include "test.h"

// The park-lock safety rule: PZYL must not drop while the car moves.
void TestParkSafe() {
  ParkSafe p;
  p.Configure(4000, PZYL_PULL_IN);
  // Off and parked: nothing happens, drivers may restart
  CHECK_EQ(p.Update(0, 0, true), 0);
  CHECK(!p.Holding());
  CHECK(p.MayRestartDrivers());
  // Moving with PZYL never energised (lock in, e.g. ParkCtrl off): no hold
  CHECK_EQ(p.Update(0, 1000, true), 0);
  CHECK(!p.Holding());

  // Release at standstill: pull-in 0.95 A for 40 ms (4 ticks), then hold
  CHECK_EQ(p.Update(4000, 0, true), PZYL_PULL_IN);
  CHECK_EQ(p.Update(4000, 0, true), PZYL_PULL_IN);
  CHECK_EQ(p.Update(4000, 0, true), PZYL_PULL_IN);
  CHECK_EQ(p.Update(4000, 0, true), PZYL_PULL_IN);
  CHECK_EQ(p.Update(4000, 0, true), 4000);
  CHECK(!p.Holding()); // not moving yet
  CHECK(p.MayRestartDrivers());

  // Driving away: hold latches at PARK_HOLD_RPM
  CHECK_EQ(p.Update(4000, PARK_HOLD_RPM - 1, true), 4000);
  CHECK(!p.Holding());
  CHECK_EQ(p.Update(4000, PARK_HOLD_RPM, true), 4000);
  CHECK(p.Holding());
  CHECK(!p.MayRestartDrivers());

  // Fault -> safe state asks for 0 while moving: PZYL stays at the hold
  // current, no pull-in pulse
  CHECK_EQ(p.Update(0, 2000, true), 4000);
  CHECK_EQ(p.Update(0, 2000, true), 4000);
  CHECK(!p.MayRestartDrivers());
  // Reverse (negative speed) counts the same
  CHECK_EQ(p.Update(0, -800, true), 4000);
  CHECK(p.Holding());
  // Speed sensor lost: assume moving, keep holding
  CHECK_EQ(p.Update(0, 0, false), 4000);
  CHECK(p.Holding());
  CHECK(!p.MayRestartDrivers());
  // Car stopped (known speed below the threshold): the hold lets go and
  // the TCU's 0 goes through, so the lock can go in at standstill
  CHECK_EQ(p.Update(0, 10, true), 0);
  CHECK(!p.Holding());
  CHECK(p.MayRestartDrivers());

  // The hold keeps the higher of PzylCurrent and the TCU's last request
  // (cold-oil offset)
  p.Update(5000, 0, true);
  for (int i = 0; i < 5; i++)
    p.Update(5000, 0, true);
  CHECK_EQ(p.Update(5000, 1500, true), 5000);
  CHECK_EQ(p.Update(0, 1500, true), 5000);
  // A lower request while moving and holding is lifted to the hold current
  CHECK_EQ(p.Update(1000, 1500, true), 4000);
  p.Update(0, 0, true);

  // Speed unknown at switch-on: the hold latches once PZYL is on
  CHECK_EQ(p.Update(4000, 0, false), PZYL_PULL_IN);
  CHECK(p.Holding());
  CHECK_EQ(p.Update(0, 0, false), PZYL_PULL_IN); // pull-in still running, never off
  for (int i = 0; i < 3; i++)
    p.Update(0, 0, false);
  CHECK_EQ(p.Update(0, 0, false), 4000);
  CHECK_EQ(p.Output(), 4000);

  // Together with the EDS safe state: the safe state zeroes the valves
  // but the PZYL current from the rule goes through.
  Eds e;
  e.SetCurrent(EDS_A, 3000);
  e.SafeState();
  e.SetCurrent(EDS_PZYL, p.Update(0, 1200, true));
  e.Tick10ms();
  CHECK_EQ(e.Output(EDS_A), 0);
  CHECK_EQ(e.Output(EDS_PZYL), 4000);
}
