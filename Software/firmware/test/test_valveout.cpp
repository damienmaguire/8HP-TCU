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
#include "fake_max22200.h"
#include "test.h"
#include "valveout.h"

static Max22200Config Cfg() {
  Max22200Config c;
  c.ifsmA = 1000;
  c.halfScale = false;
  c.freq80k = false;
  c.freqDiv = 0;
  c.slewControl = false;
  c.openLoadDetect = true;
  c.maxCurrentmA = 850;
  return c;
}

void TestValveOut() {
  FakeMaxBus bus;
  Max22200 c0(bus, 0), c1(bus, 1);
  ValveOut v;
  v.AttachChip(0, &c0);
  v.AttachChip(1, &c1);
  CHECK(!v.Validate()); // nothing mapped: A..SYS must be driven
  for (int i = 0; i < EDS_COUNT; i++)
    v.SetMap((EdsValve)i, i); // A..E, SYS, K0, POS on chip 0; rest on chip 1
  CHECK(v.Validate());
  v.SetMap(EDS_B, 0); // B on A's channel
  CHECK(!v.Validate());
  v.SetMap(EDS_B, 16); // out of range
  CHECK(!v.Validate());
  v.SetMap(EDS_B, 1);
  v.SetMap(EDS_PARK, VALVE_NOT_FITTED); // optional valves may be absent
  CHECK(v.Validate());
  v.SetMap(EDS_SYS, VALVE_NOT_FITTED);
  CHECK(!v.Validate());
  v.SetMap(EDS_SYS, 5);
  v.AttachChip(1, 0); // second chip not fitted but PZYL mapped to it
  CHECK(!v.Validate());
  v.AttachChip(1, &c1);

  Max22200Hit hit = {0, 0};
  CHECK(v.Init(Cfg(), hit));

  Eds e;
  EdsConfig ec = e.Config();
  ec.slewPer10ms = 0;
  ec.ditherAmp = 0;
  ec.tempCorrection = false;
  e.Configure(ec);
  e.SetCurrent(EDS_A, 8500);
  e.SetCurrent(EDS_C, 4000);
  e.SetCurrent(EDS_SOL1, 5000); // chip 1 channel 1
  e.Tick10ms();
  e.Tick1ms();
  v.Update(e);
  CHECK_EQ(bus.reg[0][1 + 0] >> 24, 108);
  CHECK_EQ(bus.reg[0][1 + 2] >> 24, 51);
  CHECK_EQ(bus.reg[1][1 + 1] >> 24, 64);
  CHECK_EQ(bus.reg[0][0] >> 24, 0x05); // A and C on
  CHECK_EQ(bus.reg[1][0] >> 24, 0x02); // SOL1 on
  v.AllOff();
  CHECK_EQ(bus.reg[0][0] >> 24, 0);
  CHECK_EQ(bus.reg[1][0] >> 24, 0);

  // OCP on chip 0 channel 2 = valve C (critical); open load on chip 1
  // channel 1 = SOL1 (not critical)
  bus.reg[0][MAX22200_REG_FAULT] = 0x04u << 24;
  bus.reg[1][MAX22200_REG_FAULT] = 0x02u << 8;
  uint16_t f = v.PollFaults();
  CHECK_EQ(f, (1u << EDS_C) | (1u << EDS_SOL1));
  CHECK(f & ValveOut::CriticalMask());
  CHECK(!(ValveOut::CriticalMask() & (1u << EDS_SOL1)));
  CHECK(!v.ChipLost());
  CHECK_EQ(v.PollFaults(), 0); // cleared

  // Over-temperature flags every valve on that chip
  bus.reg[1][0] |= MAX22200_ST_OVT;
  f = v.PollFaults();
  CHECK_EQ(f, (1u << EDS_PZYL) | (1u << EDS_SOL1));
}
