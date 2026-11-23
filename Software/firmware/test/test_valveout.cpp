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
  c.halfScaleMask = 0;
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
  v.SetMap(EDS_PZYL, VALVE_NOT_FITTED); // optional valves may be absent
  CHECK(v.Validate());
  v.SetMap(EDS_SYS, VALVE_NOT_FITTED);
  CHECK(!v.Validate());
  v.SetMap(EDS_SYS, 5);
  v.SetMap(EDS_PZYL, 8); // PZYL on chip 1 channel 0
  v.AttachChip(1, 0); // second chip not fitted but PZYL mapped to it
  CHECK(!v.Validate());
  v.AttachChip(1, &c1);

  // HFS per channel: bit = global channel. C (chip 0 ch 2) on half scale.
  CHECK(v.Init(Cfg(), 1u << 2));
  CHECK(bus.reg[0][1 + 2] & MAX22200_CFG_HFS);
  CHECK(!(bus.reg[0][1 + 0] & MAX22200_CFG_HFS));

  Eds e;
  EdsConfig ec = e.Config();
  ec.slewPer10ms = 0;
  ec.ditherAmp = 0;
  ec.tempCorrection = false;
  e.Configure(ec);
  e.SetCurrent(EDS_A, 8500);
  e.SetCurrent(EDS_C, 4000);
  e.SetCurrent(EDS_PZYL, 5000); // chip 1 channel 0
  e.Tick10ms();
  e.Tick1ms();
  v.Update(e);
  CHECK_EQ(bus.reg[0][1 + 0] >> 24, 108);
  // C on half scale: 400 mA of 500 mA full scale = code 102, HFS bit set
  CHECK_EQ(bus.reg[0][1 + 2] >> 24, 0x80 | 102);
  CHECK_EQ(bus.reg[1][1 + 0] >> 24, 64);
  CHECK_EQ(bus.reg[0][0] >> 24, 0x05); // A and C on
  CHECK_EQ(bus.reg[1][0] >> 24, 0x01); // PZYL on
  v.AllOff();
  CHECK_EQ(bus.reg[0][0] >> 24, 0);
  CHECK_EQ(bus.reg[1][0] >> 24, 0);

  // OCP on chip 0 channel 2 = valve C (critical); open load on chip 1
  // channel 0 = PZYL (not critical)
  bus.reg[0][MAX22200_REG_FAULT] = 0x04u << 24;
  bus.reg[1][MAX22200_REG_FAULT] = 0x01u << 8;
  uint16_t f = v.PollFaults();
  CHECK_EQ(f, (1u << EDS_C) | (1u << EDS_PZYL));
  CHECK(f & ValveOut::CriticalMask());
  CHECK(!(ValveOut::CriticalMask() & (1u << EDS_PZYL)));
  CHECK(!v.ChipLost());
  // Per-valve flags for the spot values
  CHECK_EQ(v.ValveFlags(EDS_C), VALVE_F_OCP);
  CHECK_EQ(v.ValveFlags(EDS_PZYL), VALVE_F_OLF);
  CHECK_EQ(v.ValveFlags(EDS_A), 0);
  CHECK_EQ(v.PollFaults(), 0); // cleared
  CHECK_EQ(v.ValveFlags(EDS_C), 0);

  // Over-temperature flags every valve on that chip
  bus.reg[1][0] |= MAX22200_ST_OVT;
  f = v.PollFaults();
  CHECK_EQ(f, 1u << EDS_PZYL);
  CHECK_EQ(v.ValveFlags(EDS_PZYL), VALVE_F_CHIP);
  CHECK_EQ(v.ValveFlags(EDS_C), 0); // other chip
  bus.reg[1][0] &= ~MAX22200_ST_OVT;
  v.PollFaults();

  // Rev B general-purpose outputs on chip 1 OUT1..OUT3 (global 9..11).
  v.SetAuxMap(0, 8); // on PZYL's channel: refused
  CHECK(!v.Validate());
  v.SetAuxMap(0, 9);
  v.SetAuxMap(1, 10);
  v.SetAuxMap(2, 11);
  CHECK(v.Validate());
  v.SetAuxMap(2, 3); // on valve D's channel: refused
  CHECK(!v.Validate());
  v.SetAuxMap(2, 11);
  CHECK(v.Init(Cfg(), 0));
  v.SetAux(0, 4000); // 400 mA on
  v.SetAux(1, 0);    // off
  v.SetAux(2, 2000);
  v.Update(e);
  CHECK_EQ(bus.reg[1][1 + 1] >> 24, 51); // 400 mA of 1 A = code 51
  CHECK_EQ(bus.reg[1][0] >> 24, 0x01 | 0x02 | 0x08); // PZYL, OUT1, OUT3
  bus.reg[1][MAX22200_REG_FAULT] = 0x02u << 8; // open load on OUT1
  f = v.PollFaults();
  CHECK_EQ(f, 0); // not a valve fault
  CHECK_EQ(v.AuxFlags(0), VALVE_F_OLF);
  CHECK_EQ(v.AuxFlags(1), 0);
  v.AllOff();
  CHECK_EQ(bus.reg[1][0] >> 24, 0);
}
