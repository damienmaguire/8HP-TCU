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
#include "fake_tle92466.h"
#include "test.h"
#include "valveout.h"

static TleConfig Base() {
  TleConfig c = TleConfig();
  c.pwmHz = 1000;
  c.olRelEighths = 2;
  c.slew = 1;
  return c;
}

// V5 default map: IC10 A, B, C, D, (PHOLD), PZYL; IC11 E, LINE, PREL, TCC.
static void MapV5(ValveOut &v) {
  v.SetMap(EDS_A, 0);
  v.SetMap(EDS_B, 1);
  v.SetMap(EDS_C, 2);
  v.SetMap(EDS_D, 3);
  v.SetMap(EDS_PZYL, 5);
  v.SetMap(EDS_E, 8);
  v.SetMap(EDS_SYS, 9);
  v.SetMap(EDS_POS, 10);
  v.SetMap(EDS_K0, 11);
  v.SetAuxMap(VALVE_AUX_PHOLD, 4);
}

static void TestDitherTable() {
  // Ian's 330e map at 25 degC (A..E, TCC / K0)
  CHECK_EQ(ValveOut::DitherFor(EDS_A, 500), 0);
  CHECK_EQ(ValveOut::DitherFor(EDS_A, 1200), 0);
  CHECK_EQ(ValveOut::DitherFor(EDS_A, 1500), 300);
  CHECK_EQ(ValveOut::DitherFor(EDS_C, 3100), 700);
  CHECK_EQ(ValveOut::DitherFor(EDS_D, 3900), 900);
  CHECK_EQ(ValveOut::DitherFor(EDS_E, 4000), 925);
  CHECK_EQ(ValveOut::DitherFor(EDS_K0, 5000), 1000);
  CHECK_EQ(ValveOut::DitherFor(EDS_B, 6200), 700);
  CHECK_EQ(ValveOut::DitherFor(EDS_A, 7800), 0);
  CHECK_EQ(ValveOut::DitherFor(EDS_A, 8500), 0);
  // LINE flat 0.03 A from 0.10 to 0.80 A
  CHECK_EQ(ValveOut::DitherFor(EDS_SYS, 900), 0);
  CHECK_EQ(ValveOut::DitherFor(EDS_SYS, 5000), 300);
  CHECK_EQ(ValveOut::DitherFor(EDS_SYS, 8500), 0);
  // No dither on PREL / POS and PZYL
  CHECK_EQ(ValveOut::DitherFor(EDS_POS, 4000), 0);
  CHECK_EQ(ValveOut::DitherFor(EDS_PZYL, 4000), 0);
  // Never above the TLE's 1.8 A setpoint + dither limit
  for (int32_t i = 0; i <= 8500; i += 50)
    CHECK(ValveOut::DitherFor(EDS_A, i) <= 1000);
  // Current-error limit
  CHECK_EQ(ValveOut::CurrentErrLimit(4000), 1200);
  CHECK_EQ(ValveOut::CurrentErrLimit(3999), 1700);
}

void TestValveOut() {
  TestDitherTable();
  FakeTleBus bus;
  Tle92466 c0(bus, 0), c1(bus, 1);
  ValveOut v;
  v.AttachChip(0, &c0);
  v.AttachChip(1, &c1);
  CHECK(!v.Validate()); // nothing mapped: A..SYS must be driven
  MapV5(v);
  CHECK(v.Validate());
  v.SetMap(EDS_B, 0); // B on A's channel
  CHECK(!v.Validate());
  v.SetMap(EDS_B, 6); // the chip has channels 0..5 only
  CHECK(!v.Validate());
  v.SetMap(EDS_B, 16); // out of range
  CHECK(!v.Validate());
  v.SetMap(EDS_B, 4); // on PHOLD
  CHECK(!v.Validate());
  v.SetMap(EDS_B, 1);
  v.SetMap(EDS_POS, VALVE_NOT_FITTED); // optional valves may be absent
  CHECK(v.Validate());
  v.SetMap(EDS_SYS, VALVE_NOT_FITTED);
  CHECK(!v.Validate());
  v.SetMap(EDS_SYS, 9);
  v.SetMap(EDS_POS, 10);
  v.AttachChip(1, 0); // second chip missing but valves mapped to it
  CHECK(!v.Validate());
  v.AttachChip(1, &c1);

  CHECK(v.Init(Base(), true));
  CHECK_EQ(bus.Enabled(0), 0x3F);
  CHECK_EQ(bus.Enabled(1), 0x0F); // IC11 CH4 / CH5 not connected: off
  CHECK_EQ(bus.Ch(1, 4, TLE_MODE), TLE_MODE_OFF);
  // The chip's own open-load / short check (which switches a channel off)
  // runs on the valves, never on PHOLD (CH4) or PZYL (CH5)
  CHECK_EQ(bus.Ch(0, 0, TLE_CH_CONFIG), 0x0021);
  CHECK_EQ(bus.Ch(0, 4, TLE_CH_CONFIG), 0xC001);
  CHECK_EQ(bus.Ch(0, 5, TLE_CH_CONFIG), 0xC001);
  CHECK_EQ(bus.Ch(1, 2, TLE_CH_CONFIG), 0x0021);
  // MaxOpenLoad off: no own check anywhere
  CHECK(v.Init(Base(), false));
  CHECK_EQ(bus.Ch(0, 0, TLE_CH_CONFIG), 0xC001);
  CHECK(v.Init(Base(), true));

  Eds e;
  EdsConfig ec = e.Config();
  ec.slewPer10ms = 0;
  ec.tempCorrection = false;
  e.Configure(ec);
  e.SetCurrent(EDS_A, 4000);
  e.SetCurrent(EDS_E, 8500);
  e.SetCurrent(EDS_SYS, 5000);
  e.SetCurrent(EDS_PZYL, 9500);
  e.Tick10ms();
  v.SetAux(VALVE_AUX_PHOLD, 2700);
  v.Update(e);
  CHECK_EQ(bus.Ch(0, 0, TLE_SETPOINT), Tle::SetpointCode(4000));
  CHECK_EQ(bus.Ch(0, 0, TLE_DITHER_CTRL), Tle::DitherRegs(925, 8).ctrl);
  CHECK_EQ(bus.Ch(1, 0, TLE_SETPOINT), Tle::SetpointCode(8500));
  CHECK_EQ(bus.Ch(1, 0, TLE_DITHER_CTRL), 0); // no dither at the end stop
  CHECK_EQ(bus.Ch(1, 1, TLE_DITHER_CTRL), Tle::DitherRegs(300, 8).ctrl);
  CHECK_EQ(bus.Ch(0, 5, TLE_SETPOINT), Tle::SetpointCode(9500)); // PZYL pull-in
  CHECK_EQ(bus.Ch(0, 5, TLE_DITHER_CTRL), 0);
  CHECK_EQ(bus.Ch(0, 4, TLE_SETPOINT), Tle::SetpointCode(2700)); // PHOLD
  CHECK_EQ(bus.Ch(0, 4, TLE_DITHER_CTRL), 0);

  // Measured currents: right on target, so no current error
  bus.SetMeasured(0, 0, 4000);
  bus.SetMeasured(1, 0, 8500);
  bus.SetMeasured(1, 1, 5000);
  bus.SetMeasured(0, 5, 9500);
  bus.SetMeasured(0, 4, 2700);
  CHECK_EQ(v.PollFaults(), 0);
  CHECK_EQ(v.Measured(EDS_A), 4000);
  CHECK_EQ(v.Measured(EDS_SYS), 5000);
  CHECK_EQ(v.AuxMeasured(VALVE_AUX_PHOLD), 2680); // 67 / 1000 x 4 A
  CHECK_EQ(v.Measured(EDS_B), 0);
  CHECK(!v.ChipLost());

  // Resistance estimate. PZYL: hold 0.40 A, 25 % duty at 12 V = 3 V ->
  // 7.5 ohm. Nothing on the first poll after a setpoint change (not
  // steady), then filtered; 0 when the current is too small or off.
  e.SetCurrent(EDS_PZYL, 4000);
  e.Tick10ms();
  v.Update(e);
  bus.SetMeasured(0, 5, 4000);
  bus.reg[0][Tle::ChAddr(5, TLE_FB_DC)] = 1000 | (250ul << 11);
  bus.reg[0][Tle::ChAddr(5, TLE_FB_VBAT)] = 1000ul * 12000 / 41470;
  v.PollFaults();
  CHECK_EQ(v.CoilMilliOhm(EDS_PZYL), 0); // setpoint just changed
  v.PollFaults();
  int32_t r1 = v.CoilMilliOhm(EDS_PZYL);
  CHECK(r1 > 7400 && r1 < 7600);
  // A higher reading moves the filtered value by 1/8 of the step
  bus.reg[0][Tle::ChAddr(5, TLE_FB_DC)] = 1000 | (330ul << 11);
  v.PollFaults();
  int32_t r2 = v.CoilMilliOhm(EDS_PZYL);
  CHECK(r2 > r1 && r2 < r1 + 400);
  // Below 0.2 A: not valid, shows 0 (the filter keeps its value)
  e.SetCurrent(EDS_A, 1500);
  e.Tick10ms();
  v.Update(e);
  bus.SetMeasured(0, 0, 1500);
  v.PollFaults();
  v.PollFaults();
  CHECK_EQ(v.CoilMilliOhm(EDS_A), 0);
  CHECK_EQ(v.CoilMilliOhm(EDS_B), 0); // off
  // Read-only: the estimate never changed a setpoint
  CHECK_EQ(bus.Ch(0, 5, TLE_SETPOINT), Tle::SetpointCode(4000));

  // Current error on A: report only, never in the returned mask
  bus.SetMeasured(0, 0, 3500); // 150 mA set, 350 mA measured (> 170 mA off)
  uint16_t f = v.PollFaults();
  CHECK_EQ(f, 0);
  CHECK_EQ(v.ValveFlags(EDS_A), VALVE_F_IERR);
  // PZYL below its setpoint (low supply) is fine
  bus.SetMeasured(0, 5, 2000);
  v.PollFaults();
  CHECK_EQ(v.ValveFlags(EDS_PZYL), 0);
  bus.SetMeasured(0, 0, 1500);
  bus.SetMeasured(0, 5, 4000);

  // Over-current on IC10 CH2 = valve C (critical); open load on IC11 CH4
  // (not connected) is ignored; short to ground on PZYL is reported but
  // not critical, and PZYL is switched on again
  bus.ChannelFault(0, 2, TLE_DE_OC);
  bus.ChannelFault(1, 4, TLE_DE_OL);
  bus.ChannelFault(0, 5, TLE_DE_SG);
  f = v.PollFaults();
  CHECK_EQ(f, (1u << EDS_C) | (1u << EDS_PZYL));
  CHECK(f & ValveOut::CriticalMask());
  CHECK(!(ValveOut::CriticalMask() & (1u << EDS_PZYL)));
  CHECK_EQ(v.ValveFlags(EDS_C), VALVE_F_OCP);
  CHECK_EQ(v.ValveFlags(EDS_PZYL), VALVE_F_SG);
  CHECK_EQ(bus.Enabled(0), 0x3F);
  CHECK_EQ(v.PollFaults(), 0); // cleared
  CHECK_EQ(v.ValveFlags(EDS_C), 0);

  // Open load on PHOLD: an aux fault, not a valve fault
  bus.ChannelFault(0, 4, TLE_DE_OL);
  CHECK_EQ(v.PollFaults(), 0);
  CHECK_EQ(v.AuxFlags(VALVE_AUX_PHOLD), VALVE_F_OLF);

  // IC10 falls back to Config Mode (VDD dip): every valve on it gets the
  // chip flag, and it is put straight back into Mission Mode with the
  // setpoints (PZYL included) sent again, without a reset.
  bus.DropToConfig(0, 1u << 4);
  int resets = bus.writes[0][TLE_GLOBAL_CONFIG];
  f = v.PollFaults();
  CHECK(f & (1u << EDS_A));
  CHECK(f & (1u << EDS_PZYL));
  CHECK(!(f & (1u << EDS_E)));
  CHECK_EQ(v.ValveFlags(EDS_A), VALVE_F_CHIP);
  CHECK(v.SupplyFlags() & TLE_SUP_VDD);
  CHECK(v.SupplyFlags() & TLE_SUP_NOT_MISSION0);
  CHECK(v.SupplyFlags() & TLE_SUP_RECOVER);
  CHECK(bus.Mission(0));
  CHECK_EQ(bus.writes[0][TLE_GLOBAL_CONFIG], resets + 1);
  CHECK(!v.ChipLost());
  v.Update(e);
  CHECK_EQ(bus.Ch(0, 5, TLE_SETPOINT), Tle::SetpointCode(4000));
  CHECK_EQ(bus.Ch(0, 4, TLE_SETPOINT), Tle::SetpointCode(2700));

  // IC11 stops answering: lost after VALVE_CHIP_MISSES polls in a row
  // (one bad answer is not a fault)
  bus.dead[1] = true;
  f = v.PollFaults();
  CHECK(!(f & (1u << EDS_E)));
  CHECK(!v.ChipLost());
  CHECK(v.SupplyFlags() & TLE_SUP_COMM);
  v.PollFaults();
  f = v.PollFaults();
  CHECK(f & (1u << EDS_E));
  CHECK(v.ChipLost());
  CHECK(v.SupplyFlags() & TLE_SUP_COMM);
  bus.dead[1] = false;

  // AllOff: every channel to 0 mA, chips stay in Mission Mode
  v.AllOff();
  CHECK_EQ(bus.Ch(0, 5, TLE_SETPOINT), 0);
  CHECK_EQ(bus.Ch(0, 0, TLE_SETPOINT), 0);
  CHECK(bus.Mission(0));
}
