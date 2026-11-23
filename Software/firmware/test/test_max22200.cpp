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
#include "max22200.h"
#include "test.h"

static Max22200Config DefaultCfg() {
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

static void TestCodes() {
  // Command byte, Table 9: [7] write, [4:1] address, [0] 8 bit
  CHECK_EQ(Max22200::CommandByte(MAX22200_REG_STATUS, false, false), 0x00);
  CHECK_EQ(Max22200::CommandByte(MAX22200_REG_STATUS, true, false), 0x80);
  CHECK_EQ(Max22200::CommandByte(MAX22200_REG_STATUS, true, true), 0x81);
  CHECK_EQ(Max22200::CommandByte(MAX22200_REG_CFG_CH(0), true, true), 0x83);
  CHECK_EQ(Max22200::CommandByte(MAX22200_REG_CFG_CH(7), true, false), 0x90);
  CHECK_EQ(Max22200::CommandByte(MAX22200_REG_FAULT, false, false), 0x12);
  CHECK_EQ(Max22200::CommandByte(MAX22200_REG_CFG_DPM, true, false), 0x94);

  // HOLD code = I / IFS x 127, rounded
  CHECK_EQ(Max22200::CurrentToCode(8500, 1000, false, 126), 108);
  CHECK_EQ(Max22200::CurrentToCode(500, 1000, false, 126), 6);
  CHECK_EQ(Max22200::CurrentToCode(0, 1000, false, 126), 0);
  CHECK_EQ(Max22200::CurrentToCode(1, 1000, false, 126), 1); // smallest step
  CHECK_EQ(Max22200::CurrentToCode(-50, 1000, false, 126), 0);
  // HFS halves full scale: 400 mA at IFS 1000 / 2
  CHECK_EQ(Max22200::CurrentToCode(4000, 1000, true, 126), 102);
  // Never 127 (unregulated full on), whatever is asked
  for (int32_t i = 0; i <= 20000; i += 37)
    CHECK(Max22200::CurrentToCode(i, 1000, false, 255) <= MAX22200_CODE_MAX);
  CHECK_EQ(Max22200::CurrentToCode(10000, 1000, false, 127), 126);
  // Ceiling from maxCode
  CHECK_EQ(Max22200::CurrentToCode(9000, 1000, false, 108), 108);
  CHECK_EQ(Max22200::CodeToCurrent(108, 1000, false), 8504);
  CHECK_EQ(Max22200::CodeToCurrent(127, 1000, false), 9921); // clamped to 126

  // CFG word: CDR, low side, HOLD in [30:24]
  Max22200Config c = DefaultCfg();
  uint32_t w = Max22200::CfgWord(c, 108, 0, 0);
  CHECK_EQ((w >> 24) & 0x7F, 108);
  CHECK_EQ(w & MAX22200_CFG_HFS, 0);
  CHECK_EQ(w & MAX22200_CFG_VDRNCDR, 0);
  CHECK_EQ(w & MAX22200_CFG_HSNLS, 0);
  CHECK_EQ(w & MAX22200_CFG_TRGNSPI, 0);
  CHECK(w & MAX22200_CFG_OL_EN);
  CHECK_EQ(w & MAX22200_CFG_HHF_EN, 0);
  CHECK_EQ((Max22200::CfgWord(c, 127, 127, 0) >> 24) & 0x7F, 126);
  c.freqDiv = 3;
  c.halfScale = true;
  w = Max22200::CfgWord(c, 10, 50, 20);
  CHECK_EQ((w >> 4) & 3, 3);
  CHECK(w & MAX22200_CFG_HFS);
  CHECK_EQ((w >> 16) & 0x7F, 50);
  CHECK_EQ((w >> 8) & 0xFF, 20);
  CHECK(w & MAX22200_CFG_HHF_EN);
}

static void TestInitAndWrites() {
  FakeMaxBus bus;
  Max22200 chip(bus, 1);
  CHECK(chip.Init(DefaultCfg()));
  CHECK(!chip.EchoError());
  CHECK(bus.cmdHighAtRise); // CMD still high at the CSB rising edge
  CHECK_EQ(bus.reg[1][0] & MAX22200_ST_UVM, 0);  // cleared by the first read
  CHECK_EQ(bus.reg[1][0] & MAX22200_ST_ACTIVE, 1);
  CHECK_EQ(bus.reg[1][0] >> 24, 0);              // all channels off
  CHECK_EQ(bus.reg[0][0] & MAX22200_ST_UVM, MAX22200_ST_UVM); // other chip untouched
  for (int ch = 0; ch < 8; ch++)
    CHECK_EQ(bus.reg[1][1 + ch], Max22200::CfgWord(DefaultCfg(), 0, 0, 0));

  // HOLD update is a single-byte write, and only when the code changes
  int n = bus.write8Count;
  chip.SetCurrent(3, 8500);
  CHECK_EQ(bus.write8Count, n + 1);
  CHECK_EQ(bus.reg[1][1 + 3] >> 24, 108);
  CHECK_EQ(chip.HoldCode(3), 108);
  chip.SetCurrent(3, 8501); // same code
  CHECK_EQ(bus.write8Count, n + 1);
  // The rest of the CFG word survives an 8-bit write
  CHECK_EQ(bus.reg[1][1 + 3] & 0x00FFFFFFu,
           Max22200::CfgWord(DefaultCfg(), 0, 0, 0) & 0x00FFFFFFu);
  // Ceiling = 850 mA even if more is asked
  chip.SetCurrent(3, 20000);
  CHECK_EQ(bus.reg[1][1 + 3] >> 24, 108);

  chip.SetOnMask(0x09);
  CHECK_EQ(bus.reg[1][0] >> 24, 0x09);
  CHECK_EQ(chip.GetOnMask(), 0x09);
  CHECK_EQ(bus.reg[1][0] & MAX22200_ST_ACTIVE, 1);

  // Echo check catches a write that did not take
  bus.badEcho = true;
  Max22200Hit h = {500, 10};
  chip.SetHit(2, h);
  CHECK(chip.EchoError());
}

static void TestFaults() {
  FakeMaxBus bus;
  Max22200 chip(bus, 0);
  CHECK(chip.Init(DefaultCfg()));
  // OCP on ch 1, open load on ch 6
  bus.reg[0][MAX22200_REG_FAULT] = (0x02u << 24) | (0x40u << 8);
  bus.reg[0][0] |= MAX22200_ST_OCP | MAX22200_ST_OLF;
  Max22200Faults f = chip.ReadFaults();
  CHECK_EQ(f.ocp, 0x02);
  CHECK_EQ(f.olf, 0x40);
  CHECK_EQ(f.hhf, 0);
  CHECK_EQ(f.dpm, 0);
  CHECK(f.status & MAX22200_ST_OCP);
  CHECK_EQ(bus.reg[0][MAX22200_REG_FAULT], 0); // cleared selectively

  // A chip still flagging a fault after init reports failure
  FakeMaxBus bus2;
  Max22200 chip2(bus2, 0);
  bus2.reg[0][0] |= MAX22200_ST_OVT;
  CHECK(!chip2.Init(DefaultCfg()));
}

void TestMax22200() {
  TestCodes();
  TestInitAndWrites();
  TestFaults();
}
