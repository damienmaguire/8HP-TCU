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
#include "tle92466.h"

// Plain bit-by-bit CRC-8 SAE J1850 (poly 0x1D, init 0xFF, xor out 0xFF)
// over a byte string, written separately from the driver's.
static uint8_t RefJ1850(const uint8_t *b, int n) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < n; i++)
    for (int bit = 7; bit >= 0; bit--) {
      bool in = ((b[i] >> bit) & 1) != ((crc >> 7) & 1);
      crc = (uint8_t)(crc << 1);
      if (in)
        crc ^= 0x1D;
    }
  return crc ^ 0xFF;
}

static void TestCrcAndFrames() {
  // The published check value of CRC-8 SAE J1850 for "123456789" is 0x4B.
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK_EQ(RefJ1850(check, 9), 0x4B);
  // The driver takes the frame bytes in the order [7:0], [15:8], [23:16].
  const uint32_t frames[] = {0x000000, 0x811234, 0x018001, 0x000242, 0xFFFFFF, 0x5A3C96};
  for (uint32_t f : frames) {
    uint8_t b[3] = {(uint8_t)f, (uint8_t)(f >> 8), (uint8_t)(f >> 16)};
    CHECK_EQ(Tle::Crc8(f), RefJ1850(b, 3));
    CHECK_EQ(Tle::Crc8(f | 0xAB000000u), Tle::Crc8(f)); // CRC bits ignored
  }
  // Write frame: [23:17] address, [16] = 1, [15:0] data, CRC on top.
  uint32_t w = Tle::WriteFrame(0x40, 0x1234);
  CHECK_EQ(w & 0xFFFFFF, 0x811234);
  CHECK_EQ(w >> 24, Tle::Crc8(0x811234));
  CHECK_EQ(Tle::WriteFrame(TLE_CH_CTRL, 0x8001) & 0xFFFFFF, 0x018001);
  // Read frame: [16] = 0, [15:0] the 16-bit address
  uint32_t r = Tle::ReadFrame(0x0242);
  CHECK_EQ(r & 0xFFFFFF, 0x000242);
  CHECK_EQ(r >> 24, Tle::Crc8(0x000242));
  // Channel addresses: CH0..3 at 0x40..0x70, CH4 0x20, CH5 0x30
  CHECK_EQ(Tle::ChAddr(0, TLE_SETPOINT), 0x40);
  CHECK_EQ(Tle::ChAddr(3, TLE_MODE), 0x7C);
  CHECK_EQ(Tle::ChAddr(4, TLE_CH_CONFIG), 0x27);
  CHECK_EQ(Tle::ChAddr(5, TLE_FB_I_AVG), 0x232);
  CHECK_EQ(Tle::ChAddr(0, TLE_FB_I_AVG), 0x242);
  CHECK_EQ(Tle::ChAddr(6, TLE_SETPOINT), 0xFFFF);
}

static uint32_t Reply(uint32_t f24) { return ((uint32_t)Tle::Crc8(f24) << 24) | (f24 & 0xFFFFFF); }

static void TestParse() {
  // 16-bit reply: mode 00, status, R/W, data
  TleReply p = Tle::Parse(Reply((1ul << 16) | 0xBEEF));
  CHECK_EQ(p.mode, TLE_REPLY_16);
  CHECK_EQ(p.status, 0);
  CHECK(p.rw);
  CHECK_EQ(p.data, 0xBEEF);
  CHECK(p.crcOk);
  p = Tle::Parse(Reply(2ul << 17)); // CRC error status
  CHECK_EQ(p.status, 2);
  CHECK(!p.rw);
  // 22-bit reply: mode 01
  p = Tle::Parse(Reply((1ul << 22) | 0x2ABCDE));
  CHECK_EQ(p.mode, TLE_REPLY_22);
  CHECK_EQ(p.data, 0x2ABCDE);
  // Critical fault frame: mode 10, no CRC check
  p = Tle::Parse((2ul << 22) | 0x5A | 0x77000000u);
  CHECK_EQ(p.mode, TLE_REPLY_CRITICAL);
  CHECK_EQ(p.data, 0x5A);
  CHECK(p.crcOk);
  // A flipped CRC bit is caught
  p = Tle::Parse(Reply(0x1234) ^ 0x01000000u);
  CHECK(!p.crcOk);
}

static void TestScaling() {
  // SETPOINT: I = 2 A x code / 32767
  CHECK_EQ(Tle::SetpointCode(0), 0);
  CHECK_EQ(Tle::SetpointCode(-5), 0);
  CHECK_EQ(Tle::SetpointCode(10000), 16384); // 1 A
  CHECK_EQ(Tle::SetpointCode(4000), 6553);   // 400 mA
  CHECK_EQ(Tle::SetpointCode(15000), 24575); // 1.5 A
  CHECK_EQ(Tle::SetpointCode(20000), 24575); // limited to 1.5 A
  CHECK(Tle::SetpointCode(99999) < 0x6000);  // never in the saturated range
  // Average current: 4 A x I_AVG_MANT / TP_MANT, signed
  CHECK_EQ(Tle::AvgCurrent(250, 1000), 10000);
  CHECK_EQ(Tle::AvgCurrent(0xFFF, 1000), -40);
  CHECK_EQ(Tle::AvgCurrent(250, 0), 0);
  CHECK_EQ(Tle::AvgCurrent(250 | (3u << 12), 1000 | (5u << 11)), 10000); // EXP / TO bits ignored
  // PWM 1 kHz at 28 MHz: 28000 cycles = 218 x 2^7
  CHECK_EQ(Tle::PeriodReg(1000), (7 << 8) | 218);
  CHECK_EQ(Tle::PeriodReg(0), 0);
  // Dither: 8 ms triangle, 30 mA peak. tref = 875 x 2^2 / 28 MHz = 125 us,
  // T = 64 tref = 8 ms; 16 steps x 31 x 2 A / 32767 = 30.3 mA.
  TleDither d = Tle::DitherRegs(300, 8);
  CHECK_EQ(d.clkDiv, (2 << 10) | 875);
  CHECK_EQ(d.step, 16 << 8);
  CHECK_EQ(d.ctrl, 31);
  CHECK_EQ(Tle::DitherRegs(0, 8).ctrl, 0);
  CHECK_EQ(Tle::DitherRegs(1000, 8).ctrl, 102); // 100 mA
  CHECK_EQ(Tle::DitherRegs(0, 8).clkDiv, d.clkDiv); // clock set even with no dither
  // CH_CONFIG: slew 2.5 V/us; own check on with OL_TH 2/8, or both checks off
  CHECK_EQ(Tle::ChConfig(1, true, 2), 0x0021);
  CHECK_EQ(Tle::ChConfig(1, false, 2), 0xC001);
  // FB_VOLTAGE2: temperature and VBAT
  CHECK_EQ(Tle::TempC(1314), 24);
  CHECK_EQ(Tle::VbatmV(2047ul << 11), 41470);
  CHECK_EQ(Tle::VbatmV(617ul << 11), 12500);
  // Resistance estimate: VBAT 12.5 V (617 / 2047 x 41.47), duty 25 %,
  // I = 4 A x 142 / 2047 = 277 mA -> R = 3.125 V / 0.277 A = 11.26 ohm
  uint32_t dc = 2047 | (512ul << 11), vb = 617;
  CHECK_EQ(Tle::CoilMilliOhm(dc, vb, 142), 41470LL * 617 * 512 / (4LL * 142 * 2047));
  CHECK(Tle::CoilMilliOhm(dc, vb, 142) > 11000 && Tle::CoilMilliOhm(dc, vb, 142) < 11500);
  CHECK_EQ(Tle::CoilMilliOhm(dc, vb, 0), 0);     // no current
  CHECK_EQ(Tle::CoilMilliOhm(dc, vb, 0xFFF), 0); // negative current
  CHECK_EQ(Tle::CoilMilliOhm(0, vb, 142), 0);    // no period
}

static TleConfig Cfg() {
  TleConfig c = TleConfig();
  c.usedMask = 0x3F;
  c.olMask = 0x0F;
  c.olRelEighths = 2;
  c.pwmHz = 1000;
  c.slew = 1;
  return c;
}

static void TestDriver() {
  FakeTleBus bus;
  Tle92466 t(bus, 0);
  CHECK(t.Init(Cfg()));
  CHECK(bus.Mission(0));
  CHECK_EQ(bus.Enabled(0), 0x3F);
  CHECK_EQ(bus.reg[0][TLE_GLOBAL_CONFIG], TLE_GC_CLK_WD_EN | TLE_GC_CRC_EN); // VIO 3.3 V, no SPI watchdog
  CHECK_EQ(bus.reg[0][TLE_GLOBAL_DIAG0], 0); // start-up flags cleared
  CHECK_EQ(bus.reg[0][TLE_FAULT_MASK0], 0xC03F);
  CHECK_EQ(bus.Ch(0, 0, TLE_MODE), TLE_MODE_ICC);
  CHECK_EQ(bus.Ch(0, 5, TLE_MODE), TLE_MODE_ICC);
  CHECK_EQ(bus.Ch(0, 0, TLE_CH_CONFIG), 0x0021); // own check on
  CHECK_EQ(bus.Ch(0, 4, TLE_CH_CONFIG), 0xC001); // own check off
  CHECK_EQ(bus.Ch(0, 2, TLE_PERIOD), (7 << 8) | 218);
  CHECK_EQ(bus.Ch(0, 2, TLE_DITHER_CLK_DIV), (2 << 10) | 875);
  CHECK_EQ(bus.Ch(0, 2, TLE_DITHER_CTRL), 0);
  CHECK_EQ(t.CommErrors(), 0);

  // Setpoints: only changes go out
  t.SetCurrent(0, 4000, 300);
  t.SetCurrent(5, 9500);
  t.Flush();
  CHECK_EQ(bus.Ch(0, 0, TLE_SETPOINT), 6553);
  CHECK_EQ(bus.Ch(0, 0, TLE_DITHER_CTRL), 31);
  CHECK_EQ(bus.Ch(0, 5, TLE_SETPOINT), Tle::SetpointCode(9500));
  int w = bus.writes[0][0x40];
  t.Flush();
  CHECK_EQ(bus.writes[0][0x40], w); // nothing changed, nothing sent
  t.SetCurrent(0, 0);
  t.Flush();
  CHECK_EQ(bus.Ch(0, 0, TLE_SETPOINT), 0);
  CHECK_EQ(bus.Ch(0, 0, TLE_DITHER_CTRL), 0);

  // Unused channel: never driven
  Tle92466 u(bus, 1);
  TleConfig c4 = Cfg();
  c4.usedMask = 0x0F;
  CHECK(u.Init(c4));
  CHECK_EQ(bus.Enabled(1), 0x0F);
  CHECK_EQ(bus.Ch(1, 4, TLE_MODE), TLE_MODE_OFF);
  CHECK_EQ(bus.reg[1][TLE_FAULT_MASK0], 0xC00F);
  u.SetCurrent(4, 5000);
  u.Flush();
  CHECK_EQ(bus.Ch(1, 4, TLE_SETPOINT), 0);

  // Faults: over-current on CH2 switched it off; CH4 of the unused-channel
  // chip reports open load, which is ignored.
  bus.ChannelFault(0, 2, TLE_DE_OC);
  TleDiag d;
  CHECK(t.ReadDiag(&d));
  CHECK(d.commOk && d.mission && !d.critical);
  CHECK_EQ(d.oc, 1u << 2);
  CHECK_EQ(d.ol, 0);
  CHECK_EQ(d.enabled, 0x3F & ~(1u << 2));
  CHECK_EQ(bus.reg[0][TLE_DIAG_ERR_CHGR(1)], 0); // cleared
  CHECK_EQ(bus.Enabled(0), 0x3F);                // and switched on again
  bus.ChannelFault(1, 4, TLE_DE_OL);
  CHECK(u.ReadDiag(&d));
  CHECK_EQ(d.ol, 0);
  // The same with a chip whose flags clear on writing 1
  bus.clr1 = true;
  bus.ChannelFault(0, 5, TLE_DE_SG | TLE_DE_OLSG);
  CHECK(t.ReadDiag(&d));
  CHECK_EQ(d.sg, 1u << 5);
  CHECK_EQ(d.olsg, 1u << 5);
  CHECK_EQ(bus.reg[0][TLE_DIAG_ERR_CHGR(2)], 0);
  CHECK_EQ(bus.Enabled(0), 0x3F);
  CHECK(t.ReadDiag(&d));
  CHECK_EQ(d.sg | d.olsg | d.oc | d.ol | d.ote, 0);
  bus.clr1 = false;

  // Supply fault: the chip falls back to Config Mode by itself
  bus.DropToConfig(0, 1u << 2);
  CHECK(t.ReadDiag(&d));
  CHECK(!d.mission);
  CHECK(d.diag0 & (1u << 2));
  t.SetCurrent(0, 4000);
  t.Flush(); // not sent while the chip is down
  CHECK_EQ(bus.Ch(0, 0, TLE_SETPOINT), 0);
  CHECK(t.Init(Cfg())); // back into Mission Mode
  t.Flush();
  CHECK_EQ(bus.Ch(0, 0, TLE_SETPOINT), 6553); // setpoints sent again
  CHECK(bus.Mission(0));

  // Current read-back, the reply pipeline (reply i answers frame i) and
  // the resistance estimate
  bus.SetMeasured(0, 0, 4000);
  bus.SetMeasured(0, 3, 2500);
  bus.reg[0][Tle::ChAddr(0, TLE_FB_DC)] = 1000 | (250ul << 11);           // 25 % duty
  bus.reg[0][Tle::ChAddr(0, TLE_FB_VBAT)] = 1000ul * 12000 / 41470;       // 12 V
  bus.reg[0][TLE_FB_VOLTAGE2] = (617ul << 11) | 1314;
  t.ReadCurrents();
  CHECK_EQ(t.Current(0), 4000);
  CHECK_EQ(t.Current(3), 2480); // 62 / 1000 x 4 A
  CHECK_EQ(t.Current(1), 0);
  CHECK_EQ(t.VbatmV(), 12500);
  CHECK_EQ(t.TempC(), 24);
  // 3 V / 0.4 A = 7.5 ohm
  CHECK(t.CoilMilliOhm(0) > 7400 && t.CoilMilliOhm(0) < 7600);
  CHECK_EQ(t.CoilMilliOhm(1), 0);
  uint32_t fr[3] = {Tle::ReadFrame(TLE_ICVID), Tle::ReadFrame(Tle::ChAddr(0, TLE_SETPOINT)),
                    Tle::ReadFrame(TLE_GLOBAL_CONFIG)};
  uint32_t rr[3];
  CHECK(t.Exchange(fr, 3, rr));
  CHECK_EQ(Tle::Parse(rr[0]).data, 0xC100);
  CHECK_EQ(Tle::Parse(rr[1]).data, 6553);
  CHECK_EQ(Tle::Parse(rr[2]).data, 0x0005);

  // Bad CRC on a reply counts as an error
  uint32_t e0 = t.CommErrors();
  bus.corruptNext = true;
  uint32_t one = Tle::ReadFrame(TLE_ICVID);
  CHECK(!t.Exchange(&one, 1, rr));
  CHECK_EQ(t.CommErrors(), e0 + 1);
  // A chip that does not answer: replies all ones, which fail the CRC
  bus.dead[0] = true;
  CHECK(!t.ReadDiag(&d));
  CHECK(!d.commOk && !d.mission);
  CHECK(!t.Init(Cfg()));
  bus.dead[0] = false;
  // Critical fault frame
  bus.critical[0] = true;
  t.ReadDiag(&d);
  CHECK(d.critical);
  bus.critical[0] = false;
  // No INIT_DONE: Init gives up
  bus.reg[1][TLE_FB_STAT] = 0;
  CHECK(!u.Init(c4));
}

void TestTle92466() {
  TestCrcAndFrames();
  TestParse();
  TestScaling();
  TestDriver();
}
