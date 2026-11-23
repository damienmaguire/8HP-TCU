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
/* Tests for the reference-derived features: torque input, adaptation,
 * fault detection, TCC lock-up control and the status stream. */
#include "adapt.h"
#include "cal_sig.h"
#include "diag.h"
#include "gears.h"
#include "statstream.h"
#include "tcc.h"
#include "tcu.h"
#include "test.h"
#include "torquein.h"
#include "valveout.h"
#include <string.h>

static void TestTorqueIn() {
  // Not-available codes.
  CHECK(!TorqueIn::RawValid((int16_t)0x8000));
  CHECK(!TorqueIn::RawValid(0x7FFF));
  CHECK(TorqueIn::RawValid(0));
  CHECK(TorqueIn::RawValid(-200));
  CHECK(TorqueIn::RawValid(0x7FFE));

  TorqueModelIn m = {2500, 100, 100, 100};
  CHECK_EQ(TorqueIn::Model(m), 300);  // table peak
  m.scalePct = 150;
  CHECK_EQ(TorqueIn::Model(m), 450);  // scaled for a 450 Nm engine
  m.scalePct = 100;
  m.mapKpa = 150;
  CHECK_EQ(TorqueIn::Model(m), 450);  // 1.5 bar abs: MAP ratio
  m.boostPct = 80;
  CHECK_EQ(TorqueIn::Model(m), 360);  // with the boost multiplier
  m.mapKpa = 20;
  m.boostPct = 100;
  CHECK(TorqueIn::Model(m) < 20);     // idle: friction only
  m.mapKpa = 50;                      // between rows: interpolated
  CHECK_EQ(TorqueIn::Model(m), 112);

  int32_t nm = 0;
  m.mapKpa = 100;
  CHECK_EQ(TorqueIn::Select(123, true, true, m, 600, &nm), TQSRC_CAN);
  CHECK_EQ(nm, 123);
  CHECK_EQ(TorqueIn::Select(123, false, true, m, 600, &nm), TQSRC_MODEL);
  CHECK_EQ(nm, 300);
  CHECK_EQ(TorqueIn::Select(123, false, false, m, 600, &nm), TQSRC_SUBSTITUTE);
  CHECK_EQ(nm, 600);

  // Frozen: value stuck while rpm moves.
  FrozenCheck f;
  bool fr = false;
  for (int i = 0; i < 400; i++)
    fr = f.Tick(150, 1500 + i * 2, 3000, 300);
  CHECK(fr);
  CHECK(!f.Tick(151, 2300, 3000, 300)); // moved: clear
  // Stuck value with steady rpm (cruise) is not frozen.
  FrozenCheck g;
  for (int i = 0; i < 400; i++)
    fr = g.Tick(150, 2000, 3000, 300);
  CHECK(!fr);
  // Off.
  FrozenCheck h;
  for (int i = 0; i < 400; i++)
    fr = h.Tick(150, 1500 + i * 2, 0, 300);
  CHECK(!fr);
}

static AdaConfig AdaDefaults() {
  AdaConfig c;
  c.enable = true;
  c.tqMin = 55; c.tqMax = 180;
  c.rpmMin = 1600; c.rpmMax = 3200;
  c.oilMin = 55; c.oilMax = 80;
  c.gearMin = 3; c.gearMax = 7;
  c.pedalMin = 2;
  c.driveMinS = 180;
  c.shiftMs = 500; c.bandMs = 100;
  c.flareRpm = 100;
  c.step = 5;
  c.maxOfs = 300;
  return c;
}

static AdaInputs AdaGood(int gear) {
  AdaInputs in;
  in.mode = TCU_DRIVE;
  in.gear = in.target = (uint8_t)gear;
  in.m_t = 100;
  in.n_t = 2000;
  in.n_mot = 2000;
  in.n_ab = 2000 * 100 / 100;
  in.temp = 70;
  in.pedal = 20;
  in.drivenS = 600;
  in.normalMode = true;
  in.faultFree = true;
  return in;
}

// Run one shift from -> to lasting ms, with an optional flare.
static void AdaShift(Adapt &a, AdaInputs in, int from, int to, int ms, int flare) {
  in.gear = in.target = (uint8_t)from;
  in.n_ab = 1000;
  a.Tick10ms(in);
  in.mode = TCU_SHIFT;
  in.target = (uint8_t)to;
  for (int t = 0; t < ms; t += 10) {
    in.n_t = Gears::SyncSpeed(from, in.n_ab) + (t == ms / 2 ? flare : 0);
    a.Tick10ms(in);
  }
  in.mode = TCU_DRIVE;
  in.gear = (uint8_t)to;
  in.n_t = Gears::SyncSpeed(to, in.n_ab);
  a.Tick10ms(in);
}

static void TestAdapt() {
  CHECK_EQ(Adapt::LearnClutch(3, 4), CL_D);
  CHECK_EQ(Adapt::LearnClutch(4, 5), CL_C);
  CHECK_EQ(Adapt::LearnClutch(5, 6), CL_E);
  CHECK_EQ(Adapt::LearnClutch(6, 7), CL_A);
  CHECK_EQ(Adapt::LearnClutch(6, 5), CL_B);
  CHECK_EQ(Adapt::LearnClutch(2, 3), -1);
  CHECK_EQ(Adapt::LearnClutch(7, 8), -1);
  CHECK_EQ(Adapt::LearnClutch(5, 4), -1);

  CHECK_EQ(Adapt::Class(9), -1);
  CHECK_EQ(Adapt::Class(10), ADA_COLD);
  CHECK_EQ(Adapt::Class(30), ADA_COLD);
  CHECK_EQ(Adapt::Class(40), -1);
  CHECK_EQ(Adapt::Class(50), ADA_WARM);
  CHECK_EQ(Adapt::Class(110), ADA_WARM);
  CHECK_EQ(Adapt::Class(111), -1);

  // Default off: nothing learns.
  Adapt a;
  AdaConfig c = AdaDefaults();
  c.enable = false;
  a.Configure(c);
  AdaShift(a, AdaGood(3), 3, 4, 900, 0);
  CHECK_EQ(a.Raw(CL_D, ADA_WARM), 0);
  CHECK(a.Status() & ADA_BLK_DISABLED);

  // Each gate blocks.
  c = AdaDefaults();
  a.Configure(c);
  AdaInputs in = AdaGood(4);
  a.Tick10ms(in);
  CHECK_EQ(a.Status(), 0u);
  struct { void (*f)(AdaInputs &); uint32_t bit; } gates[] = {
      {[](AdaInputs &i) { i.m_t = 50; }, ADA_BLK_TORQUE},
      {[](AdaInputs &i) { i.m_t = 190; }, ADA_BLK_TORQUE},
      {[](AdaInputs &i) { i.n_mot = 1500; }, ADA_BLK_RPM},
      {[](AdaInputs &i) { i.n_mot = 3300; }, ADA_BLK_RPM},
      {[](AdaInputs &i) { i.temp = 85; }, ADA_BLK_OIL},
      {[](AdaInputs &i) { i.temp = 45; }, ADA_BLK_OIL | ADA_BLK_CLASS},
      {[](AdaInputs &i) { i.gear = i.target = 2; }, ADA_BLK_GEAR},
      {[](AdaInputs &i) { i.gear = i.target = 8; }, ADA_BLK_GEAR},
      {[](AdaInputs &i) { i.gear = i.target = GEAR_R; }, ADA_BLK_GEAR},
      {[](AdaInputs &i) { i.pedal = 1; }, ADA_BLK_PEDAL},
      {[](AdaInputs &i) { i.drivenS = 170; }, ADA_BLK_DRIVETIME},
      {[](AdaInputs &i) { i.normalMode = false; }, ADA_BLK_MODE},
      {[](AdaInputs &i) { i.faultFree = false; }, ADA_BLK_FAULT},
  };
  for (auto &g : gates) {
    AdaInputs b = AdaGood(4);
    g.f(b);
    a.Tick10ms(b);
    CHECK_EQ(a.Status(), g.bit);
  }

  // Late (long) shift: + step on D, warm class only.
  a.Reset();
  AdaShift(a, AdaGood(3), 3, 4, 700, 0);
  CHECK_EQ(a.Raw(CL_D, ADA_WARM), 5);
  CHECK_EQ(a.Raw(CL_D, ADA_COLD), 0);
  CHECK_EQ(a.Shifts(), 1);
  CHECK(a.Dirty());
  // In band: no change but counted.
  AdaShift(a, AdaGood(3), 3, 4, 500, 0);
  CHECK_EQ(a.Raw(CL_D, ADA_WARM), 5);
  // Harsh: - step.
  AdaShift(a, AdaGood(4), 4, 5, 300, 0);
  CHECK_EQ(a.Raw(CL_C, ADA_WARM), -5);
  // Flare: + step even at the target time.
  AdaShift(a, AdaGood(5), 5, 6, 500, 200);
  CHECK_EQ(a.Raw(CL_E, ADA_WARM), 5);
  AdaShift(a, AdaGood(6), 6, 7, 800, 0);
  CHECK_EQ(a.Raw(CL_A, ADA_WARM), 5);
  AdaShift(a, AdaGood(6), 6, 5, 800, 0);
  CHECK_EQ(a.Raw(CL_B, ADA_WARM), 5);
  // Not a learning shift.
  AdaShift(a, AdaGood(7), 7, 8, 800, 0);
  AdaShift(a, AdaGood(5), 5, 4, 800, 0);
  CHECK_EQ(a.Shifts(), 6);
  // Gate failing at the end of the shift: no learning.
  {
    AdaInputs i = AdaGood(3);
    i.gear = i.target = 3;
    a.Tick10ms(i);
    i.mode = TCU_SHIFT;
    i.target = 4;
    for (int t = 0; t < 800; t += 10) a.Tick10ms(i);
    i.mode = TCU_DRIVE;
    i.gear = 4;
    i.pedal = 0;
    a.Tick10ms(i);
    CHECK_EQ(a.Raw(CL_D, ADA_WARM), 5);
  }
  // Cold class learns separately, used below 40 degC.
  {
    c = AdaDefaults();
    c.oilMin = 10; c.oilMax = 80;
    a.Configure(c);
    AdaInputs i = AdaGood(3);
    i.temp = 20;
    AdaShift(a, i, 3, 4, 900, 0);
    CHECK_EQ(a.Raw(CL_D, ADA_COLD), 5);
    CHECK_EQ(a.Offset(CL_D, 20), 5);
    CHECK_EQ(a.Offset(CL_D, 39), 5);
    CHECK_EQ(a.Offset(CL_D, 40), 5);  // warm D is also 5
    CHECK_EQ(a.Offset(CL_C, 39), 0);
    CHECK_EQ(a.Offset(CL_C, 70), -5);
    CHECK_EQ(a.Offset(-1, 70), 0);
  }
  // Clamp at +-max.
  c = AdaDefaults();
  c.maxOfs = 12;
  a.Configure(c);
  for (int k = 0; k < 5; k++) AdaShift(a, AdaGood(3), 3, 4, 900, 0);
  CHECK_EQ(a.Raw(CL_D, ADA_WARM), 12);

  // Pack / unpack round trip; corrupt image rejected.
  AdaImage img;
  a.Pack(&img);
  CHECK(!a.Dirty());
  CHECK_EQ(img.magic, ADA_MAGIC);
  Adapt b;
  CHECK(b.Unpack(img));
  CHECK_EQ(b.Raw(CL_D, ADA_WARM), 12);
  CHECK_EQ(b.Raw(CL_C, ADA_WARM), -5);
  CHECK_EQ(b.Shifts(), a.Shifts());
  AdaImage bad = img;
  bad.ofs[0][0] ^= 1;
  CHECK(!b.Unpack(bad));
  CHECK_EQ(b.Raw(CL_D, ADA_WARM), 0);
  AdaImage blank;
  memset(&blank, 0xFF, sizeof(blank)); // erased flash
  CHECK(!b.Unpack(blank));
  // Explicit reset.
  a.Reset();
  CHECK_EQ(a.Raw(CL_D, ADA_WARM), 0);
  CHECK_EQ(a.Shifts(), 0);
  CHECK(a.Dirty());
}

static void TestDiag() {
  // Expected resistance: 5.15 ohm at 20 degC + wiring, copper tempco.
  CHECK_EQ(Diag::ExpectedMilliOhm(20, 150), 5300);
  CHECK_EQ(Diag::ExpectedMilliOhm(120, 150), 5150 + 2023 + 150);
  CHECK_EQ(Diag::ExpectedMilliOhm(-30, 150), 4138 + 150);
  // 13.25 V / 2.5 A = 5.3 ohm: in band.
  CHECK_EQ(Diag::Resistance(13250, 2500, 20, 25, 150), DIAG_OK);
  CHECK_EQ(Diag::Resistance(13250, 1500, 20, 25, 150), DIAG_R_HIGH); // 8.8 ohm
  CHECK_EQ(Diag::Resistance(13250, 3500, 20, 25, 150), DIAG_R_LOW);  // 3.8 ohm
  CHECK_EQ(Diag::Resistance(1000, 2000, 20, 25, 150), DIAG_SHORT_GND); // 0.5 ohm
  CHECK_EQ(Diag::Resistance(13250, 10, 20, 25, 150), DIAG_OK); // no current
  // Hot coil reads higher but still OK.
  CHECK_EQ(Diag::Resistance(14000, 2000, 100, 25, 150), DIAG_OK); // 7.0 ohm vs 6.92
  // Flags win.
  CHECK_EQ(Diag::Channel(VALVE_F_OCP | VALVE_F_OLF, DIAG_OK), DIAG_SHORT_BAT);
  CHECK_EQ(Diag::Channel(VALVE_F_OLF, DIAG_OK), DIAG_OPEN);
  CHECK_EQ(Diag::Channel(VALVE_F_HHF, DIAG_OK), DIAG_R_HIGH);
  CHECK_EQ(Diag::Channel(VALVE_F_DPM, DIAG_OK), DIAG_OK);
  CHECK_EQ(Diag::Channel(0, DIAG_SHORT_GND), DIAG_SHORT_GND);
  // Pairs.
  uint8_t codes[8] = {0};
  CHECK_EQ(Diag::Pairs(codes, 8), 0);
  codes[2] = DIAG_OPEN;
  CHECK_EQ(Diag::Pairs(codes, 8), 0);
  codes[3] = DIAG_SHORT_BAT;
  codes[6] = codes[7] = DIAG_OPEN;
  CHECK_EQ(Diag::Pairs(codes, 8), 0x0A);
  CHECK_EQ(Diag::Pairs(codes, 7), 0x02);
  // Speed sensor.
  CHECK_EQ(Diag::Speed(1000, 2000, true, true, 3000, 200), SPD_OK);
  CHECK_EQ(Diag::Speed(3100, 3150, false, false, 3000, 200), SPD_OPEN);
  CHECK_EQ(Diag::Speed(10, 50, false, false, 3000, 200), SPD_SHORT);
  CHECK_EQ(Diag::Speed(1500, 1520, false, true, 3000, 200), SPD_NOSIGNAL);
  CHECK_EQ(Diag::Speed(1500, 1520, false, false, 3000, 200), SPD_OK); // standstill
  CHECK_EQ(Diag::Speed(3100, 3150, false, false, 0, 0), SPD_OK);       // level checks off
  // Version.
  CHECK(Diag::CalOk(0, 0x1234));
  CHECK(Diag::CalOk(0x1234, 0x1234));
  CHECK(!Diag::CalOk(0x1235, 0x1234));
  // Lock-up slip.
  SlipMonitor s;
  for (int i = 0; i < 100; i++) s.Tick10ms(true, 2000, 1800, 0, 500);
  CHECK(!s.Fault()); // off by default
  for (int i = 0; i < 40; i++) s.Tick10ms(true, 2000, 1800, 100, 500);
  CHECK(!s.Fault());
  s.Tick10ms(true, 2000, 2000, 100, 500); // back in: timer restarts
  for (int i = 0; i < 49; i++) s.Tick10ms(true, 2000, 1800, 100, 500);
  CHECK(!s.Fault());
  for (int i = 0; i < 3; i++) s.Tick10ms(true, 2000, 1800, 100, 500);
  CHECK(s.Fault());
  s.Tick10ms(false, 2000, 1800, 100, 500);
  CHECK(s.Fault()); // latched
  s.Clear();
  CHECK(!s.Fault());
  for (int i = 0; i < 100; i++) s.Tick10ms(false, 2000, 1500, 100, 500);
  CHECK(!s.Fault()); // open TCC slips by design
}

static TccConfig TccDefaults() {
  TccConfig c = {1100, 200, 10, 20, 1500, 6000, 40, 3, 1800};
  return c;
}

static void TestTcc() {
  // Schedule shape.
  CHECK(Tcc::OnRpm(8, 0) < Tcc::OnRpm(3, 0));
  CHECK(Tcc::OnRpm(5, 0) < Tcc::OnRpm(5, 100));
  CHECK_EQ(Tcc::OnRpm(5, 0), 1300);
  CHECK_EQ(Tcc::OnRpm(5, 50), 1700);

  TccConfig c = TccDefaults();
  Tcc t;
  // Below the on speed: open.
  CHECK_EQ(t.Tick10ms(c, true, 5, 0, 1250, 1400), 0);
  CHECK(!t.Closed());
  // At the on speed: closes, start pressure + P.
  int32_t p = t.Tick10ms(c, true, 5, 0, 1300, 1400);
  CHECK(t.Closed());
  CHECK(p > 1500);
  // Hysteresis: stays closed down to on - hyst.
  t.Tick10ms(c, true, 5, 0, 1150, 1150);
  CHECK(t.Closed());
  CHECK_EQ(t.Tick10ms(c, true, 5, 0, 1099, 1099), 0); // under min rpm
  CHECK(!t.Closed());
  t.Tick10ms(c, true, 6, 0, 1300, 1300);
  CHECK(t.Closed());
  t.Tick10ms(c, true, 6, 0, 1140, 1140); // 1200 - 200 = 1000 < 1140: stays closed
  CHECK(t.Closed());
  c.minRpm = 500;
  t.Tick10ms(c, true, 6, 0, 999, 999); // below on - hyst
  CHECK(!t.Closed());
  c = TccDefaults();
  // Not allowed (shift / N): opens at once.
  t.Tick10ms(c, true, 6, 0, 2000, 2050);
  CHECK(t.Closed());
  CHECK_EQ(t.Tick10ms(c, false, 6, 0, 2000, 2050), 0);
  CHECK(!t.Closed());
  // PI: slip held -> pressure rises over time, limited.
  int32_t p0 = t.Tick10ms(c, true, 6, 0, 2000, 2100);
  int32_t p1 = 0;
  for (int i = 0; i < 100; i++) p1 = t.Tick10ms(c, true, 6, 0, 2000, 2100);
  CHECK(p1 > p0);
  for (int i = 0; i < 5000; i++) p1 = t.Tick10ms(c, true, 6, 0, 2000, 2100);
  CHECK_EQ(p1, 6000);
  // Controlled slip target at low gear and rpm.
  CHECK_EQ(t.Target(c, 3, 1700), 40);
  CHECK_EQ(t.Target(c, 3, 1900), 0);
  CHECK_EQ(t.Target(c, 4, 1700), 0);
  c.slipTgt = 0;
  CHECK_EQ(t.Target(c, 3, 1700), 0);
  // At the slip target the error is 0: pressure = start + I only.
  c = TccDefaults();
  Tcc u;
  int32_t q = u.Tick10ms(c, true, 3, 0, 1700, 1740);
  CHECK_EQ(q, 1500);
}

static void TestStream() {
  CHECK_EQ(StatStream::ReductPct(200, -50), 25);
  CHECK_EQ(StatStream::ReductPct(200, 0), 0);
  CHECK_EQ(StatStream::ReductPct(200, 30), 0);
  CHECK_EQ(StatStream::ReductPct(0, -30), 0);
  CHECK_EQ(StatStream::ReductPct(100, -300), 100);

  StatStream s;
  StreamIn in;
  memset(&in, 0, sizeof(in));
  in.tqTarget = -120;
  in.tqReductPct = 30;
  in.shifting = true;
  in.tqActive = true;
  in.tccClosed = true;
  in.slip = -250;
  in.nT = 2500; in.nAb = 1000; in.nMot = 2510; in.nAux = 0;
  in.gear = 4; in.target = 5; in.selector = 3; in.mode = 3;
  in.fault = 0; in.diagBits = 0x0102; in.atfC = 85; in.boardC = -10;
  in.vbatMv = 13800; in.adaStatus = 0x40; in.tqNotCan = true;
  CanFrame f[STREAM_MAX_TX];
  int n = s.Tick10ms(0x6C0, in, f);
  CHECK_EQ(n, 4); // t = 0: all four
  CHECK_EQ(f[0].id, 0x6C0u);
  CHECK_EQ(f[0].data[0], 0x88);
  CHECK_EQ(f[0].data[1], 0xFF);
  CHECK_EQ(f[0].data[2], 30);
  CHECK_EQ(f[0].data[3], 0x15);
  CHECK_EQ((int16_t)(f[0].data[4] | (f[0].data[5] << 8)), -250);
  CHECK_EQ(f[0].data[6], 0);
  CHECK_EQ(f[1].id, 0x6C1u);
  CHECK_EQ(f[1].data[0] | (f[1].data[1] << 8), 2500);
  CHECK_EQ(f[1].data[2] | (f[1].data[3] << 8), 1000);
  CHECK_EQ(f[1].data[4] | (f[1].data[5] << 8), 2510);
  CHECK_EQ(f[2].id, 0x6C2u);
  CHECK_EQ(f[2].data[0], 4);
  CHECK_EQ(f[2].data[1], 5);
  CHECK_EQ(f[3].id, 0x6C3u);
  CHECK_EQ(f[3].data[1], 0x02);
  CHECK_EQ(f[3].data[2], 0x01);
  CHECK_EQ(f[3].data[3], 125);
  CHECK_EQ(f[3].data[4], 30);
  CHECK_EQ(f[3].data[5], 138);
  CHECK_EQ(f[3].data[6], 0x40);
  CHECK_EQ(f[3].data[7], 4);
  // Rates over 1 s: 100 / 100 / 40 / 20.
  int cnt[4] = {0, 0, 0, 0};
  for (int i = 1; i <= 100; i++) {
    n = s.Tick10ms(0x6C0, in, f);
    for (int k = 0; k < n; k++) cnt[f[k].id - 0x6C0]++;
  }
  CHECK_EQ(cnt[0], 100);
  CHECK_EQ(cnt[1], 100);
  CHECK_EQ(cnt[2], 40);
  CHECK_EQ(cnt[3], 20);
  CHECK_EQ(f[0].data[6], 100); // alive counter
}

// CAL_SIG16 must match the generated tables (run make cal after editing).
static void TestCalSig() {
  const char *files[3] = {"../src/cal_330e.cpp", "../src/cal_f11.cpp", "../src/cal_v095.cpp"};
  uint16_t crc = 0xFFFF;
  bool ok = true;
  for (const char *fn : files) {
    FILE *f = fopen(fn, "rb");
    if (!f) {
      ok = false;
      continue;
    }
    int c;
    while ((c = fgetc(f)) != EOF) {
      crc ^= (uint16_t)(c << 8);
      for (int k = 0; k < 8; k++)
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    fclose(f);
  }
  CHECK(ok);
  CHECK_EQ(crc, CAL_SIG16);
  CHECK(Diag::CalOk(0, CAL_SIG16));
  CHECK(Diag::CalOk(CAL_SIG16, CAL_SIG16));
}

void TestFeatures() {
  TestCalSig();
  TestTorqueIn();
  TestAdapt();
  TestDiag();
  TestTcc();
  TestStream();
}
