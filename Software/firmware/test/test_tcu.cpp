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
#include "gears.h"
#include "tcu.h"
#include "test.h"

static TcuConfig Cfg() {
  TcuConfig c;
  c.sas.pMax = 12750;
  c.sas.syncBand = 30;
  c.sas.glsGainPct = 100;
  c.sas.rampMaxDownMs = 1000;
  c.engage.pMax = 12750;
  c.engage.fillEqMs = 50;
  c.engage.rampMs = 300;
  c.engage.syncBand = 30;
  c.engage.syncCloseGrad = 20000;
  c.gearSelMode = GSEL_VCU;
  c.applyP = 16500;
  c.lineOffset = 0;
  c.lineMax = 25000;
  c.engageMaxNab = 300;
  c.autoUpRpm = 4000;
  c.autoDownRpm = 1500;
  c.uvLimit = 7400;
  c.gearCheckPct = 10;
  c.gearCheckMinNab = 200;
  c.gearCheckMs = 500;
  c.tqRampMs = 100;
  c.dntTauMs = 70;
  c.topGear = 8;
  c.tqInterv = 1;
  return c;
}

static TcuInputs In() {
  TcuInputs in = {};
  in.m_t = 100;
  in.temp = 60;
  in.vbat = 13000;
  in.selector = SEL_N;
  in.canOk = true;
  in.speedsValid = true;
  return in;
}

/// Run ticks; the model moves n_t to the target gear's sync during the
/// inertia phase of a shift. Returns false if the TCU went to safe.
static bool RunTicks(Tcu &t, TcuInputs &in, int ticks) {
  for (int i = 0; i < ticks; i++) {
    t.Tick10ms(in);
    const TcuOutputs &o = t.Out();
    if (o.mode == TCU_SAFE)
      return false;
    // Once the inertia phase starts the speed carries on to the new sync.
    static bool moving = false;
    if (o.mode != TCU_SHIFT)
      moving = false;
    else if (o.zKzu == 10 || o.zKzu == 13 || o.zKab == 9 || o.zKab == 12)
      moving = true;
    if (moving) {
      int32_t target = Gears::SyncSpeed(o.target, in.n_ab);
      int32_t step = 30;
      if (in.n_t < target)
        in.n_t = in.n_t + step > target ? target : in.n_t + step;
      else
        in.n_t = in.n_t - step < target ? target : in.n_t - step;
    } else if (o.mode == TCU_DRIVE) {
      in.n_t = Gears::SyncSpeed(o.gear, in.n_ab);
    }
    in.n_mot = in.n_t;
  }
  return true;
}

static void TestEngageAndShift() {
  Tcu t;
  t.Configure(Cfg());
  TcuInputs in = In();
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_INIT);
  CHECK(t.Out().safeOutputs);
  t.Ready();
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  // Neutral: only B gets pressure (pre-filled, below its apply level)
  CHECK(t.Out().p[CL_B] > 0);
  CHECK_EQ(t.Out().p[CL_A] + t.Out().p[CL_C] + t.Out().p[CL_D] + t.Out().p[CL_E], 0);
  CHECK(t.Out().pSys > 0);

  // N -> D at standstill
  in.selector = SEL_D;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_ENGAGE);
  CHECK(RunTicks(t, in, 500));
  CHECK_EQ(t.Out().mode, TCU_DRIVE);
  CHECK_EQ(t.Out().gear, GEAR_1);
  CHECK_EQ(t.Out().applied, Gears::ClutchMask(GEAR_1));

  // Drive away and ask for 3rd: goes 1 -> 2 -> 3
  in.n_ab = 1000;
  in.n_t = Gears::SyncSpeed(1, 1000);
  in.gearReq = 3;
  bool saw2 = false;
  for (int i = 0; i < 600 && t.Out().gear != 3; i++) {
    CHECK(RunTicks(t, in, 1));
    if (t.Out().gear == 2)
      saw2 = true;
  }
  CHECK(saw2);
  CHECK_EQ(t.Out().gear, 3);
  CHECK_EQ(t.Out().mode, TCU_DRIVE);
  CHECK_EQ(t.Out().fault, TF_NONE);
  CHECK_EQ(t.Out().tqDelta, 0); // ramped out after the shift
  CHECK(RunTicks(t, in, 20));

  // Back down to 2nd
  in.gearReq = 2;
  for (int i = 0; i < 400 && t.Out().gear != 2; i++)
    CHECK(RunTicks(t, in, 1));
  CHECK_EQ(t.Out().gear, 2);

  // Top gear limit
  TcuConfig c = Cfg();
  c.topGear = 2;
  t.Configure(c);
  in.gearReq = 8;
  CHECK(RunTicks(t, in, 100));
  CHECK_EQ(t.Out().gear, 2);
  CHECK_EQ(t.Out().mode, TCU_DRIVE);
  t.Configure(Cfg());

  // No CAN: no shifts
  in.canOk = false;
  in.gearReq = 3;
  CHECK(RunTicks(t, in, 50));
  CHECK_EQ(t.Out().gear, 2);
  in.canOk = true;

  // D -> N releases everything
  in.selector = SEL_N;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  CHECK_EQ(t.Out().gear, GEAR_N);

  // No engage above EngageMaxRpm output speed
  in.selector = SEL_D;
  CHECK(RunTicks(t, in, 10));
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
}

static void TestReverse() {
  Tcu t;
  t.Configure(Cfg());
  TcuInputs in = In();
  t.Ready();
  in.selector = SEL_R;
  CHECK(RunTicks(t, in, 500));
  CHECK_EQ(t.Out().gear, GEAR_R);
  CHECK_EQ(t.Out().applied, Gears::ClutchMask(GEAR_R));
  // R -> D goes through neutral
  in.selector = SEL_D;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  CHECK(RunTicks(t, in, 500));
  CHECK_EQ(t.Out().gear, GEAR_1);
}

static void TestFaults() {
  Tcu t;
  t.Configure(Cfg());
  TcuInputs in = In();
  t.Ready();
  in.selector = SEL_D;
  CHECK(RunTicks(t, in, 500));
  CHECK_EQ(t.Out().gear, GEAR_1);

  // Driver fault: safe state, every pressure 0 (all coils off = 6th)
  in.driverFault = true;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_SAFE);
  CHECK_EQ(t.Out().fault, TF_DRIVER);
  CHECK(t.Out().safeOutputs);
  CHECK_EQ(t.Out().gear, GEAR_LIMP);
  for (int c = 0; c < CL_COUNT; c++)
    CHECK_EQ(t.Out().p[c], 0);
  // Clear refused while the fault is present or not in N/P
  in.faultClear = true;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_SAFE);
  in.driverFault = false;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_SAFE); // still in D
  in.selector = SEL_N;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  CHECK_EQ(t.Out().fault, TF_NONE);
  in.faultClear = false;

  // Under-voltage
  in.vbat = 7000;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().fault, TF_UNDERVOLT);
  in.vbat = 13000;
  in.faultClear = true;
  t.Tick10ms(in);
  in.faultClear = false;
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);

  // Gear ratio check: in 1st but input speed says otherwise
  in.selector = SEL_D;
  CHECK(RunTicks(t, in, 500));
  CHECK_EQ(t.Out().gear, GEAR_1);
  in.n_ab = 500;
  in.n_t = 500; // ratio 1.0 in 1st = slipping
  for (int i = 0; i < 60; i++)
    t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_SAFE);
  CHECK_EQ(t.Out().fault, TF_SPEED);
}

static void TestNoTorqueIntervention() {
  Tcu t;
  TcuConfig c = Cfg();
  c.tqInterv = 0;
  t.Configure(c);
  TcuInputs in = In();
  t.Ready();
  in.selector = SEL_D;
  CHECK(RunTicks(t, in, 500));
  in.n_ab = 1000;
  in.n_t = Gears::SyncSpeed(1, 1000);
  in.gearReq = 2;
  for (int i = 0; i < 400 && t.Out().gear != 2; i++) {
    CHECK(RunTicks(t, in, 1));
    CHECK_EQ(t.Out().tqDelta, 0);
  }
  CHECK_EQ(t.Out().gear, 2);
}

void TestTcu() {
  TestEngageAndShift();
  TestReverse();
  TestFaults();
  TestNoTorqueIntervention();
}
