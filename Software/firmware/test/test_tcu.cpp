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
#include "calset.h"
#include <string.h>
#include "caltypes.h"
#include "eds.h"
#include "engage.h"
#include "gears.h"
#include "tcu.h"
#include "test.h"
#include "drivemode.h"

static TcuConfig Cfg() {
  TcuConfig c = {};
  c.sas.pMax = 12750;
  c.sas.syncBand = 30;
  c.sas.glsGainPct = 100;
  c.sas.rampMaxDownMs = 1000;
  c.engage.pMax = 12750;
  c.engage.fillEqMs = 50;
  c.engage.rampMs = 300;
  c.engage.syncBand = 30;
  c.engage.fastCloseGrad = 30000;
  c.engage.syncConfirmMs = 80;
  c.engage.tqZeroNm = 10;
  c.engage.variant = ENG_VAR_330E;
  c.shiftMode = SHIFT_VCU;
  c.manMaxRpm = 6000;
  c.manMinRpm = 1000;
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
  c.canLossMode = CANLOSS_HOLD; // the parameter default
  c.ilOpenLow = 600;
  c.ilOpenHigh = 7900;
  c.iMax = 8500;
  c.tempCorrection = true;
  // Ian 11/12 defaults, with the two waits that need the motor or pump
  // turned off (the older tests run with the motor at 0 rpm in N).
  c.parkRelRpm = 0;
  c.parkRelP = 17000;
  c.parkRelHoldMs = 2000;
  c.pnDelayMs = 150;
  c.parkCtrl = false;
  c.pzylI = 5500;
  c.parkMagMinMs = 100;
  c.parkMagMaxMs = 1000;
  c.gearInMinP = 8000;
  c.gearInPumpMs = 0;
  c.gearInMotRpm = 0;
  c.pumpFaultAct = PUMPFAULT_REPORT;
  c.pumpFaultRpm = 600;
  c.tipOverrideMs = 0;
  c.ch7Mode = CH7_K0;
  c.k0P = 0;
  c.tccP = 6000;
  c.tccOnRpm = 1500;
  c.tccOffRpm = 1200;
  c.tccMinGear = 2;
  c.tccRamp = 500;
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


/// Gear-in the way a VCU would do it: hold the motor torque at 0 while the
/// TCU asks for it (tqHold), then give it back. Returns true once in gear.
static bool EngageTo(Tcu &t, TcuInputs &in, uint8_t sel) {
  int32_t tq = in.m_t;
  in.selector = sel;
  bool held = false;
  for (int i = 0; i < 500 && t.Out().mode != TCU_DRIVE; i++) {
    if (!RunTicks(t, in, 1))
      return false;
    if (t.Out().tqHold) {
      held = true;
      in.m_t = 0;
    }
  }
  in.m_t = tq;
  CHECK(held);
  CHECK(!t.Out().tqHold);
  return t.Out().mode == TCU_DRIVE;
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
  CHECK(t.Out().tqHold);
  CHECK(EngageTo(t, in, SEL_D));
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
  CHECK(EngageTo(t, in, SEL_R));
  CHECK_EQ(t.Out().gear, GEAR_R);
  CHECK_EQ(t.Out().applied, Gears::ClutchMask(GEAR_R));
  // R -> D goes through neutral
  in.selector = SEL_D;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  CHECK(EngageTo(t, in, SEL_D));
  CHECK_EQ(t.Out().gear, GEAR_1);
}

static void TestFaults() {
  Tcu t;
  t.Configure(Cfg());
  TcuInputs in = In();
  t.Ready();
  CHECK(EngageTo(t, in, SEL_D));
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
  CHECK(EngageTo(t, in, SEL_D));
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
  CHECK(EngageTo(t, in, SEL_D));
  in.n_ab = 1000;
  in.n_t = Gears::SyncSpeed(1, 1000);
  in.gearReq = 2;
  for (int i = 0; i < 400 && t.Out().gear != 2; i++) {
    CHECK(RunTicks(t, in, 1));
    CHECK_EQ(t.Out().tqDelta, 0);
  }
  CHECK_EQ(t.Out().gear, 2);
}


/* Without torque held near 0 the gear-in still completes, on the slower
 * OEM gradients, and never uses the fast close. */
static void TestEngageNoTorqueHold() {
  Tcu t;
  t.Configure(Cfg());
  TcuInputs in = In(); // m_t = 100 Nm throughout
  t.Ready();
  in.selector = SEL_D;
  int fastTicks = 0, ms = 0;
  for (; ms < 30000 && t.Out().mode != TCU_DRIVE; ms += 10) {
    CHECK(RunTicks(t, in, 1));
    if (t.Out().engPhase == ENG_CLOSE && t.Out().p[CL_B] > 0 &&
        t.Out().p[CL_B] < 12750)
      fastTicks++;
  }
  CHECK_EQ(t.Out().gear, GEAR_1);
  CHECK(ms > 1500); // OEM low-torque gradients are slow

  // No gear-in while the VCU frames are missing.
  in.selector = SEL_N;
  CHECK(RunTicks(t, in, 2));
  in.canOk = false;
  in.selector = SEL_D;
  CHECK(RunTicks(t, in, 20));
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  (void)fastTicks;
}

/* F11 variant: C is the last clutch in for 1st. */
static void TestEngageF11() {
  Tcu t;
  TcuConfig c = Cfg();
  c.engage.variant = ENG_VAR_F11;
  t.Configure(c);
  TcuInputs in = In();
  t.Ready();
  in.selector = SEL_D;
  bool cWaited = false;
  for (int i = 0; i < 500 && t.Out().mode != TCU_DRIVE; i++) {
    in.m_t = t.Out().tqHold ? 0 : in.m_t;
    CHECK(RunTicks(t, in, 1));
    if (t.Out().engPhase == ENG_FILL && t.Out().p[CL_C] == 0 && t.Out().p[CL_A] > 0)
      cWaited = true;
  }
  CHECK(cWaited);
  CHECK_EQ(t.Out().gear, GEAR_1);
}

/* CanLossMode: 0 drops the shift requests (the glue then selects N);
 * 1 holds the gear at full line pressure, refuses upshifts, still steps
 * down as the car slows. */
static void TestCanLoss() {
  Tcu t;
  TcuConfig c = Cfg();
  CHECK_EQ(c.canLossMode, CANLOSS_HOLD); // default
  t.Configure(c);
  TcuInputs in = In();
  t.Ready();
  CHECK(EngageTo(t, in, SEL_D));
  in.n_ab = 1000;
  in.n_t = Gears::SyncSpeed(1, 1000);
  in.gearReq = 4;
  for (int i = 0; i < 1500 && t.Out().gear != 4; i++)
    CHECK(RunTicks(t, in, 1));
  CHECK_EQ(t.Out().gear, 4);
  CHECK(RunTicks(t, in, 20));
  CHECK(!t.Out().canLossHold);

  // Frames stop at speed: hold 4th with LineMax, no upshift even if asked.
  in.canOk = false;
  in.gearReq = 8;
  in.n_ab = 1500;
  in.n_t = Gears::SyncSpeed(4, in.n_ab);
  CHECK(RunTicks(t, in, 100));
  CHECK_EQ(t.Out().gear, 4);
  CHECK_EQ(t.Out().mode, TCU_DRIVE);
  CHECK(t.Out().canLossHold);
  CHECK_EQ(t.Out().pSys, c.lineMax);
  // Slowing down: downshifts are still allowed so it does not lug.
  in.n_ab = 300;
  in.n_t = Gears::SyncSpeed(4, in.n_ab);
  for (int i = 0; i < 2000 && t.Out().gear > 1; i++)
    CHECK(RunTicks(t, in, 1));
  CHECK(t.Out().gear < 4);
  CHECK_EQ(t.Out().fault, TF_NONE);
  // Frames back: normal again.
  in.canOk = true;
  CHECK(RunTicks(t, in, 5));
  CHECK(!t.Out().canLossHold);

  // Mode 0 (neutral option): no shifts and no hold flag.
  Tcu t0;
  TcuConfig c0 = Cfg();
  c0.canLossMode = CANLOSS_NEUTRAL;
  t0.Configure(c0);
  TcuInputs i0 = In();
  t0.Ready();
  CHECK(EngageTo(t0, i0, SEL_D));
  i0.n_ab = 1000;
  i0.n_t = Gears::SyncSpeed(1, 1000);
  i0.canOk = false;
  i0.gearReq = 3;
  CHECK(RunTicks(t0, i0, 50));
  CHECK_EQ(t0.Out().gear, GEAR_1);
  CHECK(!t0.Out().canLossHold);
}

/* A coast upshift (negative torque) runs to the end with the slip
 * controller on the off-going clutch and is reported as coast. */
static void TestCoastShift() {
  Tcu t;
  t.Configure(Cfg());
  TcuInputs in = In();
  t.Ready();
  CHECK(EngageTo(t, in, SEL_D));
  in.n_ab = 1000;
  in.n_t = Gears::SyncSpeed(1, 1000);
  in.m_t = -40;
  in.gearReq = 2;
  bool sawCoast = false;
  for (int i = 0; i < 600 && t.Out().gear != 2; i++) {
    CHECK(RunTicks(t, in, 1));
    if (t.Out().coast)
      sawCoast = true;
  }
  CHECK(sawCoast);
  CHECK_EQ(t.Out().gear, 2);
  CHECK_EQ(t.Out().fault, TF_NONE);
  CHECK(!t.Out().coast);
  // And a coast downshift back to 1st.
  in.gearReq = 1;
  for (int i = 0; i < 600 && t.Out().gear != 1; i++)
    CHECK(RunTicks(t, in, 1));
  CHECK_EQ(t.Out().gear, 1);
  CHECK_EQ(t.Out().fault, TF_NONE);
}

/// Output speed that puts the input at rpm in gear g.
static int32_t NabFor(int g, int32_t rpm) {
  return (int32_t)(((int64_t)rpm * 1000) / Gears::RatioX1000(g));
}

/// Run until the TCU is back in DRIVE in some gear (or limit ticks).
static bool Settle(Tcu &t, TcuInputs &in, int limit = 400) {
  for (int i = 0; i < limit; i++) {
    if (!RunTicks(t, in, 1))
      return false;
    if (t.Out().mode == TCU_DRIVE && i > 2)
      return true;
  }
  return t.Out().mode == TCU_DRIVE;
}

/* Manual mode: one gear per tip, rpm limits, lugging protection. */
static void TestManual() {
  Tcu t;
  TcuConfig c = Cfg();
  c.shiftMode = SHIFT_MANUAL;
  t.Configure(c);
  TcuInputs in = In();
  t.Ready();
  t.Tick10ms(in);
  CHECK(EngageTo(t, in, SEL_D));
  CHECK_EQ(t.Out().gear, GEAR_1);

  // High input speed in 1st: Auto would upshift, Manual holds the gear.
  in.n_ab = NabFor(1, 5000);
  in.n_t = 5000;
  in.gearReq = 8; // the VCU field is ignored in Manual
  CHECK(RunTicks(t, in, 50));
  CHECK_EQ(t.Out().gear, 1);
  CHECK_EQ(t.Out().mode, TCU_DRIVE);

  // Tip up: one gear
  in.tipUp = true;
  CHECK(RunTicks(t, in, 1));
  in.tipUp = false;
  CHECK_EQ(t.Out().mode, TCU_SHIFT);
  CHECK_EQ(t.Out().target, 2);
  CHECK(Settle(t, in));
  CHECK_EQ(t.Out().gear, 2);
  CHECK(RunTicks(t, in, 50));
  CHECK_EQ(t.Out().gear, 2); // no repeat without a new tip

  // Tip down that would over-rev the input (1st above ManMaxRpm): refused
  in.n_ab = NabFor(1, 6500);
  CHECK(RunTicks(t, in, 2));
  in.tipDown = true;
  CHECK(RunTicks(t, in, 1));
  in.tipDown = false;
  CHECK(RunTicks(t, in, 50));
  CHECK_EQ(t.Out().gear, 2);
  CHECK_EQ(t.Out().mode, TCU_DRIVE);

  // Tip down inside the limit: accepted
  in.n_ab = NabFor(1, 4000);
  CHECK(RunTicks(t, in, 2));
  in.tipDown = true;
  CHECK(RunTicks(t, in, 1));
  in.tipDown = false;
  CHECK_EQ(t.Out().target, 1);
  CHECK(Settle(t, in));
  CHECK_EQ(t.Out().gear, 1);

  // Tip up that would lug the input (2nd below ManMinRpm): refused
  in.n_ab = NabFor(2, 800);
  CHECK(RunTicks(t, in, 2));
  in.tipUp = true;
  CHECK(RunTicks(t, in, 1));
  in.tipUp = false;
  CHECK(RunTicks(t, in, 50));
  CHECK_EQ(t.Out().gear, 1);

  // Top gear limit still applies
  c.topGear = 1;
  t.Configure(c);
  in.n_ab = NabFor(1, 4000);
  in.tipUp = true;
  CHECK(RunTicks(t, in, 1));
  in.tipUp = false;
  CHECK(RunTicks(t, in, 20));
  CHECK_EQ(t.Out().gear, 1);
  c.topGear = 8;
  t.Configure(c);

  // Climb to 3rd with tips, then slow down without tipping: the
  // lugging protection steps down on its own.
  for (int g = 1; g < 3; g++) {
    in.n_ab = NabFor(g, 4500);
    CHECK(RunTicks(t, in, 2));
    in.tipUp = true;
    CHECK(RunTicks(t, in, 1));
    in.tipUp = false;
    CHECK(Settle(t, in));
  }
  CHECK_EQ(t.Out().gear, 3);
  in.n_ab = NabFor(3, 1200); // below AutoDownRpm (1500) in 3rd
  bool down = false;
  for (int i = 0; i < 400 && !down; i++) {
    CHECK(RunTicks(t, in, 1));
    down = t.Out().gear < 3;
  }
  CHECK(down);
  CHECK_EQ(t.Out().fault, TF_NONE);

  // The interlock still guards Manual: never more than 3 clutches in
  // gear, 4 during a swap (checked on every tick of the runs above by
  // the TCU itself; here just confirm no interlock fault was raised).
  CHECK(t.Out().interlock == 0);
}

/* Auto mode picks gears on its own thresholds. */
static void TestAuto() {
  Tcu t;
  TcuConfig c = Cfg();
  c.shiftMode = SHIFT_AUTO;
  t.Configure(c);
  TcuInputs in = In();
  t.Ready();
  t.Tick10ms(in);
  CHECK(EngageTo(t, in, SEL_D));
  in.n_ab = NabFor(1, 4500); // above AutoUpRpm in 1st
  in.tipDown = true;          // tips are ignored in Auto
  bool up = false;
  for (int i = 0; i < 400 && !up; i++) {
    CHECK(RunTicks(t, in, 1));
    up = t.Out().gear >= 2;
  }
  CHECK(up);
}

/* ManualGear dropdown: the TCU steps to the picked gear one gear at a time,
 * each step inside the tip limits; gear-in goes through the usual engage. */
static void TestManualGear() {
  Tcu t;
  TcuConfig c = Cfg();
  c.shiftMode = SHIFT_MANUAL;
  t.Configure(c);
  TcuInputs in = In();
  t.Ready();
  t.Tick10ms(in);
  in.manualGear = 4;
  CHECK(EngageTo(t, in, SEL_D));
  CHECK_EQ(t.Out().gear, GEAR_1); // gear-in is always to 1st
  // Too slow for 2nd (below ManMinRpm): waits in 1st
  in.n_ab = NabFor(2, 800);
  CHECK(RunTicks(t, in, 50));
  CHECK_EQ(t.Out().gear, 1);
  // Fast enough: climbs to 4th and stops there
  for (int i = 0; i < 2000 && !(t.Out().gear == 4 && t.Out().mode == TCU_DRIVE); i++) {
    int g = t.Out().mode == TCU_DRIVE ? t.Out().gear : t.Out().target;
    if (t.Out().mode == TCU_DRIVE)
      in.n_ab = NabFor(g, 4500);
    CHECK(RunTicks(t, in, 1));
  }
  CHECK_EQ(t.Out().gear, 4);
  CHECK(RunTicks(t, in, 50));
  CHECK_EQ(t.Out().gear, 4);
  // Pick 2nd while 2nd would over-rev: holds 4th, then 3rd is fine
  in.manualGear = 2;
  in.n_ab = NabFor(3, 6500);
  CHECK(RunTicks(t, in, 30));
  CHECK_EQ(t.Out().gear, 4);
  // Tips are ignored while a gear is picked
  in.tipUp = true;
  CHECK(RunTicks(t, in, 1));
  in.tipUp = false;
  CHECK_EQ(t.Out().gear, 4);
  CHECK_EQ(t.Out().mode, TCU_DRIVE);
  in.n_ab = NabFor(2, 3000);
  for (int i = 0; i < 2000 && !(t.Out().gear == 2 && t.Out().mode == TCU_DRIVE); i++)
    CHECK(RunTicks(t, in, 1));
  CHECK_EQ(t.Out().gear, 2);
  // Above the output speed limit no gear-in from N
  CHECK(RunTicks(t, in, 1));
  in.selector = SEL_N;
  CHECK(RunTicks(t, in, 5));
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  in.selector = SEL_D;
  CHECK(RunTicks(t, in, 20));
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL); // n_ab above EngageMaxRpm
  CHECK_EQ(t.Out().fault, TF_NONE);
  CHECK(t.Out().interlock == 0);
}

/* Auto with tip override: a tip shifts one gear and holds Auto off for
 * TipOverride ms; VCU mode ignores tips. */
static void TestTipOverride() {
  Tcu t;
  TcuConfig c = Cfg();
  c.shiftMode = SHIFT_AUTO;
  c.tipOverrideMs = 2000;
  t.Configure(c);
  TcuInputs in = In();
  t.Ready();
  t.Tick10ms(in);
  CHECK(EngageTo(t, in, SEL_D));
  in.n_ab = NabFor(1, 3000); // below AutoUpRpm: Auto stays in 1st
  CHECK(RunTicks(t, in, 20));
  CHECK_EQ(t.Out().gear, 1);
  in.tipUp = true;
  CHECK(RunTicks(t, in, 1));
  in.tipUp = false;
  CHECK(t.Out().tipAuto);
  CHECK(Settle(t, in));
  CHECK_EQ(t.Out().gear, 2);
  // High speed in 2nd: Auto would upshift, the override holds 2nd
  in.n_ab = NabFor(2, 4500);
  CHECK(RunTicks(t, in, 100));
  CHECK_EQ(t.Out().gear, 2);
  // Override runs out: Auto upshifts again
  bool up = false;
  for (int i = 0; i < 400 && !up; i++) {
    CHECK(RunTicks(t, in, 1));
    up = t.Out().gear >= 3;
  }
  CHECK(up);
  CHECK(!t.Out().tipAuto);

  // VCU mode: tips do nothing
  Tcu v;
  c.shiftMode = SHIFT_VCU;
  v.Configure(c);
  TcuInputs vi = In();
  v.Ready();
  v.Tick10ms(vi);
  CHECK(EngageTo(v, vi, SEL_D));
  vi.n_ab = NabFor(1, 4500);
  vi.tipUp = true;
  CHECK(RunTicks(v, vi, 1));
  vi.tipUp = false;
  CHECK(RunTicks(v, vi, 50));
  CHECK_EQ(v.Out().gear, 1);
  CHECK(!v.Out().tipAuto);
}

/* Park lock (Ian 11 Q9d / 12.1): release only with the motor at 700 rpm or
 * 17 bar measured, P -> N 150 ms, motor speed held 2 s after, N -> P delay
 * over temperature, PZYL and magnet with ParkCtrl. */
static void TestPark() {
  Tcu t;
  TcuConfig c = Cfg();
  c.parkRelRpm = 700;
  c.parkCtrl = true;
  t.Configure(c);
  TcuInputs in = In();
  in.temp = 40;
  in.pMeas = -1;
  in.selector = SEL_P;
  t.Ready();
  CHECK(RunTicks(t, in, 14)); // N -> P at 40 degC: 150 ms
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  CHECK(RunTicks(t, in, 2));
  CHECK_EQ(t.Out().mode, TCU_PARK);
  CHECK_EQ(t.Out().pzylI, 0);
  CHECK(!t.Out().parkMagnet);
  // Selector to N with the motor stopped: wait, ask for 700 rpm
  in.selector = SEL_N;
  in.n_mot = 0;
  for (int i = 0; i < 50; i++) {
    t.Tick10ms(in);
    in.n_mot = 0;
  }
  CHECK_EQ(t.Out().mode, TCU_PARK);
  CHECK(t.Out().parkWait);
  CHECK_EQ(t.Out().nMotReq, 700);
  CHECK_EQ(t.Out().pSys, 17000); // line held at P_SYSMINPSAUS
  // Motor up: release takes 150 ms with PZYL on, then N
  in.n_mot = 720;
  for (int i = 0; i < 14; i++)
    t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_PARK);
  CHECK_EQ(t.Out().pzylI, 5500); // 550 mA, no cold offset at 40 degC
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  CHECK(t.Out().parkMagnet);
  CHECK_EQ(t.Out().nMotReq, 700);
  // Magnet off after 1 s, motor request after 2 s
  in.n_mot = 0;
  for (int i = 0; i < 100; i++)
    t.Tick10ms(in);
  CHECK(!t.Out().parkMagnet);
  CHECK_EQ(t.Out().nMotReq, 700);
  CHECK_EQ(t.Out().pzylI, 5500);
  for (int i = 0; i < 100; i++)
    t.Tick10ms(in);
  CHECK_EQ(t.Out().nMotReq, 0);
  // Back to P cold: N -> P 1000 ms at -30, PZYL off in P
  in.temp = -30;
  in.selector = SEL_P;
  for (int i = 0; i < 99; i++)
    t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  CHECK_EQ(t.Out().pzylI, 5500 + 1900); // cold offset +190 mA
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_PARK);
  CHECK_EQ(t.Out().pzylI, 0);
  // Measured pressure alone also releases
  in.selector = SEL_R;
  in.pMeas = 17500;
  for (int i = 0; i < 16; i++)
    t.Tick10ms(in);
  CHECK(t.Out().mode != TCU_PARK);
  // ParkCtrl off: no PZYL, no magnet, the wait still applies
  Tcu u;
  c.parkCtrl = false;
  u.Configure(c);
  TcuInputs ui = In();
  ui.pMeas = -1;
  ui.selector = SEL_P;
  u.Ready();
  for (int i = 0; i < 30; i++)
    u.Tick10ms(ui);
  ui.selector = SEL_D;
  for (int i = 0; i < 30; i++)
    u.Tick10ms(ui);
  CHECK_EQ(u.Out().mode, TCU_PARK);
  CHECK_EQ(u.Out().pzylI, 0);
  ui.n_mot = 800;
  for (int i = 0; i < 16; i++)
    u.Tick10ms(ui);
  CHECK(u.Out().mode != TCU_PARK);
  CHECK(!u.Out().parkMagnet);
}

/* Gear-in source (11 Q9d [I]) and the pump fault reactions. */
static void TestGearInSource() {
  Tcu t;
  TcuConfig c = Cfg();
  c.gearInPumpMs = 1000;
  c.gearInMotRpm = 100;
  t.Configure(c);
  TcuInputs in = In();
  t.Ready();
  t.Tick10ms(in);
  in.selector = SEL_D;
  for (int i = 0; i < 20; i++)
    t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  CHECK(t.Out().gearInWait);
  in.pumpDeliverMs = 999;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  in.pumpDeliverMs = 1000;
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_ENGAGE);
  CHECK(!t.Out().gearInWait);
  // Motor turning is enough on its own
  Tcu m;
  m.Configure(c);
  TcuInputs mi = In();
  m.Ready();
  m.Tick10ms(mi);
  mi.selector = SEL_R;
  mi.n_mot = 150;
  m.Tick10ms(mi);
  CHECK_EQ(m.Out().mode, TCU_ENGAGE);

  // Pump fault, Motor: ask for 600 rpm in N; Stop: no gear-in
  Tcu f;
  c.gearInPumpMs = c.gearInMotRpm = 0;
  c.pumpFaultAct = PUMPFAULT_MOTOR;
  f.Configure(c);
  TcuInputs fi = In();
  fi.pumpFault = true;
  f.Ready();
  f.Tick10ms(fi);
  CHECK_EQ(f.Out().nMotReq, 600);
  c.pumpFaultAct = PUMPFAULT_STOP;
  f.Configure(c);
  fi.selector = SEL_D;
  for (int i = 0; i < 10; i++)
    f.Tick10ms(fi);
  CHECK_EQ(f.Out().mode, TCU_NEUTRAL);
  CHECK_EQ(f.Out().nMotReq, 0);
  fi.pumpFault = false;
  f.Tick10ms(fi);
  CHECK_EQ(f.Out().mode, TCU_ENGAGE);
  c.pumpFaultAct = PUMPFAULT_REPORT;
  f.Configure(c);
  fi.pumpFault = true;
  for (int i = 0; i < 5; i++)
    f.Tick10ms(fi);
  CHECK_EQ(f.Out().nMotReq, 0);
  CHECK(f.Out().mode != TCU_SAFE);
}

/* Channel 7: K0 held at K0Press, TCC lock-up only in a steady gear,
 * force-open always 0. */
static void TestCh7() {
  Tcu t;
  TcuConfig c = Cfg();
  c.ch7Mode = CH7_TCC;
  c.shiftMode = SHIFT_MANUAL;
  c.autoDownRpm = 800; // keep the lugging step-down out of the way
  t.Configure(c);
  TcuInputs in = In();
  t.Ready();
  t.Tick10ms(in);
  CHECK_EQ(t.Out().pCh7, 0);
  CHECK(EngageTo(t, in, SEL_D));
  in.n_ab = NabFor(1, 3000);
  CHECK(RunTicks(t, in, 20));
  CHECK_EQ(t.Out().pCh7, 0); // 1st is below TccMinGear 2
  in.n_ab = NabFor(1, 1900); // 2nd will sync below TccOnRpm
  CHECK(RunTicks(t, in, 2));
  in.tipUp = true;
  CHECK(RunTicks(t, in, 1));
  in.tipUp = false;
  CHECK_EQ(t.Out().mode, TCU_SHIFT);
  CHECK_EQ(t.Out().pCh7, 0); // never during a shift
  CHECK(Settle(t, in));
  CHECK(RunTicks(t, in, 5));
  CHECK_EQ(t.Out().pCh7, 0); // below TccOnRpm
  in.n_ab = NabFor(2, 2000);
  CHECK(RunTicks(t, in, 2)); // one tick for n_t to follow
  CHECK_EQ(t.Out().pCh7, 500); // ramp 500 mbar per 10 ms
  CHECK(RunTicks(t, in, 20));
  CHECK_EQ(t.Out().pCh7, 6000);
  in.n_ab = NabFor(2, 1300); // inside the hysteresis
  CHECK(RunTicks(t, in, 2));
  CHECK_EQ(t.Out().pCh7, 6000);
  in.n_ab = NabFor(2, 1100);
  CHECK(RunTicks(t, in, 2));
  CHECK_EQ(t.Out().pCh7, 0); // opens at once
  // Scheduled mode (TccMode = Schedule): 2nd gear, pedal 0 locks at
  // 2200 rpm input (placeholder schedule), opens 200 rpm lower.
  c.tccMode = TCCMODE_SCHEDULE;
  c.tcc.minRpm = 1100;
  c.tcc.hystRpm = 200;
  c.tcc.kp10 = 10;
  c.tcc.ki10 = 20;
  c.tcc.startP = 1500;
  c.tcc.slipTgt = 0;
  t.Configure(c);
  in.pedal = 0;
  in.n_ab = NabFor(2, 2100);
  CHECK(RunTicks(t, in, 2));
  CHECK(!t.Out().tccLocked);
  CHECK_EQ(t.Out().pCh7, 0);
  in.n_ab = NabFor(2, 2300);
  CHECK(RunTicks(t, in, 2));
  CHECK(t.Out().tccLocked);
  CHECK_EQ(t.Out().pCh7, 1500); // no motor speed: start pressure, no slip
  // 50 rpm slip: more pressure (RunTicks would reset n_mot to n_t).
  for (int i = 0; i < 10; i++) {
    in.n_mot = in.n_t + 50;
    t.Tick10ms(in);
  }
  CHECK(t.Out().pCh7 > 1500 + 50);
  in.pedal = 100; // more pedal: on speed 3800, but hysteresis from 2200 only
  CHECK(RunTicks(t, in, 1));
  CHECK(!t.Out().tccLocked); // 2300 < 3800 - 200
  in.pedal = 0;
  CHECK(RunTicks(t, in, 1));
  CHECK(t.Out().tccLocked);
  in.tccFault = true; // slip fault: open
  CHECK(RunTicks(t, in, 1));
  CHECK(!t.Out().tccLocked);
  CHECK_EQ(t.Out().pCh7, 0);
  in.tccFault = false;
  in.n_mot = 0;
  c.tccMode = TCCMODE_SIMPLE;
  t.Configure(c);
  in.selector = SEL_N;
  CHECK(RunTicks(t, in, 2));
  CHECK_EQ(t.Out().pCh7, 0);

  c.ch7Mode = CH7_K0;
  c.k0P = 0;
  t.Configure(c);
  CHECK(RunTicks(t, in, 1));
  CHECK_EQ(t.Out().pCh7, 0);
  c.ch7Mode = CH7_OPEN;
  t.Configure(c);
  CHECK(RunTicks(t, in, 1));
  CHECK_EQ(t.Out().pCh7, 0);
  t.ForceSafe(TF_DRIVER);
  CHECK_EQ(t.Out().pCh7, 0);
}

static bool SameCurve(const CalCurve &a, const CalCurve &b) {
  if (a.n != b.n)
    return false;
  for (int i = 0; i < a.n; i++)
    if (a.x[i] != b.x[i] || a.y[i] != b.y[i])
      return false;
  return true;
}

/* Variant rows: one per gearbox and cal. GearboxType ids 0 / 1 unchanged,
 * new rows appended; the solenoid set is a separate per-unit setting. */
static void TestVariants() {
  const VariantCal &e = Cal::Variant(CALVAR_330E);
  const VariantCal &f = Cal::Variant(CALVAR_F11);
  CHECK_EQ(CALVAR_330E, 0);
  CHECK_EQ(CALVAR_F11, 1);
  CHECK_EQ(CALVAR_COUNT, 7);
  CHECK_EQ(Cal::Variant(99).ch7Default, CH7_K0); // unknown -> 330e
  CHECK(Cal::Variant(-1).name == e.name);
  CHECK(e.linPump);
  CHECK(!f.linPump);
  CHECK_EQ(Cal::Ch7Effective(CH7_VARIANT, CALVAR_F11), CH7_TCC);
  CHECK_EQ(Cal::Ch7Effective(CH7_VARIANT, CALVAR_330E), CH7_K0);
  CHECK_EQ(Cal::Ch7Effective(CH7_OPEN, CALVAR_F11), CH7_OPEN);
  CHECK_EQ(Cal::Ch7Effective(CH7_OPEN, CALVAR_330E), CH7_OPEN);
  CHECK_EQ(Cal::Ch7Effective(CH7_TCC, CALVAR_330E), CH7_TCC);

  // Every row, against VARIANTS_SUMMARY.md 5 and the exports.
  struct Want {
    int id; const char *name; uint8_t gbx; uint8_t sol; bool k0;
    int16_t cls, rev, ofs; const char *sizeCal;
  };
  static const Want want[CALVAR_COUNT] = {
      {CALVAR_330E, "8HP75_330e", GBX_8HP75, SOLSET_330E_Q030, true, 760, 500, 0, "Q030"},
      {CALVAR_F11, "8HP45_F11", GBX_8HP45, SOLSET_F11_V058, false, 550, 330, 30, "F11N47"},
      {CALVAR_V095_F11N47, "8HP45_F11N47_v095", GBX_8HP45, SOLSET_V095_45_70, false, 550, 330, 30, "F11N47"},
      {CALVAR_V095_N57, "8HP70_N57_v095", GBX_8HP70, SOLSET_V095_45_70, false, 760, 500, 0, "N57"},
      {CALVAR_V095_N63, "8HP70_N63_v095", GBX_8HP70, SOLSET_V095_45_70, false, 760, 500, 0, "N63"},
      {CALVAR_V095_N74, "8HP90_N74_v095", GBX_8HP90, SOLSET_V095_90, false, 1100, 600, 15, "N74"},
      {CALVAR_V095_RR4, "8HP90_RR4N74_v095", GBX_8HP90, SOLSET_V095_90, false, 1100, 600, 15, "RR4N74"},
  };
  for (const Want &w : want) {
    const VariantCal &v = Cal::Variant(w.id);
    CHECK(strcmp(v.name, w.name) == 0);
    CHECK_EQ(v.gearbox, w.gbx);
    CHECK_EQ(v.solSet, w.sol);
    CHECK_EQ(v.k0Present, w.k0);
    CHECK_EQ(v.ch7Default, w.k0 ? CH7_K0 : CH7_TCC);
    CHECK_EQ(v.engVar, w.k0 ? ENG_VAR_330E : ENG_VAR_F11);
    CHECK_EQ(v.linPump, w.k0);
    CHECK_EQ(v.torqueTxId, 0x0B0); // every row [C]
    CHECK_EQ(v.posConfirmed, w.k0);
    CHECK_EQ(v.size->torqueClassNm, w.cls);
    CHECK_EQ(v.size->revLimitNm, w.rev);
    CHECK_EQ(v.size->safetyOfsNm, w.ofs);
    CHECK(strcmp(v.size->cal, w.sizeCal) == 0);
    // The size data comes from the tables it names (exported values).
    const CalRef *r = Cal::Ref(*v.size, "KL_RGANGLIMIT");
    CHECK(r && r->kind == 1 && r->z[0] == w.rev * 1000);
    const CalRef *t = Cal::Ref(*v.size, "TAB_MVG_MT");
    CHECK(t && t->z[t->nx - 1] == w.cls * 1000);
    CHECK(Cal::Ref(*v.size, "NO_SUCH_LABEL") == 0);
    // Fill tables: 330e Q030 [C] on every row. Shift tables: the F11's
    // own per program (stage 4) on the F11 row, 330e elsewhere; Sport and
    // Track use the Sport set (F11: programs 1 and 2).
    CHECK(v.clutch == calClutch);
    CHECK(v.upMode[DM_NORMAL] == 0);
    CHECK(v.clutchMode[DM_SPORT] == calClutchSport);
    if (w.id == CALVAR_F11) {
      CHECK(v.up == calF11Up[0] && v.down == calF11Down[0]);
      CHECK(v.upMode[DM_SPORT] == calF11Up[1] && v.upMode[DM_TRACK] == calF11Up[2]);
      CHECK(v.downMode[DM_SPORT] == calF11Down[1] && v.downMode[DM_TRACK] == calF11Down[2]);
    } else {
      CHECK(v.up == calUp && v.down == calDown);
      CHECK(v.upMode[DM_SPORT] == calUpSport && v.upMode[DM_TRACK] == calUpSport);
      CHECK(v.downMode[DM_SPORT] == calDownSport);
    }
    // Line pressure / gear-in: own F11 tables on the F11 row only.
    if (w.id == CALVAR_F11)
      CHECK(v.psys == calF11Psys && v.engage == calF11Engage && v.global == &calF11Global);
    else
      CHECK(v.psys == calPsys && v.engage == calEngage && v.global == &calGlobal);
    // K0 guard: K0 asked on a row without one -> open.
    CHECK_EQ(Cal::Ch7Effective(CH7_K0, w.id), w.k0 ? CH7_K0 : CH7_OPEN);
    // Select with Auto picks the row default; an explicit set wins.
    Cal::Select(w.id);
    CHECK_EQ(Cal::ActiveSolSet(), w.sol);
    CHECK(Cal::Eds() == Cal::SolSet(w.sol).eds);
    Cal::Select(w.id, SOLSET_330E_Q030);
    CHECK_EQ(Cal::ActiveSolSet(), SOLSET_330E_Q030);
    CHECK(Cal::Eds() == calEds);
    Cal::Select(w.id, 77); // out of range -> row default
    CHECK_EQ(Cal::ActiveSolSet(), w.sol);
  }
  CHECK_EQ(Cal::SolSet(SOLSET_AUTO).eds, calEds);
  CHECK_EQ(Cal::SolSet(SOLSET_V095_90).evidence, 'I');
  CHECK_EQ(Cal::SolSet(SOLSET_F11_V058).evidence, 'C');

  // F11 v058 vs v095 8HP90 solenoid sets: A..E and ch7 identical [C],
  // SYS differs; the v095 45 / 70 set is its own.
  for (int i = 0; i < 7; i++)
    CHECK_EQ(SameCurve(calF11Eds[i], calV095Eds90[i]), i != 5);
  CHECK(!SameCurve(calV095Eds45[0], calV095Eds90[0]) ||
        !SameCurve(calV095Eds45[5], calV095Eds90[5]));

  // DriveMode tables: Sport has its own OEM labels (Q030 *SPORT*).
  CHECK(CAL_SPORT_LABELS > 100);
  Cal::Select(CALVAR_330E);
  Cal::SelectMode(DM_SPORT);
  CHECK(Cal::Up() == calUpSport && Cal::Down() == calDownSport &&
        Cal::Clutch() == calClutchSport);
  Cal::SelectMode(DM_TRACK);
  CHECK(Cal::Up() == calUpSport);
  Cal::SelectMode(9);
  CHECK(Cal::Up() == calUp && Cal::Clutch() == calClutch);
  int differ = 0;
  for (int i = 0; i < CAL_NUM_PAIRS; i++)
    differ += calUpSport[i].pofmtn.z != calUp[i].pofmtn.z;
  CHECK(differ > 0);
  Cal::SelectMode(DM_NORMAL);

  // n6 k-factors exported (not used): D_k_n6 starts 1.5, x10000.
  CHECK_EQ(calN6.k_n6.n, 20);
  CHECK_EQ(calN6.k_n6.y[0], 15000);
  CHECK_EQ(calN6.k_n6_ntopt.n, 32);
  CHECK_EQ(calN6.i_targetgear.y[0], 33060);

  // Clutch A start current: F11 ~176 mA, 330e ~240.6 mA [C]
  Cal::Select(CALVAR_F11);
  CHECK_EQ(Eds::PressureToCurrent(EDS_A, 10), 1760);
  CHECK_EQ(Eds::PressureToCurrent(EDS_B, 300), 2350);
  CHECK_EQ(Eds::PressureToCurrent(EDS_K0, 280), 3070); // TCC curve
  CHECK_EQ(Eds::PressureToCurrent(EDS_SYS, 3904), 8500);
  CHECK_EQ(Eds::PressureToCurrent(EDS_C, 14625), 500);
  // F11 fill is the 330e Q030 set now (the F11 port's fill rows are [I]).
  CHECK_EQ(Cal::Curve(Cal::Active().clutch[CL_A].tsfcg, -30), 390);
  // F11 line pressure / gear-in from f11_psys_gearin_cal, the rest 330e.
  CHECK_EQ(Cal::Map(Cal::Active().psys[0], -200, 2500), 14650);   // KF_PSYS_R
  CHECK_EQ(Cal::Map(Cal::Active().global->psysn, 2000, 10), 10000);
  CHECK_EQ(Cal::Map(calGlobal.psysn, 2000, 1000), 6000);
  CHECK_EQ(Cal::Map(Cal::Active().engage[0].pkzu, 704, -18), 400); // KF_PKZU_1
  CHECK_EQ(Cal::Curve(Cal::Active().engage[0].pgrasyn, -50), 500);
  CHECK_EQ(Cal::Curve(calEngage[0].pgrasyn, -50), 1000);
  // Stage 4 gear-in timings: T_SYN R/1 = 1000 ms, TFV1 / TVKZUCGV2 = 0,
  // TVKZU 100 ms; TFCGV2 stays 330e [H].
  CHECK_EQ(Cal::Active().engage[0].t_syn, 1000);
  CHECK_EQ(Cal::Active().engage[1].t_syn, 1000);
  CHECK_EQ(Cal::Curve(Cal::Active().engage[0].tfv1, 0), 0);
  CHECK_EQ(Cal::Curve(Cal::Active().engage[1].tvkzucgv2, 20), 0);
  CHECK_EQ(Cal::Curve(Cal::Active().engage[0].tvkzu, 500), 100);
  CHECK(SameCurve(Cal::Active().engage[0].tfcgv2, calEngage[0].tfcgv2));
  const int tsyn[9] = {1000, 1000, 1000, 200, 200, 200, 200, 200, 0}; // R, 1..8
  for (int i = 0; i < 9; i++) {
    CHECK_EQ(calF11EngageAll[i].t_syn, tsyn[i]);
    CHECK_EQ(Cal::Curve(calF11EngageAll[i].tvkzucgv2, 60), 0);
  }
  CHECK_EQ(Cal::Curve(calF11EngageAll[2].tfv1, 1000), 200); // 2nd [C]
  CHECK_EQ(Cal::Curve(calF11EngageAll[8].tvkzu, 0), 0);
  CHECK(calF11Engage[1].pkzu.z == calF11EngageAll[0].pkzu.z); // N->R = R
  CHECK(calF11Engage[0].pkzu.z == calF11EngageAll[1].pkzu.z); // N->1 = 1st
  CHECK_EQ(Cal::Map(calF11EngageAll[8].pkzu, 992, -18), 0);    // KF_PKZU_8 [C]
  // Stage 4 globals; P_SYSMINPSAUS and I_EDSPOSV / PSV stay 330e.
  CHECK_EQ(Cal::Active().global->p_sysmin_d, 7000);
  CHECK_EQ(Cal::Active().global->p_sysmin_r, 6500);
  CHECK_EQ(Cal::Active().global->p_maxobgls, 3000);
  CHECK_EQ(Cal::Active().global->p_maxungls, 3000);
  CHECK_EQ(Cal::Active().global->t_fngtgls, 70);
  CHECK_EQ(calF11Global.p_sysminpsaus, calGlobal.p_sysminpsaus);
  CHECK_EQ(calF11Global.i_edsposv, calGlobal.i_edsposv);
  CHECK_EQ(calF11Global.i_edspsv, calGlobal.i_edspsv);
  CHECK_EQ(calF11PosysCg.n, 8);
  CHECK_EQ(Cal::Curve(calF11PosysCg, -40), 6500);
  CHECK_EQ(Cal::Curve(calF11PosysCg, 20), 0);
  // Stage 4 shift maps keyed by program + pair + direction; axes speed
  // rpm (raw x 32), torque Nm, values mbar.
  const UpShiftCal &u34 = calF11Up[0][2], &u34s = calF11Up[2][2];
  CHECK_EQ(u34.pkzu.nx, 8);
  CHECK_EQ(u34.pkzu.ny, 10);
  CHECK_EQ(u34.pkzu.x[0], 992);
  CHECK_EQ(u34.pkzu.y[0], -72);
  CHECK_EQ(Cal::Map(u34.pkzu, 992, -72), 100);
  CHECK_EQ(Cal::Map(u34.pkzu, 7008, 552), 5100);   // KF_PKZU34
  CHECK_EQ(Cal::Map(u34s.pkzu, 7008, 552), 6400);  // program 2 (SPORT)
  CHECK(u34.pkab.z == u34s.pkab.z);                 // one off-going set
  CHECK_EQ(Cal::Map(u34.pkab, 992, -102), 500);     // KF_PKAB34
  CHECK_EQ(u34.p_sysmin, 5000);
  CHECK_EQ(calF11Up[0][0].p_sysmin, 6500);           // P_SYSMIN12
  const DownShiftCal &d21 = calF11Down[0][0], &d21s = calF11Down[1][0];
  CHECK_EQ(Cal::Map(d21.pkab, 0, -102), 500);        // KF_PKAB21
  CHECK_EQ(Cal::Map(d21s.pkab, 32, -102), 1100);     // program 1 (SP)
  CHECK_EQ(Cal::Map(calF11Down[2][4].pkab, 992, -102), 250); // SPORT 65
  CHECK_EQ(d21.p_sysmin, 6500);
  // Fields without an F11 source point at the 330e arrays.
  for (int k = 0; k < CAL_NUM_PAIRS; k++) {
    CHECK(calF11Up[0][k].tfcgnt.z == calUp[k].tfcgnt.z);
    CHECK(calF11Up[1][k].tfcgnt.z == calUpSport[k].tfcgnt.z);
    CHECK(calF11Down[0][k].pkzu.z == calDown[k].pkzu.z); // 0x54BA0 not mapped
    CHECK(calF11Down[2][k].pkzu.z == calDownSport[k].pkzu.z);
    CHECK(calF11Up[0][k].pkzu.z != calUp[k].pkzu.z);
    CHECK(calF11Down[0][k].pkab.z != calDown[k].pkab.z);
    CHECK_EQ(calF11Up[1][k].t_syn, calUpSport[k].t_syn);
  }
  // DriveMode on the F11 row switches the program tables.
  Cal::SelectMode(DM_SPORT);
  CHECK(Cal::Up() == calF11Up[1] && Cal::Down() == calF11Down[1]);
  Cal::SelectMode(DM_TRACK);
  CHECK(Cal::Up() == calF11Up[2]);
  Cal::SelectMode(DM_NORMAL);
  CHECK(Cal::Up() == calF11Up[0]);
  CHECK(SameCurve(calF11Global.tvpsein, calGlobal.tvpsein)); // [C] port, same
  CHECK_EQ(calF11TmaxPsys.n, 4);
  // v095 rows run the 330e tables with their own solenoid set.
  Cal::Select(CALVAR_V095_N74);
  CHECK(Cal::Eds() == calV095Eds90);
  CHECK(Cal::Active().psys == calPsys);
  // F11 gear-in through the whole TCU with the F11 tables: last clutch C
  // for 1st, D for R
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_1, ENG_VAR_F11), CL_C);
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_R, ENG_VAR_F11), CL_D);
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_1, ENG_VAR_330E), CL_B);
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_R, ENG_VAR_330E), CL_B);
  for (int dir = 0; dir < 2; dir++) {
    Tcu t;
    TcuConfig c = Cfg();
    c.engage.variant = ENG_VAR_F11;
    c.ch7Mode = CH7_TCC;
    c.shiftMode = SHIFT_AUTO;
    t.Configure(c);
    TcuInputs in = In();
    t.Ready();
    t.Tick10ms(in);
    CHECK(EngageTo(t, in, dir ? SEL_R : SEL_D));
    CHECK_EQ(t.Out().gear, dir ? GEAR_R : GEAR_1);
    if (!dir) {
      in.n_ab = NabFor(1, 4500);
      CHECK(Settle(t, in));
      bool up = false;
      for (int i = 0; i < 400 && !up; i++) {
        CHECK(RunTicks(t, in, 1));
        up = t.Out().gear >= 2;
      }
      CHECK(up);
    }
    CHECK_EQ(t.Out().fault, TF_NONE);
  }
  Cal::Select(CALVAR_330E);
  CHECK_EQ(Eds::PressureToCurrent(EDS_A, 10), 2406);
  CHECK_EQ(Cal::Curve(Cal::Active().clutch[CL_A].tsfcg, -30), 390);
}

/* DriveMode: Sport / Track modifiers on the Normal config. */
static int ShiftTicks(int32_t timePct) {
  Tcu t;
  TcuConfig c = Cfg();
  c.shiftMode = SHIFT_MANUAL;
  c.autoDownRpm = 800;
  c.sas.timePct = timePct;
  t.Configure(c);
  TcuInputs in = In();
  t.Ready();
  t.Tick10ms(in);
  if (!EngageTo(t, in, SEL_D))
    return -1;
  in.n_ab = NabFor(1, 3000);
  RunTicks(t, in, 5);
  in.tipUp = true;
  RunTicks(t, in, 1);
  in.tipUp = false;
  int n = 1;
  while (t.Out().mode == TCU_SHIFT && n < 1000) {
    if (!RunTicks(t, in, 1))
      return -1;
    n++;
  }
  return t.Out().gear == 2 ? n : -1;
}

static void TestDriveMode() {
  TcuConfig n = Cfg();
  // Normal: unchanged.
  TcuConfig c = DriveModes::Apply(n, DriveModes::Neutral());
  CHECK_EQ(c.autoUpRpm, n.autoUpRpm);
  CHECK_EQ(c.autoDownRpm, n.autoDownRpm);
  CHECK_EQ(c.tccOnRpm, n.tccOnRpm);
  CHECK_EQ(c.tccOffRpm, n.tccOffRpm);
  CHECK_EQ(c.sas.timePct, 100);
  CHECK(!c.noLiftUpshift);

  // Sport (the parameter defaults).
  DriveModeMod sp = DriveModes::Neutral();
  sp.upOfsRpm = 300;
  sp.downOfsRpm = 200;
  sp.timePct = 90;
  sp.tccOfsRpm = 200;
  c = DriveModes::Apply(n, sp);
  CHECK_EQ(c.autoUpRpm, 4300);
  CHECK_EQ(c.autoDownRpm, 1700);
  CHECK_EQ(c.sas.timePct, 90);
  CHECK_EQ(c.tccOnRpm, 1300);
  CHECK_EQ(c.tccOffRpm, 1000);
  // Safety limits are not touched.
  CHECK_EQ(c.manMaxRpm, n.manMaxRpm);
  CHECK_EQ(c.manMinRpm, n.manMinRpm);
  CHECK_EQ(c.ilOpenLow, n.ilOpenLow);
  CHECK_EQ(c.ilOpenHigh, n.ilOpenHigh);
  CHECK_EQ(c.lineMax, n.lineMax);
  CHECK_EQ(c.iMax, n.iMax);
  CHECK_EQ(c.engageMaxNab, n.engageMaxNab);
  CHECK_EQ(c.applyP, n.applyP);
  CHECK_EQ(c.sas.pMax, n.sas.pMax);
  CHECK_EQ(c.canLossMode, n.canLossMode);

  // Clamps: time never below the floor or above 100 %, no up point over
  // ManMaxRpm, the down point stays 500 rpm under the up point, the TCC
  // thresholds stay >= 0 and ordered.
  sp.timePct = 10;
  sp.upOfsRpm = 9000;
  sp.downOfsRpm = 9000;
  sp.tccOfsRpm = 3000;
  c = DriveModes::Apply(n, sp);
  CHECK_EQ(c.sas.timePct, DRIVEMODE_TIME_MIN_PCT);
  CHECK_EQ(c.autoUpRpm, n.manMaxRpm);
  CHECK_EQ(c.autoDownRpm, n.manMaxRpm - 500);
  CHECK_EQ(c.tccOffRpm, 0);
  CHECK(c.tccOnRpm >= c.tccOffRpm);
  sp.timePct = 150;
  CHECK_EQ(DriveModes::Apply(n, sp).sas.timePct, 100);

  // Track: up at the redline only.
  DriveModeMod tr = DriveModes::Neutral();
  tr.holdToRedline = true;
  tr.redlineRpm = 5500;
  tr.liftTqNm = 10;
  tr.timePct = 80;
  c = DriveModes::Apply(n, tr);
  CHECK_EQ(c.autoUpRpm, 5500);
  CHECK(c.noLiftUpshift);
  tr.redlineRpm = 9000;
  CHECK_EQ(DriveModes::Apply(n, tr).autoUpRpm, n.manMaxRpm);
  tr.redlineRpm = 5500;

  {
    Tcu t;
    TcuConfig tc = DriveModes::Apply(n, tr);
    tc.shiftMode = SHIFT_AUTO;
    t.Configure(tc);
    TcuInputs in = In();
    t.Ready();
    t.Tick10ms(in);
    CHECK(EngageTo(t, in, SEL_D));
    in.n_ab = NabFor(1, 4500); // Normal would upshift here
    CHECK(RunTicks(t, in, 100));
    CHECK_EQ(t.Out().gear, 1);
    CHECK_EQ(t.Out().mode, TCU_DRIVE);
    in.n_ab = NabFor(1, 5700); // past the redline but lifted off
    in.m_t = 0;
    CHECK(RunTicks(t, in, 100));
    CHECK_EQ(t.Out().gear, 1);
    CHECK_EQ(t.Out().mode, TCU_DRIVE);
    in.m_t = 100; // back on the throttle: upshift at the redline
    bool up = false;
    for (int i = 0; i < 400 && !up; i++) {
      CHECK(RunTicks(t, in, 1));
      up = t.Out().gear >= 2;
    }
    CHECK(up);
  }
  {
    // Lifted off at the over-rev limit: it still upshifts.
    Tcu t;
    TcuConfig tc = DriveModes::Apply(n, tr);
    tc.shiftMode = SHIFT_AUTO;
    t.Configure(tc);
    TcuInputs in = In();
    t.Ready();
    t.Tick10ms(in);
    CHECK(EngageTo(t, in, SEL_D));
    in.m_t = 0;
    in.n_ab = NabFor(1, 6100);
    bool up = false;
    for (int i = 0; i < 400 && !up; i++) {
      CHECK(RunTicks(t, in, 1));
      up = t.Out().gear >= 2;
    }
    CHECK(up);
  }

  // Quicker shift with a lower time scale, same end gear.
  int tNormal = ShiftTicks(100);
  int tQuick = ShiftTicks(60);
  CHECK(tNormal > 0);
  CHECK(tQuick > 0);
  CHECK(tQuick < tNormal);
  CHECK_EQ(ShiftTicks(0), tNormal); // 0 = cal as is

  // Shift tables: Normal uses the row's, Sport and Track the OEM Sport
  // set from Q030; an out-of-range mode falls back to Normal.
  Cal::Select(CALVAR_330E);
  for (int m = 0; m < DM_COUNT; m++) {
    Cal::SelectMode(m);
    CHECK(Cal::Up() == (m == DM_NORMAL ? Cal::Active().up : calUpSport));
    CHECK(Cal::Down() == (m == DM_NORMAL ? Cal::Active().down : calDownSport));
  }
  Cal::SelectMode(7);
  CHECK_EQ(Cal::ActiveMode(), DM_NORMAL);
}

/* TIS cross-check: in a steady gear exactly 3 elements are applied, and
 * the interlock does not tighten the 4-in-a-shift rule. */
static void TestSteadyStateThree() {
  Tcu t;
  t.Configure(Cfg());
  TcuInputs in = In();
  t.Ready();
  CHECK(RunTicks(t, in, 2));
  CHECK(EngageTo(t, in, SEL_D));
  in.n_ab = 1000;
  in.n_t = Gears::SyncSpeed(1, 1000);
  bool saw4 = false;
  for (int g = 1; g <= 8; g++) {
    in.gearReq = (uint8_t)g;
    for (int i = 0; i < 600 && !(t.Out().gear == g && t.Out().mode == TCU_DRIVE); i++) {
      CHECK(RunTicks(t, in, 1));
      if (t.Out().mode == TCU_SHIFT && Gears::CountClutches(t.Out().applied) == 4)
        saw4 = true;
      CHECK(Gears::CountClutches(t.Out().applied) <= 4); // never 5
    }
    CHECK(RunTicks(t, in, 5));
    CHECK_EQ(t.Out().gear, g);
    CHECK_EQ(t.Out().mode, TCU_DRIVE);
    CHECK_EQ(Gears::CountClutches(t.Out().applied), 3);
    CHECK_EQ(t.Out().applied, Gears::ClutchMask(g));
  }
  CHECK(saw4); // overlap shifts do use the fourth element
}

/* TIS park rules: engage only stopped, release needs the footbrake, the
 * magnet is never on for more than 1 s. */
// ParkBrake sources, the rev C 12 V brake input included.
static void TestFootbrakeSrc() {
  // VcuCan (default): the frame bit, only while CAN is alive
  CHECK(FootbrakeFrom(PARKBRAKE_CAN, true, true, false, true, false));
  CHECK(!FootbrakeFrom(PARKBRAKE_CAN, false, true, false, true, true));
  CHECK(!FootbrakeFrom(PARKBRAKE_CAN, true, false, true, true, true));
  // Param
  CHECK(FootbrakeFrom(PARKBRAKE_PARAM, false, false, true, false, false));
  CHECK(!FootbrakeFrom(PARKBRAKE_PARAM, true, true, false, true, true));
  // Bypass
  CHECK(FootbrakeFrom(PARKBRAKE_BYPASS, false, false, false, false, false));
  // Input: the pin, independent of CAN; never pressed where not fitted
  CHECK(FootbrakeFrom(PARKBRAKE_INPUT, false, false, false, true, true));
  CHECK(!FootbrakeFrom(PARKBRAKE_INPUT, true, true, true, true, false));
  CHECK(!FootbrakeFrom(PARKBRAKE_INPUT, true, true, true, false, true));
  // Unknown values fall back to VcuCan
  CHECK(!FootbrakeFrom(9, false, true, true, true, true));

  // Debounce: 3 equal samples to change, a 1-2 sample bounce is ignored
  Debounce d;
  CHECK(!d.Tick(true, 3));
  CHECK(!d.Tick(true, 3));
  CHECK(d.Tick(true, 3));
  CHECK(d.Tick(false, 3));
  CHECK(d.Tick(true, 3)); // bounce: count restarts
  CHECK(d.Tick(false, 3));
  CHECK(d.Tick(false, 3));
  CHECK(!d.Tick(false, 3));
  CHECK(!d.State());

  // End to end: P only releases with the input pressed
  TcuConfig c = Cfg();
  c.parkCtrl = true;
  c.parkNeedBrake = true;
  Tcu t;
  t.Configure(c);
  t.Ready();
  TcuInputs in = In();
  in.selector = SEL_P;
  for (int i = 0; i < 50; i++)
    t.Tick10ms(in);
  in.selector = SEL_D;
  in.brake = FootbrakeFrom(PARKBRAKE_INPUT, false, false, false, true, false);
  for (int i = 0; i < 100; i++)
    t.Tick10ms(in);
  CHECK(t.Out().parkBrakeWait);
  in.brake = FootbrakeFrom(PARKBRAKE_INPUT, false, false, false, true, true);
  t.Tick10ms(in);
  CHECK(!t.Out().parkBrakeWait);
}

static void TestParkTis() {
  TcuConfig c = Cfg();
  c.parkCtrl = true;
  c.parkEngKmh = 5;
  c.nabPerKmh = 25;
  c.parkEngNab = 5;
  c.parkNeedBrake = true;
  c.parkMagMaxMs = 5000; // asks for more than the hard 1 s limit
  Tcu t;
  t.Configure(c);
  TcuInputs in = In();
  in.temp = 40;
  t.Ready();
  CHECK(RunTicks(t, in, 2));
  // P asked while rolling at 4 km/h (100 rpm output): held in N, PZYL on
  in.selector = SEL_P;
  in.n_ab = 100;
  for (int i = 0; i < 100; i++)
    t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  CHECK(t.Out().parkInhibit);
  CHECK(t.Out().pzylI > 0);
  // Still creeping at 5 rpm (well under 1 km/h): still held
  in.n_ab = 5;
  for (int i = 0; i < 50; i++)
    t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_NEUTRAL);
  CHECK(t.Out().parkInhibit);
  // km/h rule on its own: 6 km/h is too fast even with the rpm check off
  {
    TcuConfig k = c;
    k.parkEngNab = 0;
    Tcu u;
    u.Configure(k);
    TcuInputs ui = In();
    ui.temp = 40;
    u.Ready();
    u.Tick10ms(ui);
    ui.selector = SEL_P;
    ui.n_ab = 6 * 25;
    for (int i = 0; i < 50; i++)
      u.Tick10ms(ui);
    CHECK_EQ(u.Out().mode, TCU_NEUTRAL);
    ui.n_ab = 4 * 25;
    for (int i = 0; i < 20; i++)
      u.Tick10ms(ui);
    CHECK_EQ(u.Out().mode, TCU_PARK);
  }
  // Stopped: the 150 ms delay runs, then P with PZYL and magnet off
  in.n_ab = 0;
  for (int i = 0; i < 16; i++)
    t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_PARK);
  CHECK(!t.Out().parkInhibit);
  CHECK_EQ(t.Out().pzylI, 0);
  CHECK(!t.Out().parkMagnet);
  // Release with pressure but no footbrake: waits
  in.selector = SEL_D;
  in.pMeas = 17500;
  for (int i = 0; i < 50; i++)
    t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_PARK);
  CHECK(t.Out().parkBrakeWait);
  CHECK(!t.Out().parkWait);
  // Brake pressed: 150 ms of brake + 17 bar, then out of P
  in.brake = true;
  for (int i = 0; i < 10; i++)
    t.Tick10ms(in);
  in.brake = false; // let go half way: the delay starts again
  t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_PARK);
  in.brake = true;
  for (int i = 0; i < 14; i++)
    t.Tick10ms(in);
  CHECK_EQ(t.Out().mode, TCU_PARK);
  t.Tick10ms(in);
  CHECK(t.Out().mode != TCU_PARK);
  CHECK(!t.Out().parkBrakeWait);
  // Magnet: on after the release, off by 1 s although ParkMagMax = 5 s
  CHECK(t.Out().parkMagnet);
  int onTicks = 0;
  for (int i = 0; i < 300; i++) {
    t.Tick10ms(in);
    if (t.Out().parkMagnet)
      onTicks++;
  }
  CHECK(onTicks <= PARK_MAG_LIMIT_MS / 10);
  CHECK(!t.Out().parkMagnet);
  // Pressure gone (motor off) while held by the brake only: no release
  Tcu v;
  v.Configure(c);
  TcuInputs vi = In();
  vi.temp = 40;
  vi.pMeas = -1;
  v.Ready();
  v.Tick10ms(vi);
  vi.selector = SEL_P;
  for (int i = 0; i < 20; i++)
    v.Tick10ms(vi);
  CHECK_EQ(v.Out().mode, TCU_PARK);
  c.parkRelRpm = 700;
  v.Configure(c);
  vi.selector = SEL_N;
  vi.brake = true;
  for (int i = 0; i < 50; i++)
    v.Tick10ms(vi);
  CHECK_EQ(v.Out().mode, TCU_PARK);
  CHECK(v.Out().parkWait);
  CHECK(!v.Out().parkBrakeWait);
}

/* The 1 s limiter used on the park magnet output. */
static void TestOnTimeLimit() {
  OnTimeLimit l;
  int on = 0;
  for (int i = 0; i < 300; i++)
    on += l.Tick(true, PARK_MAG_LIMIT_MS, 10) ? 1 : 0;
  CHECK_EQ(on, 100); // 1 s, then off although still requested
  CHECK(!l.Tick(true, PARK_MAG_LIMIT_MS, 10));
  CHECK(!l.Tick(false, PARK_MAG_LIMIT_MS, 10)); // request dropped
  CHECK(l.Tick(true, PARK_MAG_LIMIT_MS, 10));   // a new pulse is allowed
}

/* parkSt spot value. */
static void TestParkState() {
  Tcu t;
  t.Configure(Cfg());
  TcuInputs in = In();
  CHECK_EQ(t.Out().parkState, PARK_UNHELD); // INIT
  t.Ready();
  in.selector = SEL_N;
  CHECK(RunTicks(t, in, 2));
  CHECK_EQ(t.Out().parkState, PARK_RELEASED);
  in.selector = SEL_P;
  CHECK(RunTicks(t, in, 1));
  CHECK_EQ(t.Out().parkState, PARK_ENGAGING);
  CHECK(RunTicks(t, in, 300));
  CHECK_EQ(t.Out().mode, TCU_PARK);
  CHECK_EQ(t.Out().parkState, PARK_ENGAGED);
  in.selector = SEL_N; // no pressure source: the release waits
  CHECK(RunTicks(t, in, 3));
  CHECK_EQ(t.Out().parkState, PARK_RELEASING);
  t.ForceSafe(TF_DRIVER);
  CHECK_EQ(t.Out().parkState, PARK_UNHELD);
  CHECK(!t.Out().inertia);
}

void TestTcu() {
  TestDriveMode();
  TestParkState();
  TestManualGear();
  TestTipOverride();
  TestPark();
  TestParkTis();
  TestFootbrakeSrc();
  TestOnTimeLimit();
  TestSteadyStateThree();
  TestGearInSource();
  TestCh7();
  TestVariants();
  TestManual();
  TestAuto();
  TestEngageAndShift();
  TestReverse();
  TestFaults();
  TestNoTorqueIntervention();
  TestEngageNoTorqueHold();
  TestEngageF11();
  TestCanLoss();
  TestCoastShift();
}
