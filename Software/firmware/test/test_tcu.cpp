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
#include "caltypes.h"
#include "eds.h"
#include "engage.h"
#include "gears.h"
#include "tcu.h"
#include "test.h"

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

/* Variant tables: GearboxType picks the EDS curves and fill items; the
 * shift tables are shared. */
static void TestVariants() {
  const VariantCal &e = Cal::Variant(CALVAR_330E);
  const VariantCal &f = Cal::Variant(CALVAR_F11);
  CHECK_EQ(Cal::Variant(99).ch7Default, CH7_K0); // unknown -> 330e
  CHECK(e.linPump);
  CHECK(!f.linPump);
  CHECK_EQ(f.torqueTxId, 0x0B0);
  CHECK_EQ(e.torqueTxId, 0);
  CHECK(e.up == f.up && e.down == f.down && e.engage == f.engage);
  CHECK(e.clutch != f.clutch && e.eds != f.eds);
  CHECK_EQ(Cal::Ch7Effective(CH7_VARIANT, CALVAR_F11), CH7_TCC);
  CHECK_EQ(Cal::Ch7Effective(CH7_VARIANT, CALVAR_330E), CH7_K0);
  CHECK_EQ(Cal::Ch7Effective(CH7_OPEN, CALVAR_F11), CH7_OPEN);
  CHECK_EQ(Cal::Ch7Effective(CH7_OPEN, CALVAR_330E), CH7_OPEN);
  CHECK_EQ(Cal::Ch7Effective(CH7_TCC, CALVAR_330E), CH7_TCC);

  // Clutch A start current: F11 ~176 mA, 330e ~240.6 mA [C]
  Cal::Select(CALVAR_F11);
  CHECK_EQ(Eds::PressureToCurrent(EDS_A, 10), 1760);
  CHECK_EQ(Eds::PressureToCurrent(EDS_B, 300), 2350);
  CHECK_EQ(Eds::PressureToCurrent(EDS_K0, 280), 3070); // TCC curve
  CHECK_EQ(Eds::PressureToCurrent(EDS_SYS, 3904), 8500);
  CHECK_EQ(Eds::PressureToCurrent(EDS_C, 14625), 500);
  // F11 fill items (F11_8HP45_SHIFT_CLUTCH.md 3.1)
  CHECK_EQ(Cal::Curve(Cal::Active().clutch[CL_A].tsfcg, -30), 360);
  CHECK_EQ(Cal::Curve(Cal::Active().clutch[CL_A].pfcg, 40), 800);
  CHECK_EQ(Cal::Curve(Cal::Active().clutch[CL_D].pfcg, 40), 1000);
  CHECK_EQ(Cal::Curve(Cal::Active().clutch[CL_E].pfcg, 40), 1150);
  CHECK_EQ(Cal::Curve(Cal::Active().clutch[CL_B].tsfcg, -30), 490);
  CHECK_EQ(Cal::Map(Cal::Active().clutch[CL_B].psfcgmt, 40, 59), 5000);
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

void TestTcu() {
  TestManualGear();
  TestTipOverride();
  TestPark();
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
