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
#include "engage.h"
#include "cal_330e.h"
#include "gears.h"
#include "test.h"

static EngageConfig Cfg(int variant) {
  EngageConfig c;
  c.pMax = 12750;
  c.fillEqMs = 50;
  c.rampMs = 300;
  c.syncBand = 30;
  c.fastCloseGrad = 30000;
  c.syncConfirmMs = 80;
  c.tqZeroNm = 10;
  c.variant = (uint8_t)variant;
  return c;
}

struct EngResult
{
  int ms;          // total time to done (or the limit)
  int closeMs;     // time spent in ENG_CLOSE
  bool done;
  bool sawFast;
  int32_t maxStep; // largest per-tick rise of the power-flow element in ENG_CLOSE
};

static EngResult RunEngage(int gear, int variant, EngageInputs in, int limitMs = 10000) {
  EngResult r = {0, 0, false, false, 0};
  Engage e;
  CHECK(e.Start(gear, Cfg(variant)));
  int pc = Engage::PowerFlowClutch(gear, variant);
  CHECK_EQ(e.PowerClutch(), pc);
  uint8_t mask = Gears::ClutchMask(gear);
  int lastPhase = ENG_FILL;
  bool order = true;
  int32_t lastP = 0;
  while (!e.Done() && r.ms < limitMs) {
    e.Tick(in);
    if (e.Phase() < lastPhase)
      order = false;
    lastPhase = e.Phase();
    // Only the elements of the target gear ever get pressure.
    for (int c = 0; c < CL_COUNT; c++)
      if (!(mask & CLUTCH_BIT(c)))
        CHECK_EQ(e.Pressure(c), 0);
    // The power-flow element waits until the other two are filled (B
    // stays at its neutral pre-fill on the 330e).
    if (e.Phase() == ENG_FILL)
      CHECK_EQ(e.Pressure(pc), pc == CL_B ? 550 : 0);
    if (e.Phase() == ENG_CLOSE || e.Done()) {
      int32_t step = e.Pressure(pc) - lastP;
      if (step > r.maxStep)
        r.maxStep = step;
      r.closeMs += 10;
    }
    // Torque must be held near 0 for as long as the gear is not clamped.
    CHECK(e.TorqueHold() == !e.Done());
    if (e.FastClose())
      r.sawFast = true;
    lastP = e.Pressure(pc);
    r.ms += 10;
  }
  r.done = e.Done();
  CHECK(order);
  if (r.done)
    for (int c = 0; c < CL_COUNT; c++)
      if (mask & CLUTCH_BIT(c))
        CHECK(e.Pressure(c) >= 12750);
  return r;
}

void TestEngage() {
  // Last clutch in, per variant (Ian, QA2).
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_1, ENG_VAR_330E), CL_B);
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_R, ENG_VAR_330E), CL_B);
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_1, ENG_VAR_F11), CL_C);
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_R, ENG_VAR_F11), CL_D);
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_2, ENG_VAR_330E), -1);
  CHECK_EQ(Engage::PowerFlowClutch(GEAR_1, 7), -1);
  Engage e;
  CHECK(!e.Start(GEAR_2, Cfg(ENG_VAR_330E)));
  CHECK(!e.Start(GEAR_1, Cfg(9)));

  // 330e: A+C (1st) and A+D (R) are the pre-filled pair.
  EngageInputs still = {0, 0, 0, 40, true, true}; // standstill, torque 0
  for (int v = 0; v < 2; v++) {
    for (int g = 0; g < 2; g++) {
      int gear = g ? GEAR_R : GEAR_1;
      // Fast close: no slip on valid sensors, torque near 0, line OK.
      EngResult fast = RunEngage(gear, v, still);
      CHECK(fast.done);
      CHECK(fast.sawFast);
      CHECK(fast.ms < 5000);
      CHECK(fast.closeMs <= 1500); // full within about 1.5 s
      CHECK(fast.maxStep <= 30000 * 10 / 1000); // rate limited
      // Torque not near 0: OEM gradients, slower.
      EngageInputs tq = still;
      tq.m_t = 40;
      EngResult oem = RunEngage(gear, v, tq, 30000);
      CHECK(oem.done);
      CHECK(!oem.sawFast);
      CHECK(oem.closeMs > fast.closeMs);
      // OEM: T_SYN_n in sync, then KL_PORASYN_n to PMAX (1500 ms on N->1,
      // 1100 ms on N->R). Its last step to PMAX is steep (OEM data), so the
      // rate is not checked here.
      const EngageCal &ec = calEngage[g];
      // (at standstill the T_SYN_n dwell may already be over when the
      // close phase begins, so only the ramp length is certain)
      CHECK_EQ(ec.t_syn, 200);
      CHECK(oem.closeMs >= ec.porasyn.x[ec.porasyn.n - 1]);
      // Line pressure not adequate: no fast close.
      EngageInputs line = still;
      line.lineOk = false;
      EngResult lo = RunEngage(gear, v, line, 30000);
      CHECK(lo.done);
      CHECK(!lo.sawFast);
      // Speed sensors not valid: no fast close, but it still finishes.
      EngageInputs ns = still;
      ns.speedsValid = false;
      EngResult nsr = RunEngage(gear, v, ns, 30000);
      CHECK(nsr.done);
      CHECK(!nsr.sawFast);
      // Persistent slip: no fast close and never "in gear".
      EngageInputs slip = still;
      slip.n_t = 300;
      EngResult sl = RunEngage(gear, v, slip, 4000);
      CHECK(!sl.done);
      CHECK(!sl.sawFast);
    }
  }

  // V1 starts at once, V2 waits KL_TVKZUCGV2_n (50 ms at 40 degC) and the
  // roles follow tables 0x4C21E / 0x4C228 (330e) and 0x5BA12 / 0x5BA1C (F11).
  struct { int v, g, v1, v2; } roleCase[] = {
      {ENG_VAR_330E, GEAR_1, CL_A, CL_C}, {ENG_VAR_330E, GEAR_R, CL_A, CL_D},
      {ENG_VAR_F11, GEAR_1, CL_A, CL_B}, {ENG_VAR_F11, GEAR_R, CL_A, CL_B}};
  for (auto &rc : roleCase) {
    Engage r;
    CHECK(r.Start(rc.g, Cfg(rc.v)));
    int32_t tv2 = Cal::Curve(calEngage[rc.g == GEAR_1 ? 0 : 1].tvkzucgv2, 40);
    CHECK_EQ(tv2, 50);
    r.Tick(still);
    CHECK(r.Pressure(rc.v1) > 0);
    if (rc.v2 != CL_B) // B (F11 V2) has no neutral pre-fill here
      CHECK_EQ(r.Pressure(rc.v2), 0);
    for (int i = 0; i < tv2 / 10 + 1; i++)
      r.Tick(still);
    CHECK(r.Pressure(rc.v2) > 0);
  }

  // The fast close waits for the no-slip confirm time.
  Engage f;
  CHECK(f.Start(GEAR_1, Cfg(ENG_VAR_330E)));
  EngageInputs in = still;
  in.n_t = 300;
  for (int i = 0; i < 100; i++)
    f.Tick(in);
  CHECK(!f.FastClose());
  in.n_t = 0;
  for (int i = 0; i < 7; i++) {
    f.Tick(in);
    CHECK(!f.FastClose());
  }
  f.Tick(in);
  CHECK(f.FastClose()); // 80 ms of no slip
  in.m_t = 50;          // torque comes back: drop out of the fast close
  f.Tick(in);
  CHECK(!f.FastClose());
}
