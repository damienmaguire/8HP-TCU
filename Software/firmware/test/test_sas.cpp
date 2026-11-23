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
#include "cal_330e.h"
#include "gears.h"
#include "sas.h"
#include "test.h"
#include <vector>

/* Simple drivetrain model for the shift sequences: output speed fixed,
 * input speed moves towards the new gear's sync at a fixed rate while the
 * shift is in its inertia phase. */
struct ShiftRun {
  std::vector<int> kzu, kab;
  bool done = false, timedOut = false, sawTq = false;
  int ms = 0;
  int32_t maxKzu = 0;
  std::vector<int> glsKzu, glsKab; // states seen while the GLS was non-zero
  int syncToIdleMs = -1;           // upshift: from reaching sync to KZU idle
  bool coast = false;
  int kzuAtKabEnd = -1;    // on-coming state when the off-going let go
  bool kabDroppedEarly = false; // off-going at 0 while on-coming still fills
};

static void Push(std::vector<int> &v, int s) {
  if (v.empty() || v.back() != s)
    v.push_back(s);
}

static SasConfig Cfg() {
  SasConfig c = {};
  c.pMax = 12750;
  c.syncBand = 30;
  c.glsGainPct = 100;
  c.rampMaxDownMs = 1000;
  return c;
}

static ShiftRun Run(int from, int to, int32_t m_t, bool plantMoves,
                    int32_t rate = 3000 /* rpm/s */) {
  const int32_t nab = 1000;
  int32_t nt = Gears::SyncSpeed(from, nab);
  int32_t target = Gears::SyncSpeed(to, nab);
  int off, on;
  Gears::SingleSwap(from, to, off, on);
  SasShift s;
  ShiftRun r;
  bool moving = false;
  auto inputs = [&]() {
    SasInputs in;
    in.m_t = m_t;
    in.n_t = nt;
    in.n_mot = nt;
    in.temp = 60;
    in.ndSyn = nt - target;
    in.ndSynIst = nt - Gears::SyncSpeed(from, nab);
    in.dnt = moving ? (target > nt ? rate : -rate) : 0;
    int32_t left = target - nt;
    in.tSyn = moving && left != 0 ? (left < 0 ? -left : left) * 1000 / rate
                                  : SAS_TSYN_NONE;
    return in;
  };
  CHECK(s.Start(from, to, off, on, Cfg(), inputs()));
  r.coast = s.IsCoast();
  int syncAt = -1;
  for (int i = 0; i < 1000 && s.Active(); i++) {
    const SasOutputs &o = s.Out();
    if (syncAt < 0 && o.zKzu == KZU_RAMP3 && nt == target)
      syncAt = r.ms;
    if (syncAt >= 0 && r.syncToIdleMs < 0 && o.zKzu == KZU_IDLE)
      r.syncToIdleMs = r.ms - syncAt;
    Push(r.kzu, o.zKzu);
    Push(r.kab, o.zKab);
    if (o.pKzu > r.maxKzu)
      r.maxKzu = o.pKzu;
    if (o.tqDelta != 0)
      r.sawTq = true;
    // Once the speed change has started it carries on to the new sync.
    if (plantMoves && o.inertia)
      moving = true;
    if (nt == target)
      moving = false;
    if (moving) {
      int32_t step = rate / 100;
      if (nt < target)
        nt = nt + step > target ? target : nt + step;
      else if (nt > target)
        nt = nt - step < target ? target : nt - step;
    }
    // Note the states a tick started in, and whether it ran the GLS.
    int pk = o.zKzu, pa = o.zKab;
    s.Tick(inputs());
    if (r.kzuAtKabEnd < 0 && s.Out().zKab == KAB_END)
      r.kzuAtKabEnd = s.Out().zKzu;
    {
      int z = s.Out().zKzu;
      bool filling = z == KZU_WAIT || z == KZU_QUICK_FILL || z == KZU_FILL_EQ_UP ||
                     z == KZU_FILL_EQ_DOWN;
      if (filling && s.Out().zKab != KAB_END && s.Active() && s.Out().pKab <= 0)
        r.kabDroppedEarly = true;
    }
    if (s.Out().gls != 0) {
      Push(r.glsKzu, pk);
      Push(r.glsKab, pa);
    }
    r.ms += 10;
  }
  Push(r.kzu, s.Out().zKzu);
  Push(r.kab, s.Out().zKab);
  if (s.Out().pKzu > r.maxKzu)
    r.maxKzu = s.Out().pKzu;
  if (syncAt >= 0 && r.syncToIdleMs < 0 && s.Out().zKzu == KZU_IDLE)
    r.syncToIdleMs = r.ms - syncAt;
  r.done = s.Out().done;
  r.timedOut = s.Out().timedOut;
  return r;
}

static bool Seq(const std::vector<int> &got, std::initializer_list<int> want) {
  bool same = got.size() == want.size();
  size_t i = 0;
  for (int w : want)
    if (same && got[i++] != w)
      same = false;
  if (!same) {
    printf("  got sequence:");
    for (int v : got)
      printf(" %d", v);
    printf("\n");
  }
  return same;
}

static void TestUpshift12() {
  ShiftRun r = Run(1, 2, 150, true);
  CHECK(r.done);
  CHECK(!r.timedOut);
  // OEM KZU states: quick fill, fill-equalise, ramp 1, ramp 2, ramp 3, idle.
  // The 1->2 delays T_VKZU12 and T_VKABHS12 are 0, so wait (1) and the
  // KAB delay (1) are left in the first tick, inside Start().
  CHECK(Seq(r.kzu, {2, 4, 9, 10, 13, 0}));
  // OEM KAB states: hold, ramp, shift pressure, takeover, end
  CHECK(Seq(r.kab, {2, 3, 4, 9, 16}));
  CHECK(r.maxKzu <= 12750);
  CHECK(r.ms < 2700); // inside KL_TMAXCG
}

static void TestDownshift21() {
  ShiftRun r = Run(2, 1, 150, true);
  CHECK(r.done);
  CHECK(!r.timedOut);
  // KZU: quick fill, fill-equalise, ramp, hold to sync, close, idle
  CHECK(Seq(r.kzu, {2, 5, 9, 12, 18, 0}));
  // KAB: (hold 10 is left inside Start) ramp, slip, ramp to sync, hold in
  // sync, overlap, end
  CHECK(Seq(r.kab, {10, 11, 12, 13, 14, 15, 16}));
}

static void TestAllPairsFinish() {
  for (int g = 1; g < 8; g++) {
    ShiftRun up = Run(g, g + 1, 100, true);
    CHECK(up.done);
    CHECK(!up.timedOut);
    ShiftRun dn = Run(g + 1, g, 100, true);
    CHECK(dn.done);
    CHECK(!dn.timedOut);
  }
}

/* Overlap (TIS "all shifts overlap"): the off-going clutch keeps some
 * pressure until the on-coming one is past its fill and taking torque. */
static void TestOverlap() {
  for (int g = 1; g < 8; g++) {
    for (int d = 0; d < 2; d++) {
      ShiftRun r = d ? Run(g + 1, g, 150, true) : Run(g, g + 1, 150, true);
      CHECK(r.done);
      CHECK(!r.kabDroppedEarly);
      CHECK(r.kzuAtKabEnd != KZU_WAIT && r.kzuAtKabEnd != KZU_QUICK_FILL &&
            r.kzuAtKabEnd != KZU_FILL_EQ_UP && r.kzuAtKabEnd != KZU_FILL_EQ_DOWN);
    }
  }
}

static void TestTimeout() {
  // Input speed never moves: the shift must time out and still finish
  // with the on-coming element closed.
  ShiftRun r = Run(1, 2, 150, false);
  CHECK(r.timedOut);
  CHECK(r.done);
  CHECK_EQ(r.maxKzu, 12750);
  ShiftRun d = Run(2, 1, 150, false);
  CHECK(d.timedOut);
  CHECK(d.done);
}

static void TestRejects() {
  SasShift s;
  SasInputs in = {};
  CHECK(!s.Start(1, 3, CL_A, CL_E, Cfg(), in)); // not adjacent
  CHECK(!s.Start(0, 1, CL_A, CL_E, Cfg(), in));
  CHECK(!s.Active());
}


/* GLS scaling (Ian, QA2): KP 1 count = 0.02 mbar per rpm/s, KI 1 count =
 * 0.02/64 mbar per rpm/s per 10 ms, output x50/64. */
static void TestGlsScaling() {
  // 50 KP counts = 1 mbar per rpm/s; 64 rpm/s -> 64 mbar -> x50/64 = 50.
  CHECK_EQ(SasShift::GlsScale(50, 64, 0), 50);
  CHECK_EQ(SasShift::GlsScale(50, -64, 0), -50);
  CHECK_EQ(SasShift::GlsScale(1, 1000, 0), 15); // 20 mbar x 50/64 = 15.6
  // KI: 3200 x 64 accumulated counts = 64 mbar -> 50 mbar.
  CHECK_EQ(SasShift::GlsScale(0, 0, 3200 * 64), 50);
  // OEM 1->2: KP 2, KI 27. 1000 rpm/s error for one step:
  // P = 40 mbar, I = 27000/3200 = 8.4 mbar, sum x50/64 = 37 mbar.
  CHECK_EQ(calUp[0].kp_glskzu, 2);
  CHECK_EQ(calUp[0].ki_glskzu, 27);
  CHECK_EQ(SasShift::GlsScale(2, 1000, 27 * 1000), 37);
  // The +/-3 bar clamp comes from P_MAXOBGLS / P_MAXUNGLS.
  CHECK_EQ(calGlobal.p_maxobgls, 3000);
  CHECK_EQ(calGlobal.p_maxungls, 3000);
  // KAB start delay KL_TVGLSKABCG12 is 250/150/50 ms over -15/0/30 degC.
  CHECK_EQ(Cal::Curve(calUp[0].tvglskabcg, -15), 250);
  CHECK_EQ(Cal::Curve(calUp[0].tvglskabcg, 0), 150);
  CHECK_EQ(Cal::Curve(calUp[0].tvglskabcg, 30), 50);
  // Other QA2 gain examples: 1->2 KAB KP 3 / KI 11, 2->1 KZU 2 / 8,
  // 2->1 KAB 1 / 13.
  CHECK_EQ(calUp[0].kp_glskab, 3);
  CHECK_EQ(calUp[0].ki_glskab, 11);
  CHECK_EQ(calDown[0].kp_glskzu, 2);
  CHECK_EQ(calDown[0].ki_glskzu, 8);
  CHECK_EQ(calDown[0].kp_glskab, 1);
  CHECK_EQ(calDown[0].ki_glskab, 13);
  // Start criterion FUN_0010e60e: KL_NDGLSPOSHS12 350 rpm, KL_NDGLSNEGHS12
  // -160..-300 rpm over torque, ND_GLSRS21 150 rpm.
  CHECK_EQ(Cal::Curve(calUp[0].ndglsposhs, 100), 350);
  CHECK_EQ(Cal::Curve(calUp[0].ndglsneghs, 50), -160);
  CHECK_EQ(Cal::Curve(calUp[0].ndglsneghs, 350), -300);
  CHECK_EQ(calDown[0].nd_glsrs, 150);
  // RS-KAB block: KL_POHKAB21 0.4..1.0 bar, KL_TRAHKAB21 100 ms, both
  // factors 1.0.
  CHECK_EQ(Cal::Curve(calDown[0].pohkab, 24), 400);
  CHECK_EQ(Cal::Curve(calDown[0].pohkab, 448), 1000);
  CHECK_EQ(Cal::Curve(calDown[0].trahkab, 0), 100);
  CHECK_EQ(calDown[0].f_trahkab, 100);
  CHECK_EQ(calDown[0].f_prahkab, 100);
  // KZU start delay KL_TVGLSKZUCG12 is 180-200 ms.
  CHECK(Cal::Curve(calUp[0].tvglskzucg, 60) >= 180);
  CHECK(Cal::Curve(calUp[0].tvglskzucg, -20) <= 200);
}

/* Which clutch carries the slip controller: power-on upshift KZU, coast
 * upshift KAB, power-on downshift KAB, coast downshift KZU. */
static void TestGlsOwner() {
  ShiftRun up = Run(1, 2, 150, true);
  CHECK(!up.coast);
  CHECK(Seq(up.glsKzu, {KZU_RAMP2}));
  ShiftRun upC = Run(1, 2, -40, true);
  CHECK(upC.coast);
  CHECK(upC.done);
  CHECK(!upC.timedOut);
  for (int s : upC.glsKab)
    CHECK(s == KAB_SHIFT_P_UP || s == KAB_TAKEOVER);
  CHECK(!upC.glsKab.empty());
  ShiftRun dn = Run(2, 1, 150, true);
  CHECK(!dn.coast);
  CHECK(Seq(dn.glsKab, {KAB_SLIP_DOWN}));
  // Slower speed change so the KZU start delay (KL_TVGLSKZUCG21) passes
  // before sync in this simple plant.
  ShiftRun dnC = Run(2, 1, -40, true, 1000);
  CHECK(dnC.coast);
  CHECK(dnC.done);
  for (int s : dnC.glsKzu)
    CHECK(s == KZU_HOLD_SYNC);
  CHECK(!dnC.glsKzu.empty());
}

/* No third (takeover) ramp: after sync KL_PGRASYN runs for T_SYN, then
 * KL_PORASYN up to PMAX, then the on-coming clutch is done. T_RAMAX is the
 * timeout ramp only (10 ms on 1->2). */
static void TestUpshiftAfterSync() {
  CHECK_EQ(calUp[0].t_syn, 300);
  CHECK_EQ(calUp[0].t_ramax, 10);
  const CalCurve &por = calUp[0].porasyn;
  CHECK_EQ(por.n, 3);
  CHECK_EQ(por.x[2], 450);
  CHECK_EQ(por.y[0], 100);
  CHECK_EQ(por.y[2], 12750);
  ShiftRun r = Run(1, 2, 150, true);
  CHECK(r.done);
  // (sync is detected inside SyncBand, a tick or so before the plant's
  // exact sync, hence the -30 ms)
  CHECK(r.syncToIdleMs >= calUp[0].t_syn + por.x[2] - 30);
  CHECK(r.syncToIdleMs <= calUp[0].t_syn + por.x[2] + 40);
  CHECK_EQ(r.maxKzu, 12750);
}

// Learned fill offsets (adapt.h) add to the on-coming clutch's fill
// pressure only, up and down.
static void TestFillOffset() {
  const int pairs[2][2] = {{3, 4}, {6, 5}};
  for (auto &pr : pairs) {
    int off, on;
    Gears::SingleSwap(pr[0], pr[1], off, on);
    int16_t ofs[5] = {0, 0, 0, 0, 0};
    ofs[on] = 120;
    ofs[off] = -999; // must not be used
    SasConfig ca = Cfg(), cb = Cfg();
    cb.fillOfs = ofs;
    SasInputs in = {};
    in.m_t = 100;
    in.n_t = Gears::SyncSpeed(pr[0], 1000);
    in.n_mot = in.n_t;
    in.temp = 60;
    in.ndSyn = in.n_t - Gears::SyncSpeed(pr[1], 1000);
    in.tSyn = SAS_TSYN_NONE;
    SasShift a, b;
    CHECK(a.Start(pr[0], pr[1], off, on, ca, in));
    CHECK(b.Start(pr[0], pr[1], off, on, cb, in));
    int fillState = pr[0] < pr[1] ? KZU_FILL_EQ_UP : KZU_FILL_EQ_DOWN;
    bool seen = false;
    int inState = 0; // the first tick in the state still outputs quick fill
    for (int i = 0; i < 300 && !seen; i++) {
      a.Tick(in);
      b.Tick(in);
      inState = a.Out().zKzu == fillState && b.Out().zKzu == fillState ? inState + 1 : 0;
      if (inState == 2) {
        CHECK_EQ(b.Out().pKzu - a.Out().pKzu, 120);
        CHECK_EQ(b.Out().pKab, a.Out().pKab);
        seen = true;
      }
    }
    CHECK(seen);
  }
}

void TestSas() {
  TestFillOffset();
  TestGlsScaling();
  TestGlsOwner();
  TestUpshiftAfterSync();
  TestUpshift12();
  TestDownshift21();
  TestAllPairsFinish();
  TestOverlap();
  TestTimeout();
  TestRejects();
}
