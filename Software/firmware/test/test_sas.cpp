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
};

static void Push(std::vector<int> &v, int s) {
  if (v.empty() || v.back() != s)
    v.push_back(s);
}

static SasConfig Cfg() {
  SasConfig c;
  c.pMax = 12750;
  c.syncBand = 30;
  c.glsGainPct = 100;
  c.rampMaxDownMs = 1000;
  return c;
}

static ShiftRun Run(int from, int to, int32_t m_t, bool plantMoves) {
  const int32_t nab = 1000;
  int32_t nt = Gears::SyncSpeed(from, nab);
  int32_t target = Gears::SyncSpeed(to, nab);
  const int32_t rate = 3000; // rpm/s
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
  for (int i = 0; i < 1000 && s.Active(); i++) {
    const SasOutputs &o = s.Out();
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
    s.Tick(inputs());
    r.ms += 10;
  }
  Push(r.kzu, s.Out().zKzu);
  Push(r.kab, s.Out().zKab);
  if (s.Out().pKzu > r.maxKzu)
    r.maxKzu = s.Out().pKzu;
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
  CHECK(Seq(r.kab, {11, 12, 13, 14, 15, 16}));
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

void TestSas() {
  TestUpshift12();
  TestDownshift21();
  TestAllPairsFinish();
  TestTimeout();
  TestRejects();
}
