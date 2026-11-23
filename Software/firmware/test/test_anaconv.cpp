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
#include "anaconv.h"
#include "speedsens.h"
#include "test.h"
#include <stdlib.h>

/// Counts an ideal ADC would read for sensor resistance rs on a V4
/// temperature input (pull-up rpu to 5 V, 10k/10k divider).
static int32_t TempCounts(int64_t rs, int64_t rpu) {
  int64_t rp = rs * 20000 / (rs + 20000);
  int64_t vin = 5000 * rp / (rpu + rp);
  return (int32_t)((vin / 2) * 4095 / 3300);
}

static bool Near(int32_t a, int32_t b, int32_t pct) {
  return llabs((long long)a - b) * 100 <= (long long)b * pct;
}

void TestAnaConv() {
  using namespace AnaConv;
  // mV <-> counts, 3.3 V reference
  CHECK_EQ(ToMv(0), 0);
  CHECK_EQ(ToMv(4095), 3300);
  CHECK_EQ(ToCounts(3300), 4095);
  CHECK_EQ(ToCounts(1650), 2048);
  CHECK_EQ(ToCounts(-5), 0);

  // Battery: 18.33 V full scale (Bob: 0.18 divider, V = Vadc x 5.556)
  CHECK_EQ(VbatMv(4095, 18330), 18330);
  CHECK(Near(VbatMv(ToCounts(12000 * 18 / 100), 18330), 12000, 1));

  // Temperature inputs: TempIn1 2.6k pull-up, TempIn2 470R pull-up.
  const int32_t rs[] = {100, 470, 1180, 2500, 10000};
  for (int32_t r : rs) {
    CHECK(Near(TempInputOhms(TempCounts(r, 2600), 2600), r, 3));
    CHECK(Near(TempInputOhms(TempCounts(r, 470), 470), r, 6));
  }
  // Open sensor: only the 20k divider is left -> -1 or far above any
  // sensor value (AtfRMax catches it)
  {
    int32_t open = TempInputOhms(TempCounts(100000000, 2600), 2600);
    CHECK(open == -1 || open > 1000000);
  }
  CHECK_EQ(TempInputOhms(4095, 2600), -1);
  CHECK_EQ(TempInputOhms(0, 2600), 0); // short

  // Generic NTC table (default parameters): falling resistance with T
  const int32_t rPts[] = {70, 190, 450, 1180, 3800, 26000};
  const int32_t tPts[] = {140, 100, 70, 40, 10, -30};
  CHECK_EQ(TempFromOhms(1180, rPts, tPts, 6), 40);
  CHECK_EQ(TempFromOhms(10, rPts, tPts, 6), 140);      // clamp hot end
  CHECK_EQ(TempFromOhms(90000, rPts, tPts, 6), -30);   // clamp cold end
  CHECK_EQ(TempFromOhms(815, rPts, tPts, 6), 55);      // halfway 450..1180
  // Ascending table works as well (PTC-style sensor)
  const int32_t rUp[] = {500, 1000};
  const int32_t tUp[] = {0, 100};
  CHECK_EQ(TempFromOhms(750, rUp, tUp, 2), 50);

  // Speed sensor edge finder: 0.7 V low / 1.4 V high, thresholds 0.9 /
  // 1.2 V. Noise between the thresholds must not make extra edges.
  AdcEdge e;
  e.Configure(ToCounts(900), ToCounts(1200));
  const int32_t lo = ToCounts(700), hi = ToCounts(1400), mid = ToCounts(1050);
  int edges = 0;
  for (int cyc = 0; cyc < 10; cyc++) {
    int32_t wave[] = {lo, lo, mid, hi, mid, hi, hi, mid, lo, mid, lo};
    for (int32_t v : wave)
      edges += e.Sample((uint16_t)v);
  }
  CHECK_EQ(edges, 10);
  // A signal that never goes below the low threshold gives one edge only
  AdcEdge e2;
  e2.Configure(ToCounts(900), ToCounts(1200));
  edges = 0;
  for (int i = 0; i < 20; i++)
    edges += e2.Sample((uint16_t)(i & 1 ? hi : mid));
  CHECK_EQ(edges, 1);

  // End to end: a 1000 rpm, 60-tooth square wave sampled at 40 kHz
  // (20 samples high, 20 low) through AdcEdge into SpeedSensor.
  SpeedSensor s;
  s.Configure(40000, 60, 100, 30);
  AdcEdge e3;
  e3.Configure(ToCounts(900), ToCounts(1200));
  uint32_t n = 0, last = 0;
  bool first = true;
  for (int win = 0; win < 3; win++) {
    for (int i = 0; i < 400; i++, n++) { // 10 ms
      if (e3.Sample((uint16_t)((n / 20) & 1 ? hi : lo))) {
        if (!first)
          s.Edge(n - last);
        first = false;
        last = n;
      }
    }
    s.Tick10ms();
  }
  CHECK_EQ(s.Rpm(), 1000);
}
