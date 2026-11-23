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
#include "speedsens.h"
#include "anaconv.h"
#include "test.h"

/* Rev B: three channels scanned back to back by ADC1 + DMA at 153.8 kHz
 * (12 MHz / 78 cycles), processed a half-buffer (64 scans) at a time. */
static void TestScanEdges() {
  const uint32_t hz = 153846;
  const int HALF = 64;
  SpeedSensor a, b, c;
  a.Configure(hz, 60, 100, 30);
  b.Configure(hz, 60, 100, 30);
  c.Configure(hz, 60, 100, 30);
  SpeedSensor *const sens[3] = {&a, &b, &c};
  ScanEdges se;
  uint16_t lo = (uint16_t)AnaConv::ToCounts(1000), hi = (uint16_t)AnaConv::ToCounts(1100);
  for (int i = 0; i < 3; i++)
    se.edge[i].Configure(lo, hi);
  const uint16_t vLow = (uint16_t)AnaConv::ToCounts(700), vHigh = (uint16_t)AnaConv::ToCounts(1400);
  // ch0 1000 rpm (1 kHz), ch1 10000 rpm (10 kHz, the fastest Bob sized
  // the RC for), ch2 standing still.
  const double fHz[3] = {1000.0, 10000.0, 0.0};
  uint16_t buf[2 * HALF * 3];
  uint32_t scan = 0;
  for (int tick = 0; tick < 30; tick++) {
    // 10 ms = 1538.46 scans; run whole halves until the time is reached
    while (scan * 100.0 / hz < (tick + 1)) {
      for (int h = 0; h < 2; h++) {
        uint16_t *blk = buf + h * HALF * 3;
        for (int k = 0; k < HALF; k++, scan++) {
          double t = scan / (double)hz;
          for (int ch = 0; ch < 3; ch++) {
            double ph = fHz[ch] * t;
            ph -= (long)ph;
            blk[k * 3 + ch] = (fHz[ch] > 0 && ph < 0.5) ? vHigh : vLow;
          }
        }
        se.Block(blk, HALF, 3, sens);
      }
    }
    a.Tick10ms();
    b.Tick10ms();
    c.Tick10ms();
  }
  CHECK(a.Valid());
  CHECK(a.Rpm() >= 990 && a.Rpm() <= 1010);
  CHECK(b.Valid());
  CHECK(b.Rpm() >= 9900 && b.Rpm() <= 10100);
  CHECK(!c.Valid());
  CHECK_EQ(c.Rpm(), 0);
  CHECK_EQ(se.Scans(), scan);
  CHECK_EQ(se.Min(0), vLow);
  CHECK_EQ(se.Max(0), vHigh);
  CHECK_EQ(se.Max(2), vLow);
  se.ResetLevels();
  CHECK_EQ(se.Min(0), 4095);
  CHECK_EQ(se.Max(0), 0);

  // The block loop must find exactly the edges AdcEdge::Sample finds,
  // noise around the thresholds included (hysteresis), across blocks.
  {
    SpeedSensor x;
    x.Configure(hz, 60, 100, 0);
    SpeedSensor *const one[1] = {&x};
    x.Edge(1); // arm: the next window counts every edge
    x.Tick10ms();
    ScanEdges sb;
    AdcEdge ref;
    sb.edge[0].Configure(1200, 1400);
    ref.Configure(1200, 1400);
    uint32_t seed = 12345, refEdges = 0, refSum = 0, lastRef = 0, cnt = 0;
    uint16_t blk[HALF];
    for (int b = 0; b < 200; b++) {
      for (int k = 0; k < HALF; k++) {
        seed = seed * 1103515245u + 12345u;
        uint16_t v = (uint16_t)(1000 + (seed >> 16) % 600); // 1000..1599
        blk[k] = v;
        cnt++;
        if (ref.Sample(v)) {
          refEdges++;
          refSum += cnt - lastRef;
          lastRef = cnt;
        }
      }
      sb.Block(blk, HALF, 1, one);
    }
    CHECK(refEdges > 100);
    // Same edges, same periods: the averaged rpm must match exactly.
    x.Tick10ms();
    int32_t want = (int32_t)(((uint64_t)hz * 60u * refEdges) / ((uint64_t)refSum * 60));
    CHECK_EQ(x.Rpm(), want);
    CHECK_EQ(sb.edge[0].High(), ref.High());
  }
}

void TestSpeedSens() {
  // 60 teeth, 1000 rpm -> 1000 edges/s -> 500 ticks at 500 kHz
  CHECK_EQ(SpeedSensor::PeriodToRpm(500000, 500, 60), 1000);
  CHECK_EQ(SpeedSensor::PeriodToRpm(500000, 0, 60), 0);

  SpeedSensor s;
  s.Configure(500000, 60, 100, 30);
  CHECK(!s.Valid());
  // First edge after start-up is dropped (period may span a wrap)
  s.Edge(500);
  s.Tick10ms();
  CHECK_EQ(s.Rpm(), 0);
  for (int i = 0; i < 10; i++)
    s.Edge(500);
  s.Tick10ms();
  CHECK_EQ(s.Rpm(), 1000);
  CHECK(s.Valid());
  // Tooth count is a parameter: same edges with 30 teeth = double speed
  s.Configure(500000, 30, 100, 30);
  for (int i = 0; i < 10; i++)
    s.Edge(500);
  s.Tick10ms();
  CHECK_EQ(s.Rpm(), 2000);
  // Averaging over the window
  s.Configure(500000, 60, 100, 30);
  s.Edge(400);
  s.Edge(600);
  s.Tick10ms();
  CHECK_EQ(s.Rpm(), 1000);
  // Holds through short gaps, 0 after the timeout
  for (int i = 0; i < 9; i++)
    s.Tick10ms();
  CHECK_EQ(s.Rpm(), 1000);
  s.Tick10ms();
  CHECK_EQ(s.Rpm(), 0);
  CHECK(!s.Valid());
  // Below the minimum speed reads 0: 20 rpm = 25000 ticks per tooth
  s.Edge(25000);
  s.Tick10ms();
  s.Edge(25000);
  s.Tick10ms();
  CHECK_EQ(s.Rpm(), 0);

  // V4 time base: ADC2 samples at 40 kHz. 60 teeth at 1000 rpm =
  // 1000 edges/s = 40 samples per edge.
  s.Configure(40000, 60, 100, 30);
  s.Edge(40);
  s.Tick10ms();
  for (int i = 0; i < 10; i++)
    s.Edge(40);
  s.Tick10ms();
  CHECK_EQ(s.Rpm(), 1000);

  TestScanEdges();
}
