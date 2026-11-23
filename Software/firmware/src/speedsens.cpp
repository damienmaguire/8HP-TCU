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
/*
 * Speed sensors: edge periods from the timer capture to rpm, with a
 * timeout so a dead sensor reads 0 and not valid.
 */
#include "speedsens.h"

// No edges yet: speed 0, not valid.
SpeedSensor::SpeedSensor()
    : timerHz(1000000), teeth(60), timeoutMs(100), minRpm(30), sumTicks(0),
      count(0), edgeSeen(false), msSinceEdge(0xFFFF), rpm(0), valid(false) {}

// Timer rate, teeth per turn, timeout and minimum speed.
void SpeedSensor::Configure(uint32_t hz, uint16_t t, uint16_t timeout,
                            uint16_t minR) {
  timerHz = hz;
  teeth = t ? t : 1;
  timeoutMs = timeout;
  minRpm = minR;
}

// One edge with the time since the last one, in timer ticks.
void SpeedSensor::Edge(uint32_t periodTicks) {
  edgeSeen = true;
  // After a timeout the period may span a timer wrap: drop it. The next
  // edge starts counting again.
  if (periodTicks == 0 || msSinceEdge >= timeoutMs)
    return;
  sumTicks += periodTicks;
  count++;
}

// Edge period to rpm for a tooth count.
int32_t SpeedSensor::PeriodToRpm(uint32_t hz, uint32_t period, uint16_t t) {
  if (period == 0 || t == 0)
    return 0;
  return (int32_t)(((uint64_t)hz * 60u) / ((uint64_t)period * t));
}

// Speed 0 and not valid once edges stop for the timeout.
void SpeedSensor::Tick10ms() {
  uint32_t s = sumTicks, n = count;
  bool seen = edgeSeen;
  sumTicks = 0;
  count = 0;
  edgeSeen = false;
  if (n == 0 && seen) {
    msSinceEdge = 0; // first edge after a timeout: speed follows next window
  } else if (n > 0) {
    msSinceEdge = 0;
    // Average period over the window: sum / n.
    rpm = (int32_t)(((uint64_t)timerHz * 60u * n) / ((uint64_t)s * teeth));
    valid = true;
  } else {
    if (msSinceEdge < 0xFFFF)
      msSinceEdge += 10;
    // No edge in this window: keep the last speed until the timeout.
    if (msSinceEdge >= timeoutMs) {
      rpm = 0;
      valid = false;
    }
  }
  if (rpm < minRpm)
    rpm = 0;
}

// No scans yet, levels window empty.
ScanEdges::ScanEdges() : n(0) {
  for (int c = 0; c < SCAN_MAX_CH; c++)
    last[c] = 0;
  ResetLevels();
}

// Empty the min / max window.
void ScanEdges::ResetLevels() {
  for (int c = 0; c < SCAN_MAX_CH; c++) {
    mn[c] = 4095;
    mx[c] = 0;
  }
}

// Inner loop of Block(): min / max until the threshold test hits.
// Returns the sample that hit, or end. Few live values, so it all stays in
// registers (README "CPU load"). Not inlined: inlined into Block(), GCC
// -Os runs out of registers and keeps lo / hi on the stack.
__attribute__((noinline)) static const uint16_t *ScanRun(const uint16_t *p, const uint16_t *end,
                                                         int step, uint32_t mode,
                                                         uint32_t *lohi) {
  // mode = flip << 16 | thr: four register arguments, so nothing in the
  // loop comes from the stack.
  const uint32_t flip = mode >> 16, thr = mode & 0xFFFFu;
  uint32_t l = lohi[0], h = lohi[1];
  for (; p < end; p += step) {
    uint32_t v = *p;
    l = v < l ? v : l;
    h = v > h ? v : h;
    if ((v ^ flip) >= thr)
      break;
  }
  lohi[0] = l;
  lohi[1] = h;
  return p;
}

// One DMA half-buffer: edges and levels, channel by channel. Runs about
// 2400 times a second, so the inner loop is kept to a load, min / max and
// one compare: the AdcEdge test "at or below lo while high / at or above
// hi while low" becomes (v ^ flip) >= thr, with flip = 0xFFFF while high
// (0xFFFF - v >= 0xFFFF - lo is v <= lo). Same result as AdcEdge::Sample
// (host test TestScanEdges).
void ScanEdges::Block(const volatile uint16_t *buf, int scans, int nch,
                      SpeedSensor *const *sens) {
  if (nch > SCAN_MAX_CH)
    nch = SCAN_MAX_CH;
  // The DMA is filling the other half: this half is stable while we read.
  const uint16_t *base = (const uint16_t *)buf;
  for (int c = 0; c < nch; c++) {
    AdcEdge &e = edge[c];
    const uint32_t thrRise = e.Hi(), thrFall = 0xFFFFu - e.Lo();
    bool high = e.High();
    uint32_t lohi[2] = {mn[c], mx[c]};
    const uint16_t *first = base + c;
    const uint16_t *end = first + scans * nch;
    const uint16_t *p = first;
    while ((p = ScanRun(p, end, nch, high ? (0xFFFF0000u | thrFall) : thrRise, lohi)) < end) {
      high = !high;
      if (high) {
        uint32_t at = n + (uint32_t)((p - first) / nch) + 1;
        sens[c]->Edge(at - last[c]); // dropped inside after a timeout
        last[c] = at;
      }
      p += nch;
    }
    e.SetHigh(high);
    mn[c] = (uint16_t)lohi[0];
    mx[c] = (uint16_t)lohi[1];
  }
  n += (uint32_t)scans;
}
