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

SpeedSensor::SpeedSensor()
    : timerHz(1000000), teeth(60), timeoutMs(100), minRpm(30), sumTicks(0),
      count(0), edgeSeen(false), msSinceEdge(0xFFFF), rpm(0), valid(false) {}

void SpeedSensor::Configure(uint32_t hz, uint16_t t, uint16_t timeout,
                            uint16_t minR) {
  timerHz = hz;
  teeth = t ? t : 1;
  timeoutMs = timeout;
  minRpm = minR;
}

void SpeedSensor::Edge(uint32_t periodTicks) {
  edgeSeen = true;
  // After a timeout the period may span a timer wrap: drop it. The next
  // edge starts counting again.
  if (periodTicks == 0 || msSinceEdge >= timeoutMs)
    return;
  sumTicks += periodTicks;
  count++;
}

int32_t SpeedSensor::PeriodToRpm(uint32_t hz, uint32_t period, uint16_t t) {
  if (period == 0 || t == 0)
    return 0;
  return (int32_t)(((uint64_t)hz * 60u) / ((uint64_t)period * t));
}

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
