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
#include "test.h"

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

  // ATF table: falling NTC-style curve
  const int32_t a[] = {400, 1000, 1800, 2600, 3300, 3900};
  const int32_t t[] = {140, 100, 70, 40, 10, -30};
  CHECK_EQ(AtfTemp::FromAdc(400, a, t, 6), 140);
  CHECK_EQ(AtfTemp::FromAdc(2200, a, t, 6), 55);
  CHECK_EQ(AtfTemp::FromAdc(4095, a, t, 6), -30);
}
