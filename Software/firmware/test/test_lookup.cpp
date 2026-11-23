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
#include "caltypes.h"
#include "test.h"

void TestLookup() {
  static const int16_t cx[] = {0, 100, 200};
  static const int32_t cy[] = {10, 20, 0};
  CalCurve c = {3, cx, cy};
  CHECK_EQ(Cal::Curve(c, -50), 10);  // clamped low
  CHECK_EQ(Cal::Curve(c, 0), 10);
  CHECK_EQ(Cal::Curve(c, 50), 15);
  CHECK_EQ(Cal::Curve(c, 150), 10);
  CHECK_EQ(Cal::Curve(c, 200), 0);
  CHECK_EQ(Cal::Curve(c, 999), 0);   // clamped high

  // z[iy * nx + ix], x is the first axis
  static const int16_t mx[] = {0, 10};
  static const int16_t my[] = {0, 100};
  static const int32_t mz[] = {0, 100,    // y = 0
                               1000, 1100}; // y = 100
  CalMap m = {2, 2, mx, my, mz};
  CHECK_EQ(Cal::Map(m, 0, 0), 0);
  CHECK_EQ(Cal::Map(m, 10, 0), 100);
  CHECK_EQ(Cal::Map(m, 0, 100), 1000);
  CHECK_EQ(Cal::Map(m, 5, 50), 550);
  CHECK_EQ(Cal::Map(m, 20, 200), 1100);
  CHECK_EQ(Cal::Map(m, -5, -5), 0);

  // Interp with a falling table (NTC style)
  const int32_t xs[] = {100, 200, 300};
  const int32_t ys[] = {90, 50, -10};
  CHECK_EQ(Cal::Interp(xs, ys, 3, 150), 70);
  CHECK_EQ(Cal::Interp(xs, ys, 3, 250), 20);
  CHECK_EQ(Cal::Interp(xs, ys, 3, 0), 90);
  CHECK_EQ(Cal::Interp(xs, ys, 3, 400), -10);

  // Single point curve
  static const int16_t sx[] = {5};
  static const int32_t sy[] = {42};
  CalCurve s = {1, sx, sy};
  CHECK_EQ(Cal::Curve(s, -100), 42);
  CHECK_EQ(Cal::Curve(s, 100), 42);
}
