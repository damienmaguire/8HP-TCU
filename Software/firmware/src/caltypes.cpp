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

/* Find the segment that contains x and return the index of its left point
 * plus the fraction inside it, as a numerator over den (0..den).
 */
template <typename T>
// Find the axis interval holding x and how far along it x is (for interpolation).
static int Segment(const T *axis, int n, int32_t x, int32_t &num,
                   int32_t &den) {
  if (n <= 1 || x <= axis[0]) {
    num = 0;
    den = 1;
    return 0;
  }
  if (x >= axis[n - 1]) {
    num = 1;
    den = 1;
    return n - 2;
  }
  int i = 0;
  while (i < n - 2 && x >= axis[i + 1])
    i++;
  den = (int32_t)axis[i + 1] - (int32_t)axis[i];
  num = x - axis[i];
  if (den <= 0) { // flat or broken axis: take the left point
    num = 0;
    den = 1;
  }
  return i;
}

/* a + (b - a) * num / den, in 64 bit so large mbar/s values cannot overflow. */
static int32_t Lerp(int32_t a, int32_t b, int32_t num, int32_t den) {
  return a + (int32_t)(((int64_t)(b - a) * num) / den);
}

namespace Cal {

// Linear interpolation on a curve, clamped at both ends.
int32_t Curve(const CalCurve &c, int32_t x) {
  if (c.n == 0)
    return 0;
  if (c.n == 1)
    return c.y[0];
  int32_t num, den;
  int i = Segment(c.x, c.n, x, num, den);
  return Lerp(c.y[i], c.y[i + 1], num, den);
}

// Bilinear interpolation on a map, clamped at the edges.
int32_t Map(const CalMap &m, int32_t x, int32_t y) {
  if (m.nx == 0 || m.ny == 0)
    return 0;
  int32_t nxn = 0, dxn = 1, nyn = 0, dyn = 1;
  int ix = m.nx > 1 ? Segment(m.x, m.nx, x, nxn, dxn) : 0;
  int iy = m.ny > 1 ? Segment(m.y, m.ny, y, nyn, dyn) : 0;
  int ix1 = m.nx > 1 ? ix + 1 : ix;
  int iy1 = m.ny > 1 ? iy + 1 : iy;
  int32_t z00 = m.z[iy * m.nx + ix];
  int32_t z01 = m.z[iy * m.nx + ix1];
  int32_t z10 = m.z[iy1 * m.nx + ix];
  int32_t z11 = m.z[iy1 * m.nx + ix1];
  int32_t r0 = Lerp(z00, z01, nxn, dxn);
  int32_t r1 = Lerp(z10, z11, nxn, dxn);
  return Lerp(r0, r1, nyn, dyn);
}

// Linear interpolation on plain arrays (small fixed tables in code).
int32_t Interp(const int32_t *xs, const int32_t *ys, int n, int32_t x) {
  if (n <= 0)
    return 0;
  if (n == 1)
    return ys[0];
  int32_t num, den;
  int i = Segment(xs, n, x, num, den);
  return Lerp(ys[i], ys[i + 1], num, den);
}

} // namespace Cal
