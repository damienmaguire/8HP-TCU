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
#ifndef CALTYPES_H
#define CALTYPES_H

#include <stdint.h>

/* Calibration curve (A2L "KL_" item): y = f(x).
 * x must be ascending. Lookups interpolate linearly and clamp at the ends,
 * the same way the OEM firmware treats its curves.
 */
struct CalCurve
{
   uint8_t n;
   const int16_t *x;
   const int32_t *y;
};

/* Calibration map (A2L "KF_" item): z = f(x, y).
 * z is stored row by row: z[iy * nx + ix]. x is the first A2L axis and
 * y the second, which is the orientation Ian checked against the firmware.
 */
struct CalMap
{
   uint8_t nx;
   uint8_t ny;
   const int16_t *x;
   const int16_t *y;
   const int32_t *z;
};

namespace Cal
{
/// Linear interpolation on a curve, clamped at both ends.
int32_t Curve(const CalCurve &c, int32_t x);
/// Bilinear interpolation on a map, clamped at all edges.
int32_t Map(const CalMap &m, int32_t x, int32_t y);
/// Interpolate between two raw arrays (used for parameter-based tables).
int32_t Interp(const int32_t *xs, const int32_t *ys, int n, int32_t x);
} // namespace Cal

#endif // CALTYPES_H
