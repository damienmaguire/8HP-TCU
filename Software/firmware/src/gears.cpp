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

/* Applied elements per gear, indexed by enum Gear. [C]
 * BMW TIS names the same elements B1 = A, B2 = B, K1 = C, K2 = D, K3 = E;
 * its shift-element table matches this one cell for cell [C-TIS]. Exactly
 * three elements are closed in every gear. */
static const uint8_t gearMask[GEAR_COUNT] = {
    0,                                                // N (B is only pre-filled)
    CLUTCH_BIT(CL_A) | CLUTCH_BIT(CL_B) | CLUTCH_BIT(CL_C), // 1 = A B C
    CLUTCH_BIT(CL_A) | CLUTCH_BIT(CL_B) | CLUTCH_BIT(CL_E), // 2 = A B E
    CLUTCH_BIT(CL_B) | CLUTCH_BIT(CL_C) | CLUTCH_BIT(CL_E), // 3 = B C E
    CLUTCH_BIT(CL_B) | CLUTCH_BIT(CL_D) | CLUTCH_BIT(CL_E), // 4 = B D E
    CLUTCH_BIT(CL_B) | CLUTCH_BIT(CL_C) | CLUTCH_BIT(CL_D), // 5 = B C D
    CLUTCH_BIT(CL_C) | CLUTCH_BIT(CL_D) | CLUTCH_BIT(CL_E), // 6 = C D E
    CLUTCH_BIT(CL_A) | CLUTCH_BIT(CL_C) | CLUTCH_BIT(CL_D), // 7 = A C D
    CLUTCH_BIT(CL_A) | CLUTCH_BIT(CL_D) | CLUTCH_BIT(CL_E), // 8 = A D E
    CLUTCH_BIT(CL_A) | CLUTCH_BIT(CL_B) | CLUTCH_BIT(CL_D), // R = A B D
};

/* Ratios from D_i_TargetGear @0x201FBC in the 330e image [C]; the F11
 * 8HP45 constant @0x4C7DC has the same values. Used for both gearboxes.
 *
 * Reference only, NOT used: BMW TIS, G30 520e 8HPCC (the later 8HP
 * gearset) [C-TIS]:
 *   1 5.000  2 3.200  3 2.143  4 1.720  5 1.314  6 1.000  7 0.822
 *   8 0.640  R 3.456
 * Starting point for a future G-series variant; check it against that
 * unit's own calibration first. Do not put it in the F11 / 330e rows. */
static const int32_t gearRatio[GEAR_COUNT] = {
    0, 4714, 3143, 2106, 1667, 1285, 1000, 839, 667, 3306,
};

namespace Gears {

// 1st..8th or R.
bool IsDriveGear(int gear) { return gear >= GEAR_1 && gear <= GEAR_R; }

// Clutches applied in a gear (bit per clutch).
uint8_t ClutchMask(int gear) {
  if (gear < 0 || gear >= GEAR_COUNT)
    return 0;
  return gearMask[gear];
}

// Gear ratio x 1000 (0 for N).
int32_t RatioX1000(int gear) {
  if (gear < 0 || gear >= GEAR_COUNT)
    return 0;
  return gearRatio[gear];
}

// Number of applied clutches in a mask.
int CountClutches(uint8_t mask) {
  int n = 0;
  for (int c = 0; c < CL_COUNT; c++)
    if (mask & CLUTCH_BIT(c))
      n++;
  return n;
}

// Index of the single bit set in a mask, -1 if not exactly one.
static int OnlyBit(uint8_t mask) {
  for (int c = 0; c < CL_COUNT; c++)
    if (mask == CLUTCH_BIT(c))
      return c;
  return -1;
}

// Off-going and on-coming clutch of a one-clutch swap; false if not one.
bool SingleSwap(int from, int to, int &offGoing, int &onComing) {
  offGoing = onComing = -1;
  if (!IsDriveGear(from) || !IsDriveGear(to) || from == to)
    return false;
  uint8_t a = ClutchMask(from), b = ClutchMask(to);
  int off = OnlyBit(a & ~b);
  int on = OnlyBit(b & ~a);
  if (off < 0 || on < 0)
    return false;
  offGoing = off;
  onComing = on;
  return true;
}

// Next gear on the way from one gear to another (adjacent steps only).
int NextStep(int from, int to) {
  int off, on;
  if (from == to)
    return to;
  // Forward-only routing. Reverse is never reached by a shift, only by
  // engaging from neutral.
  if (from == GEAR_R || to == GEAR_R || from == GEAR_N || to == GEAR_N)
    return to;
  if (SingleSwap(from, to, off, on) && (to - from == 1 || from - to == 1))
    return to;
  // v1: adjacent steps only. Skips that happen to be single swaps are
  // allowed by Ian's table but the cal export only covers adjacent pairs,
  // so take them one gear at a time.
  return to > from ? from + 1 : from - 1;
}

// Input speed the gear would give at this output speed.
int32_t SyncSpeed(int gear, int32_t outputRpm) {
  return (int32_t)(((int64_t)outputRpm * RatioX1000(gear)) / 1000);
}

// Input / output speed ratio x 1000 (0 at standstill).
int32_t MeasuredRatioX1000(int32_t inputRpm, int32_t outputRpm) {
  if (outputRpm < 100 && outputRpm > -100)
    return 0;
  return (int32_t)(((int64_t)inputRpm * 1000) / outputRpm);
}

// Input speed minus the gear's sync speed (0 outside a gear).
int32_t SlipRpm(int gear, int32_t inputRpm, int32_t outputRpm) {
  if (!IsDriveGear(gear))
    return 0;
  return inputRpm - SyncSpeed(gear, outputRpm);
}

// Clutch letter A..E for logs.
char ClutchName(int clutch) {
  return clutch >= 0 && clutch < CL_COUNT ? (char)('A' + clutch) : '-';
}

} // namespace Gears
