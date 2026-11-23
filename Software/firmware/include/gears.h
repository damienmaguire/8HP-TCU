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
#ifndef GEARS_H
#define GEARS_H

#include <stdint.h>

/* Gear and clutch tables for the ZF 8HP (GA8P75HZ).
 *
 * Source: Ian's 330E_OPEN_TCU_CONTROL_SPEC.md section 4 and
 * 330E_SHIFT_CLUTCH_FIRST_CUT.md, confirmed against 27 per-gear-per-element
 * maps in the 330e firmware [C].
 *
 * Five shift elements: A and B are brakes, C, D and E are clutches. Every
 * gear has exactly three of them applied:
 *   R = A B D   1 = A B C   2 = A B E   3 = B C E   4 = B D E
 *   5 = B C D   6 = C D E   7 = A C D   8 = A D E
 * With all coils at 0 A the normally-high valves apply C, D and E, which is
 * 6th gear: the mechanical limp-home state [C].
 */

enum Clutch
{
   CL_A = 0,
   CL_B,
   CL_C,
   CL_D,
   CL_E,
   CL_COUNT
};

#define CLUTCH_BIT(c) (1u << (c))

/* Gear numbering used everywhere in this firmware.
 * GEAR_N is neutral, 1..8 are the forward gears and GEAR_R is reverse.
 */
enum Gear
{
   GEAR_N = 0,
   GEAR_1 = 1,
   GEAR_2,
   GEAR_3,
   GEAR_4,
   GEAR_5,
   GEAR_6,
   GEAR_7,
   GEAR_8,
   GEAR_R,
   GEAR_COUNT
};

/// The gear the gearbox falls into with every solenoid unpowered.
#define GEAR_LIMP GEAR_6

namespace Gears
{
/// Bit mask (CLUTCH_BIT) of the elements applied in a gear. N returns 0.
uint8_t ClutchMask(int gear);

/// Gear ratio times 1000 (R is given as a positive number). N returns 0.
int32_t RatioX1000(int gear);

/// Number of elements set in a mask.
int CountClutches(uint8_t mask);

/** Work out the single-swap shift between two gears.
 * A single swap releases one element (off-going, "KAB") and applies one
 * element (on-coming, "KZU") while the other two stay applied.
 * \return true if the shift is a single swap, false if it needs an
 *         intermediate gear (a "2-swap" in Ian's table) or is not a shift.
 */
bool SingleSwap(int from, int to, int &offGoing, int &onComing);

/** Next gear to aim for when going from 'from' towards 'to'.
 * v1 rule from Ian's spec section 5.1: use a direct single swap if one
 * exists, otherwise step one gear at a time. The OEM routing tables
 * (0x51418 quiet, 0x51350 sport) are not used yet.
 */
int NextStep(int from, int to);

/// Synchronous input speed for a gear: n_ab * ratio. Returns rpm.
int32_t SyncSpeed(int gear, int32_t outputRpm);

/// Measured ratio n_t / n_ab x 1000, 0 below 100 rpm output (no reading).
int32_t MeasuredRatioX1000(int32_t inputRpm, int32_t outputRpm);
/// Slip in a gear: n_t - SyncSpeed(gear, n_ab). 0 in N or another non-gear.
int32_t SlipRpm(int gear, int32_t inputRpm, int32_t outputRpm);

/// True for 1..8 and R.
bool IsDriveGear(int gear);

/// Letter for logging ('A'..'E').
char ClutchName(int clutch);
} // namespace Gears

#endif // GEARS_H
