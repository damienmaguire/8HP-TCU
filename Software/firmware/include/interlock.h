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
#ifndef INTERLOCK_H
#define INTERLOCK_H

#include "gears.h"
#include <stdint.h>

/* Hard clutch interlock, checked every 10 ms after the hydraulic layer has
 * produced its commands and before anything reaches the driver.
 *
 * It works on the commanded coil currents (the targets handed to the
 * MAX22200 layer, before slew and dither), not on pressures, so it checks
 * what the valves will actually be told. Rules, from Ian (QA2 answer 2):
 *
 *  1. A clutch counts as "open" only when its coil current is past the
 *     open threshold for its valve type:
 *       A, B (normally low, 0 A = no pressure):  open below  60 mA
 *       C, D, E (normally high, 0 A = full):     open above 790 mA
 *     Anything else counts as applied (filling, slipping or closed).
 *  2. Every clutch outside the allowed set must be open. The allowed set
 *     is the current gear, the gear being engaged, or during a shift the
 *     from-gear plus the to-gear.
 *  3. During a single swap four applied clutches are allowed (off-going
 *     slipping out while on-coming slips in). Five is never allowed.
 *  4. Outside a swap at most three, and three must be a real gear (A C E
 *     is the one three-element combination that is not).
 *
 * OEM background [C]: the L2 current-plausibility monitor
 * (iUl2IGetrPlausiZustandInt) has a legal state 4 "Verschachtelung, 4 Kup
 * beteiligt" and errors on 5 clutches or on wrong actuation outside the
 * shift. The open thresholds are IEDS_STOER_VUNPLAUSI* (A/B 60 mA, C/D/E
 * 790 mA). Not used here: the "pressurised" thresholds
 * I_EDSPOWERFLOWUEBMAX_A/B (287/215 mA) and I_EDSPOWERFLOWUEBMIN_C/D/E
 * (639/618/642 mA), and the position-N check (A/B < 150 mA, C/D/E >
 * 750 mA). Tie-up during a swap is prevented by the pressure ramps, not by
 * this count [I].
 *
 * A violation sends the TCU to its safe state.
 */

enum InterlockResult
{
   IL_OK = 0,
   IL_NOT_ALLOWED, // a clutch outside the allowed set is not open
   IL_TOO_MANY,    // more than 3 (or 4 in a swap) clutches applied, or all 5
   IL_NOT_A_GEAR,  // three clutches applied that do not form a gear
};

struct InterlockInput
{
   int32_t current[CL_COUNT]; // commanded coil current per clutch, 0.1 mA
   int32_t openLow;           // A/B open below this, 0.1 mA (600 = 60 mA)
   int32_t openHigh;          // C/D/E open above this, 0.1 mA (7900 = 790 mA)
   uint8_t allowedMask;       // CLUTCH_BIT set the sequencer may apply
   bool shifting;             // true while a single swap is running
};

namespace Interlock
{
/// True if the clutch's valve is normally low (A, B): more current = more pressure.
bool NormallyLow(int clutch);
/// True if this current puts the clutch in its open state.
bool IsOpen(int clutch, int32_t current, int32_t openLow, int32_t openHigh);
/// Mask of the clutches that are not open.
uint8_t AppliedMask(const InterlockInput &in);
/// Run the check. Returns IL_OK if the command is safe to send.
InterlockResult Check(const InterlockInput &in);
} // namespace Interlock

#endif // INTERLOCK_H
