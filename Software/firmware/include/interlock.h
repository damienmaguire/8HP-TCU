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
 * produced its pressure commands and before anything reaches the driver.
 *
 * Ian's rule [I]: never command more than three elements above fill
 * pressure, because four applied elements lock the gearbox. A single-swap
 * shift is the one case where four elements legitimately carry pressure at
 * the same time (the off-going one is slipping out while the on-coming one
 * slips in), so the rule is applied like this:
 *
 *  1. Every element above its fill pressure must be in the "allowed" set
 *     the sequencer declares: the current gear, the gear being engaged,
 *     or (during a shift) the union of the from-gear and to-gear.
 *  2. Outside a shift, at most three elements may be above fill, and if
 *     three are, they must be a real gear. (A C E is the one three-element
 *     combination that is not a gear.)
 *  3. During a shift, at most four, and only the four of the swap.
 *
 * A violation sends the TCU to its safe state. The OEM uses an ASIL
 * current-plausibility table (jHydEdsLogik) instead; this simpler check is
 * what Ian recommended in 330E_EDS_PWM.md section 7.
 */

enum InterlockResult
{
   IL_OK = 0,
   IL_NOT_ALLOWED, // an element outside the allowed set is above fill
   IL_TOO_MANY,    // more than 3 (or 4 in a swap) elements above fill
   IL_NOT_A_GEAR,  // three elements applied that do not form a gear
};

struct InterlockInput
{
   int32_t pressure[CL_COUNT];  // commanded pressure per element, mbar
   int32_t fillLevel[CL_COUNT]; // above this an element counts as applied, mbar
   uint8_t allowedMask;         // CLUTCH_BIT set the sequencer may apply
   bool shifting;               // true while a single swap is running
};

namespace Interlock
{
/// Mask of the elements whose command is above their fill level.
uint8_t AppliedMask(const InterlockInput &in);
/// Run the check. Returns IL_OK if the command is safe to send.
InterlockResult Check(const InterlockInput &in);
} // namespace Interlock

#endif // INTERLOCK_H
