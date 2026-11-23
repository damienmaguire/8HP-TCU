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
#ifndef CALSET_H
#define CALSET_H

#include "cal_330e.h"
#include <stdint.h>

/* Calibration set per gearbox variant, picked by the GearboxType
 * parameter. The shift engine is the same for every variant (the clutch
 * pattern tables are byte-identical in the F11 and 330e images [C]); only
 * what is listed here changes.
 *
 * Adding a gearbox (Ian will send 8HP45/70/90 sets later; they share the
 * a81 program) is one more entry in the table in calset.cpp, its
 * generated cal file, and one more name in GBOXTYPES. Tables a variant
 * does not have yet point at the 330e ones.
 *
 * Today:
 *   8HP75 330e  everything from the 330e export (cal_330e.*).
 *   8HP45 F11   EDS curves and the per-clutch fill items from the F11
 *               export (cal_f11.*); shift, gear-in, line pressure and the
 *               global values are the 330e ones [I: shared until an F11
 *               set exists]. Clutch A starts at about 176 mA here against
 *               about 240 mA on the 330e [C].
 */

enum CalVariant
{
   CALVAR_330E = 0, // GA8P75HZ, hybrid (K0 on channel 7, LIN pump)
   CALVAR_F11 = 1,  // 8HP45, engine + torque converter (TCC on channel 7)
   CALVAR_COUNT
};

/// What channel 7 (the K0 / TCC valve slot) does. Ch7Mode parameter.
enum Ch7Mode
{
   CH7_VARIANT = 0, // the variant's default (F11 lock-up, 330e K0)
   CH7_TCC = 1,     // converter lock-up clutch control
   CH7_K0 = 2,      // hybrid disconnect clutch, pressure from K0Press
   CH7_OPEN = 3,    // always open (both gearboxes, e.g. an EV with no use for it)
};

struct VariantCal
{
   const char *name;
   const ClutchFillCal *clutch; // [5] A..E
   const CalCurve *eds;         // [7] A B C D E SYS ch7, mbar -> 0.1 mA
   const UpShiftCal *up;        // [CAL_NUM_PAIRS]
   const DownShiftCal *down;    // [CAL_NUM_PAIRS]
   const EngageCal *engage;     // [2] N->1, N->R
   const CalMap *psys;          // [9] R 1..8
   const GlobalCal *global;
   uint8_t ch7Default;          // CH7_TCC or CH7_K0
   bool linPump;                // has the LIN electric oil pump
   uint16_t torqueTxId;         // engine torque intervention frame, 0 = none
                                // known (F11 0x0B0 [C]; the 330e hybrid ID
                                // index differs and is not mapped)
   bool posConfirmed;           // POS valve seen in this variant's software
                                // (F11: no, an 8th curve may be POS [I])
};

namespace Cal
{
/// Table entry for a variant; the 330e for an unknown number.
const VariantCal &Variant(int id);
/// Make a variant the active one (at start-up and on a GearboxType change).
void Select(int id);
/// The active variant's tables.
const VariantCal &Active();
/// Channel 7 mode after resolving CH7_VARIANT.
uint8_t Ch7Effective(uint8_t mode, int variant);
} // namespace Cal

#endif // CALSET_H
