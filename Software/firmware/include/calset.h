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
#include "cal_f11.h"
#include "cal_v095.h"
#include <stdint.h>

/* Calibration set per gearbox AND calibration (one row per cal), picked by
 * the GearboxType parameter (ids 0 and 1 kept, new rows appended). The
 * shift engine is the same for every row: the clutch pattern tables are
 * byte-identical in every SH program [C].
 *
 * Sources (tools/gen_cal.py, VARIANTS_SUMMARY.md 5):
 *  - Shift, gear-in, line pressure, globals and the per-clutch fill data:
 *    the 330e Q030 tables (exact A2L labels [C]). The v095 sets are [I],
 *    so they do not drive the shift engine. The F11 v058 port is [I]/[H]
 *    except a few [C] rows; the F11 row uses its own line pressure
 *    (KF_PSYS_R..8, KF_PSYSN), EDS temperature corrections (KF_KIEDSCG*)
 *    and R / 1st gear-in pressures from f11_psys_gearin_cal, with the
 *    330e tables as the marked FALLBACK for everything else.
 *  - F11 EDS: the block at 0x19FCA8 is seven curves (A..E, SYS, TCC). The
 *    8th item at 0x19FE14 is KF_KIEDSCGFAACE (the EDS temperature
 *    correction), not POS; there is no F11 POS curve.
 *  - Solenoid p->I curves: a separate per-unit setting (SolCurveSet),
 *    following the valve hardware revision, not the gearbox size. Each row
 *    has a default set.
 *  - K0 present / absent: the 330e is the K0 reference (Q030 strategy).
 *  - Size data (torque class, reverse limit, safety offsets, fill /
 *    piston / hysteresis reference tables): calSizes, [I], from the
 *    exported values only. Data only: no output uses it yet.
 *  - DriveMode Sport / Track: the OEM Sport set from Q030 (*SPORT*
 *    labels, Normal where none), plus the DriveMode modifiers.
 */

enum CalVariant
{
   CALVAR_330E = 0,       // 8HP75 GA8P75HZ 330e Q030: hybrid, K0, LIN pump
   CALVAR_F11 = 1,        // 8HP45 F11 v058: torque converter (TCC on ch 7)
   CALVAR_V095_F11N47 = 2, // 8HP45 a81 v095 cal 2768 (F11 N47)
   CALVAR_V095_N57 = 3,   // 8HP70 a81 v095 cal 1367 (N57)
   CALVAR_V095_N63 = 4,   // 8HP70 a81 v095 cal 317e (N63)
   CALVAR_V095_N74 = 5,   // 8HP90 a81 v095 cal 0a8b (N74)
   CALVAR_V095_RR4 = 6,   // 8HP90 a81 v095 cal 0a92 (RR4 N74)
   CALVAR_COUNT
};

/// Solenoid curve set (SolCurveSet parameter). 0 = the row's default.
enum SolCurveSet
{
   SOLSET_AUTO = 0,
   SOLSET_330E_Q030 = 1,  // 330e Q030 KL_EDSKV* [C]
   SOLSET_F11_V058 = 2,   // F11 v058 block 0x19FCA8 (A..E, ch7 = v095 8HP90 [C])
   SOLSET_V095_90 = 3,    // a81 v095 8HP90 (N74, RR4) [I]
   SOLSET_V095_45_70 = 4, // a81 v095 8HP45 / 8HP70 (F11 N47, N57, N63) [I]
   SOLSET_COUNT
};

enum GearboxSize
{
   GBX_8HP45 = 45,
   GBX_8HP70 = 70,
   GBX_8HP75 = 75,
   GBX_8HP90 = 90,
};

/// What channel 7 (the K0 / TCC valve slot) does. Ch7Mode parameter.
enum Ch7Mode
{
   CH7_VARIANT = 0, // the variant's default (F11 lock-up, 330e K0)
   CH7_TCC = 1,     // converter lock-up clutch control
   CH7_K0 = 2,      // hybrid disconnect clutch, pressure from K0Press
   CH7_OPEN = 3,    // always open (both gearboxes, e.g. an EV with no use for it)
};

/// DriveMode parameter. Normal = the cal as it is.
enum DriveMode
{
   DM_NORMAL = 0,
   DM_SPORT = 1,
   DM_TRACK = 2,
   DM_COUNT
};

struct VariantCal
{
   const char *name;
   const char *cal;             // calibration id / source
   uint8_t gearbox;             // GearboxSize
   const ClutchFillCal *clutch; // [5] A..E (Normal)
   const UpShiftCal *up;        // [CAL_NUM_PAIRS]
   const DownShiftCal *down;    // [CAL_NUM_PAIRS]
   const EngageCal *engage;     // [2] N->1, N->R
   const CalMap *psys;          // [9] R 1..8
   const GlobalCal *global;
   uint8_t solSet;              // default SolCurveSet
   bool k0Present;              // hybrid disconnect clutch K0 fitted
   uint8_t ch7Default;          // CH7_TCC or CH7_K0
   uint8_t engVar;              // EngageVariant: gear-in role table
   bool linPump;                // has the LIN electric oil pump
   bool posConfirmed;           // POS valve seen in this row's software
   const CalSizeInfo *size;     // per-size data [I]
   /* Tables per DriveMode, 0 = the Normal ones. Sport and Track use the
    * OEM Sport set from Q030 (calUpSport etc.). */
   const UpShiftCal *upMode[DM_COUNT];
   const DownShiftCal *downMode[DM_COUNT];
   const ClutchFillCal *clutchMode[DM_COUNT];
};

struct SolCurveSetInfo
{
   const char *name;
   const CalCurve *eds;         // [7] A B C D E SYS ch7, mbar -> 0.1 mA
   char evidence;               // 'C' or 'I'
};

namespace Cal
{
/// Table entry for a variant; the 330e for an unknown number.
const VariantCal &Variant(int id);
/// Make a row the active one (at start-up and on a GearboxType change),
/// with a SolCurveSet (SOLSET_AUTO = the row's default).
void Select(int id, int solSet = SOLSET_AUTO);
/// Solenoid curve set info; SOLSET_AUTO / unknown -> the 330e set.
const SolCurveSetInfo &SolSet(int set);
/// The SolCurveSet in use after resolving SOLSET_AUTO.
int ActiveSolSet();
/// Active solenoid curves [7].
const CalCurve *Eds();
/// Per-clutch fill data of the active row and DriveMode [5].
const ClutchFillCal *Clutch();
/// Reference table of a size row by A2L name, 0 if it has none.
const CalRef *Ref(const CalSizeInfo &s, const char *name);
/// The active variant's tables.
const VariantCal &Active();
/// Make a DriveMode the active one (its shift tables, see upMode).
void SelectMode(int mode);
int ActiveMode();
/// Shift tables of the active variant and DriveMode [CAL_NUM_PAIRS].
const UpShiftCal *Up();
const DownShiftCal *Down();
/// Channel 7 mode after resolving CH7_VARIANT. K0 on a row without a K0
/// is forced open (the slot drives a TCC valve there).
uint8_t Ch7Effective(uint8_t mode, int variant);
} // namespace Cal

#endif // CALSET_H
