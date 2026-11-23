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
#include "calset.h"
#include "engage.h"
#include <string.h>

#define SPORT_SET {0, calUpSport, calUpSport}, {0, calDownSport, calDownSport}, \
                  {0, calClutchSport, calClutchSport}
// Shift / gear-in / line pressure / fill: 330e Q030 [C].
#define Q030_TABLES calClutch, calUp, calDown, calEngage, calPsys, &calGlobal
/* F11 v058: line pressure, EDS temperature corrections, gear-in, the
 * shift pressure maps (stage 4: up on-coming, up off-going, down
 * off-going, P_SYSMINxy) and the P_SYSMIN / GLS globals from the F11
 * export. Fill, the shift timings, the downshift on-coming map and the
 * remaining scalars are the 330e Q030 ones (FALLBACK, see cal_f11.cpp).
 * Program 0 = Normal; programs 1 (SP) and 2 (SPORT) [I] are the Sport
 * and Track tables. */
#define F11_TABLES calClutch, calF11Up[0], calF11Down[0], calF11Engage, calF11Psys, \
                   &calF11Global
#define F11_MODES {0, calF11Up[1], calF11Up[2]}, {0, calF11Down[1], calF11Down[2]}, \
                  {0, calClutchSport, calClutchSport}

static const VariantCal variants[CALVAR_COUNT] = {
    // 8HP75 330e Q030: the reference set, K0 reference.
    {"8HP75_330e", "330e Q030", GBX_8HP75, Q030_TABLES, SOLSET_330E_Q030, true,
     CH7_K0, ENG_VAR_330E, true, true, &calSizes[CALSIZE_330E_Q030], SPORT_SET},
    // 8HP45 F11 v058: own solenoid block; the size data is the v095 F11
    // N47 cal's (same car and gearbox; the v058 port has no [C] size rows).
    {"8HP45_F11", "F11 v058", GBX_8HP45, F11_TABLES, SOLSET_F11_V058, false,
     CH7_TCC, ENG_VAR_F11, false, false, &calSizes[CALSIZE_V095_F11N47],
     F11_MODES},
    // a81 v095 cals [I]. Gear-in roles: the F11 (Gen1 a81) table [I].
    {"8HP45_F11N47_v095", "a81 v095 2768", GBX_8HP45, Q030_TABLES,
     SOLSET_V095_45_70, false, CH7_TCC, ENG_VAR_F11, false, false,
     &calSizes[CALSIZE_V095_F11N47], SPORT_SET},
    {"8HP70_N57_v095", "a81 v095 1367", GBX_8HP70, Q030_TABLES, SOLSET_V095_45_70,
     false, CH7_TCC, ENG_VAR_F11, false, false, &calSizes[CALSIZE_V095_N57],
     SPORT_SET},
    {"8HP70_N63_v095", "a81 v095 317e", GBX_8HP70, Q030_TABLES, SOLSET_V095_45_70,
     false, CH7_TCC, ENG_VAR_F11, false, false, &calSizes[CALSIZE_V095_N63],
     SPORT_SET},
    {"8HP90_N74_v095", "a81 v095 0a8b", GBX_8HP90, Q030_TABLES, SOLSET_V095_90,
     false, CH7_TCC, ENG_VAR_F11, false, false, &calSizes[CALSIZE_V095_N74],
     SPORT_SET},
    {"8HP90_RR4N74_v095", "a81 v095 0a92", GBX_8HP90, Q030_TABLES, SOLSET_V095_90,
     false, CH7_TCC, ENG_VAR_F11, false, false, &calSizes[CALSIZE_V095_RR4N74],
     SPORT_SET},
};

static const SolCurveSetInfo solSets[SOLSET_COUNT] = {
    {"Auto", calEds, 'C'}, // resolved per row, never used directly
    {"330e_Q030", calEds, 'C'},
    {"F11_v058", calF11Eds, 'C'},
    {"v095_8HP90", calV095Eds90, 'I'},
    {"v095_8HP45_70", calV095Eds45, 'I'},
};

static const VariantCal *active = &variants[CALVAR_330E];
static int activeSol = SOLSET_330E_Q030;
static int activeMode = DM_NORMAL;

namespace Cal {

// Table row for a GearboxType id (the 330e row for an unknown id).
const VariantCal &Variant(int id) {
  if (id < 0 || id >= CALVAR_COUNT)
    id = CALVAR_330E;
  return variants[id];
}

// Make a row and solenoid curve set the active ones.
void Select(int id, int solSet) {
  active = &Variant(id);
  activeSol = solSet > SOLSET_AUTO && solSet < SOLSET_COUNT ? solSet : active->solSet;
}

// Solenoid curve set by id (the 330e set for an unknown id).
const SolCurveSetInfo &SolSet(int set) {
  if (set <= SOLSET_AUTO || set >= SOLSET_COUNT)
    set = SOLSET_330E_Q030;
  return solSets[set];
}

// Solenoid curve set in use.
int ActiveSolSet() { return activeSol; }

// Pressure-to-current curves in use (A..E, SYS, ch7).
const CalCurve *Eds() { return solSets[activeSol].eds; }

// The active gearbox row.
const VariantCal &Active() { return *active; }

// Switch the shift tables to a DriveMode's set.
void SelectMode(int mode) {
  activeMode = mode >= 0 && mode < DM_COUNT ? mode : DM_NORMAL;
}

// DriveMode whose tables are in use.
int ActiveMode() { return activeMode; }

// Upshift tables of the active row and DriveMode.
const UpShiftCal *Up() {
  const UpShiftCal *t = active->upMode[activeMode];
  return t ? t : active->up;
}

// Downshift tables of the active row and DriveMode.
const DownShiftCal *Down() {
  const DownShiftCal *t = active->downMode[activeMode];
  return t ? t : active->down;
}

// Clutch fill tables of the active row and DriveMode.
const ClutchFillCal *Clutch() {
  const ClutchFillCal *t = active->clutchMode[activeMode];
  return t ? t : active->clutch;
}

// Find a reference table of a size row by its A2L name.
const CalRef *Ref(const CalSizeInfo &s, const char *name) {
  for (int i = 0; i < s.nRef; i++)
    if (strcmp(s.ref[i].name, name) == 0)
      return &s.ref[i];
  return 0;
}

// Channel 7 mode actually used: K0 on a gearbox without K0 is held open.
uint8_t Ch7Effective(uint8_t mode, int variant) {
  const VariantCal &v = Variant(variant);
  if (mode == CH7_VARIANT || mode > CH7_OPEN)
    mode = v.ch7Default;
  if (mode == CH7_K0 && !v.k0Present)
    return CH7_OPEN;
  return mode;
}

} // namespace Cal
