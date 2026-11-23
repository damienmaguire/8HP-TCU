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
#include "cal_f11.h"

static const VariantCal variants[CALVAR_COUNT] = {
    // 8HP75 330e: the reference set.
    {"8HP75_330e", calClutch, calEds, calUp, calDown, calEngage, calPsys,
     &calGlobal, CH7_K0, true, 0, true,
     {calUp, 0, 0}, {calDown, 0, 0}},
    // 8HP45 F11: own EDS curves and fill items, the rest shared [I].
    {"8HP45_F11", calF11Clutch, calF11Eds, calUp, calDown, calEngage, calPsys,
     &calGlobal, CH7_TCC, false, 0x0B0, false,
     {calUp, 0, 0}, {calDown, 0, 0}},
};

static const VariantCal *active = &variants[CALVAR_330E];
static int activeMode = DM_NORMAL;

namespace Cal {

const VariantCal &Variant(int id) {
  if (id < 0 || id >= CALVAR_COUNT)
    id = CALVAR_330E;
  return variants[id];
}

void Select(int id) { active = &Variant(id); }

const VariantCal &Active() { return *active; }

void SelectMode(int mode) {
  activeMode = mode >= 0 && mode < DM_COUNT ? mode : DM_NORMAL;
}

int ActiveMode() { return activeMode; }

const UpShiftCal *Up() {
  const UpShiftCal *t = active->upMode[activeMode];
  return t ? t : active->up;
}

const DownShiftCal *Down() {
  const DownShiftCal *t = active->downMode[activeMode];
  return t ? t : active->down;
}

uint8_t Ch7Effective(uint8_t mode, int variant) {
  if (mode == CH7_VARIANT || mode > CH7_OPEN)
    return Variant(variant).ch7Default;
  return mode;
}

} // namespace Cal
