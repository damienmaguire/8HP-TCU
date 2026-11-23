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
#include "shifter.h"
#include "tcu.h" // SEL_P .. SEL_D

Shifter::Shifter()
    : cfg(), selector(SEL_N), candSel(SEL_N), candBad(false), selCount(0),
      upRaw(false), downRaw(false), upStable(false), downStable(false),
      upCount(2), downCount(2), tipUp(false), tipDown(false), fault(false) {
  cfg.type = SHIFTER_CAN;
  cfg.anaRMax = 1500;
  cfg.anaNMax = 3000;
  cfg.anaMin = 200;
  cfg.anaMax = 6000;
}

uint8_t Shifter::Decode(const ShifterRaw &r, bool *bad) const {
  *bad = false;
  if (cfg.type == SHIFTER_CAN)
    return r.canSelector <= SEL_D ? r.canSelector : (uint8_t)SEL_N;
  if (r.parkIn == r.notParkIn) {
    *bad = true;
    return SEL_N;
  }
  if (r.parkIn)
    return SEL_P;
  if (r.anaMv < cfg.anaMin || r.anaMv > cfg.anaMax) {
    *bad = true;
    return SEL_N;
  }
  if (r.anaMv < cfg.anaRMax)
    return SEL_R;
  if (r.anaMv < cfg.anaNMax)
    return SEL_N;
  return SEL_D;
}

/// Two equal samples in a row before a new level is taken.
static bool Debounce(bool raw, bool *last, bool *stable, uint8_t *count) {
  if (raw != *last) {
    *last = raw;
    *count = 1;
  } else if (*count < 2) {
    (*count)++;
  }
  if (*count >= 2 && *stable != raw) {
    *stable = raw;
    return raw; // rising edge of the stable level
  }
  return false;
}

void Shifter::Update(const ShifterRaw &r) {
  bool bad;
  uint8_t s = Decode(r, &bad);
  if (s != candSel || bad != candBad) {
    candSel = s;
    candBad = bad;
    selCount = 1;
  } else if (selCount < 2) {
    selCount++;
  }
  if (selCount >= 2) {
    selector = candSel;
    fault = candBad;
  }

  bool up = cfg.type == SHIFTER_CAN ? r.canTipUp : r.din1;
  bool dn = cfg.type == SHIFTER_CAN ? r.canTipDown : r.din2;
  tipUp = Debounce(up, &upRaw, &upStable, &upCount);
  tipDown = Debounce(dn, &downRaw, &downStable, &downCount);
  if (tipUp && tipDown)
    tipUp = tipDown = false; // both at once: ignore
}
