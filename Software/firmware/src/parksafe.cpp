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
#include "parksafe.h"

ParkSafe::ParkSafe() : holdI(0), pullI(0), out(0), lastOn(0), pullLeft(0), hold(false) {}

void ParkSafe::Configure(int32_t h, int32_t p) {
  holdI = h;
  pullI = p;
}

int32_t ParkSafe::Update(int32_t req, int32_t nAb, bool speedKnown) {
  if (nAb < 0)
    nAb = -nAb;
  bool moving = !speedKnown || nAb >= PARK_HOLD_RPM;
  // Only a known speed below the threshold lets go of the hold.
  if (!moving)
    hold = false;

  int32_t i = req > 0 ? req : 0;
  if (i > 0)
    lastOn = i;
  // SAFETY: while the hold is on PZYL never drops below the hold current
  // (or the last current the TCU used, if that is higher), whatever the
  // TCU, a fault or the safe state asks for. lastOn is never 0 here: the
  // hold only latches after PZYL was on.
  if (hold && i < holdI)
    i = holdI;
  if (hold && i < lastOn)
    i = lastOn;

  // Pull-in pulse when switching on from off.
  if (i > 0 && out == 0)
    pullLeft = PZYL_PULL_MS;
  if (i <= 0)
    pullLeft = 0;
  int32_t drive = i;
  if (pullLeft > 0) {
    if (drive < pullI)
      drive = pullI;
    pullLeft -= 10;
  }
  out = drive;
  if (drive == 0)
    lastOn = 0; // PZYL off: forget the old request
  // Latch: PZYL is on (lock out) and the car moves (or may be moving).
  if (out > 0 && moving)
    hold = true;
  return drive;
}
