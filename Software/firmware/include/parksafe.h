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
#ifndef PARKSAFE_H
#define PARKSAFE_H

#include <stdint.h>

/* Park-lock safety rule (V5 board).
 *
 * Why: the park lock is held out by oil pressure in the park cylinder,
 * fed through the PZYL valve, plus the PHOLD magnet. If PZYL loses its
 * current while the car is moving, the spring pushes the park pawl into
 * the turning output gear. On the V5 board both coils sit on IC10, which
 * drops them as soon as its EN pin goes low, its RESN pin goes low, or it
 * is reset. So:
 *
 *   Once PZYL is energised (park lock out) and the output shaft turns at
 *   PARK_HOLD_RPM or more, or its speed is not known, PZYL stays energised
 *   until the car has stopped. While that "hold" is on:
 *    - the safe / limp state keeps PZYL at its hold current (the
 *      pressure valves still go to 0 mA = 6th gear, full line pressure,
 *      which also feeds the park cylinder);
 *    - the driver chips are never reset or re-initialised and EN is
 *      never pulled low (MayRestartDrivers() is false);
 *    - a chip fault is reported but never switches PZYL off.
 *
 * What the firmware cannot cover: an MCU reset or brown-out releases EN
 * and RESN (they only have pull-downs on the board), which drops PZYL.
 * That needs a hardware answer (see README "Park lock safety").
 *
 * This class also shapes the PZYL current: when PZYL goes from off to on
 * it gets the pull-in current for PZYL_PULL_MS, then the hold current.
 * Pure logic, tested on the host (test_parksafe.cpp).
 */

/// Output shaft speed at and above which the hold latches, rpm.
/// PROVISIONAL: a guess; the lowest speed at which the pawl can still
/// ratchet over the gear is not known.
#define PARK_HOLD_RPM 50
/// PZYL pull-in: 0.95 A for 40 ms after switching on, then the hold
/// current (PzylCurrent, 0.40 A). PROVISIONAL (Ian / Bob, V5 valve
/// defaults). On a low supply the ~11 ohm coil may not reach 0.95 A; the
/// chip then runs at full duty, which is fine.
#define PZYL_PULL_MS 40
#define PZYL_PULL_IN 9500 // 0.1 mA
/// PHOLD (park-hold magnet, 24-26 ohm) current. PROVISIONAL: Ian's
/// estimate of the OEM average, 0.21-0.27 A. No pull-in peak, no dither;
/// on for at least 100 ms (ParkMagMin) and at most 1 s per release.
#define PHOLD_CURRENT 2700 // 0.1 mA

class ParkSafe
{
public:
   ParkSafe();
   /// Hold current used when the TCU itself asks for 0 (safe state), and
   /// the pull-in current. Both 0.1 mA.
   void Configure(int32_t holdI, int32_t pullInI);
   /** Every 10 ms. req = PZYL current the TCU asks for (0.1 mA, 0 in the
    * safe state); nAb = output shaft rpm; speedKnown = the output speed
    * sensor is working. Returns the PZYL current to drive. */
   int32_t Update(int32_t req, int32_t nAb, bool speedKnown);
   /// The park lock must be held out: PZYL may not drop.
   bool Holding() const { return hold; }
   /// Chips may be reset, re-initialised or EN pulled low.
   bool MayRestartDrivers() const { return !hold; }
   /// Last PZYL current Update() returned.
   int32_t Output() const { return out; }

private:
   int32_t holdI;
   int32_t pullI;
   int32_t out;
   int32_t lastOn;   // last non-zero TCU request
   int32_t pullLeft; // ms of pull-in still to go
   bool hold;
};

#endif // PARKSAFE_H
