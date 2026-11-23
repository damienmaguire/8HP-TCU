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
#ifndef VALVEOUT_H
#define VALVEOUT_H

#include "eds.h"
#include "max22200.h"
#include <stdint.h>

/* Glue between the valve setpoints (eds.h) and the MAX22200 chips.
 *
 * Which MAX22200 channel drives which valve is not known until Bob's
 * board is done, so the map is configuration: one entry per valve,
 * "chip * 8 + channel" (0..15), or VALVE_NOT_FITTED. The map is checked
 * before use: an out-of-range entry or two valves on one channel is a
 * configuration error and nothing is driven.
 *
 * Faults are read every 100 ms. A fault on a pressure valve (A..E, SYS)
 * means the TCU can no longer control pressure, so the caller goes to the
 * safe state (all coils off = 6th gear, full line pressure). A fault on
 * K0, POS, PZYL, SOL1 or PARK is reported but does not force limp.
 */

#define VALVE_MAX_CHIPS 2
#define VALVE_NOT_FITTED 0xFF

class ValveOut
{
public:
   ValveOut();
   /// Attach a chip (index 0 or 1). Chips not attached count as absent.
   void AttachChip(uint8_t idx, Max22200 *chip);
   /// Set the channel for a valve: chip * 8 + channel, or VALVE_NOT_FITTED.
   void SetMap(EdsValve v, uint8_t globalCh) { map[v] = globalCh; }
   uint8_t Map(EdsValve v) const { return map[v]; }
   /// Check the map. False if an entry is out of range, on a missing chip
   /// or shared with another valve. Also false if A..E or SYS is unmapped.
   bool Validate() const;

   /** Bring the chips up (data sheet Figure 6) and set the HIT phase on the
    * on/off outputs. Returns false on a map error or if a chip reports a
    * fault after init. Every channel is left off.
    */
   bool Init(const Max22200Config &cfg, const Max22200Hit &onOffHit);

   /// 1 ms: send the current setpoints (dither included) to the chips.
   void Update(const Eds &eds);
   /// Switch every channel off at once (safe state).
   void AllOff();

   /// 100 ms: read and clear chip faults. Returns the valves with a fault
   /// as a bit mask (bit = EdsValve).
   uint16_t PollFaults();
   /// Valves whose fault forces the safe state.
   static uint16_t CriticalMask();
   /// Chip status flags of the last poll (STATUS[7:0]) for logging.
   uint8_t ChipFlags(uint8_t idx) const { return flags[idx]; }
   /// True if a chip reset itself (UVM) or the SPI echo check failed.
   bool ChipLost() const { return lost; }

private:
   Max22200 *chips[VALVE_MAX_CHIPS];
   uint8_t map[EDS_COUNT];
   uint8_t flags[VALVE_MAX_CHIPS];
   bool ok;
   bool lost;
};

#endif // VALVEOUT_H
