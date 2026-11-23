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
 * K0, POS or PZYL is reported but does not force limp.
 *
 * The V4 board has one MAX22200 (8 channels, OUT0..7 = A, B, C, D, E,
 * LINE, OUT6 (POS? bench-check), TCC). The class still handles two chips so a later board
 * can add one; stm32_tcu.cpp only attaches chip 0.
 */

#define VALVE_MAX_CHIPS 2
/// General-purpose low-side outputs on spare driver channels (rev B: IC4
/// OUT1..OUT3 = J1.10 / J1.11 / J1.12, which were discrete drivers on V4).
/// They are not valves: never part of the interlock, a fault on them is
/// only reported.
#define VALVE_AUX_MAX 3
#define VALVE_NOT_FITTED 0xFF

/// Per-valve fault flags of the last PollFaults() (ValveFlags()).
enum ValveFlag
{
   VALVE_F_OCP = 0x01,  // over-current
   VALVE_F_HHF = 0x02,  // hit current not reached
   VALVE_F_OLF = 0x04,  // open load
   VALVE_F_DPM = 0x08,  // plunger movement not detected
   VALVE_F_CHIP = 0x10, // whole chip: over-temperature, reset (UVM) or SPI echo
};

class ValveOut
{
public:
   ValveOut();
   /// Attach a chip (index 0 or 1). Chips not attached count as absent.
   void AttachChip(uint8_t idx, Max22200 *chip);
   /// Set the channel for a valve: chip * 8 + channel, or VALVE_NOT_FITTED.
   void SetMap(EdsValve v, uint8_t globalCh) { map[v] = globalCh; }
   uint8_t Map(EdsValve v) const { return map[v]; }
   /// Put general-purpose output i (0..VALVE_AUX_MAX-1) on a channel, or
   /// VALVE_NOT_FITTED (the default).
   void SetAuxMap(uint8_t i, uint8_t globalCh) { if (i < VALVE_AUX_MAX) auxMap[i] = globalCh; }
   /// Current for output i in 0.1 mA; 0 switches it off. Sent by Update().
   void SetAux(uint8_t i, int32_t current_0p1mA) { if (i < VALVE_AUX_MAX) aux[i] = current_0p1mA; }
   /// Check the map. False if an entry is out of range, on a missing chip
   /// or shared with another valve or output. Also false if A..E or SYS is
   /// unmapped.
   bool Validate() const;

   /** Bring the chips up (data sheet Figure 6). Returns false on a map
    * error or if a chip reports a fault after init. Every channel is left
    * off. halfScaleMask: bit per global channel (chip * 8 + ch) that runs
    * with HFS = 1 (full scale halved).
    */
   bool Init(const Max22200Config &cfg, uint16_t halfScaleMask = 0);

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
   /// ValveFlag bits of one valve from the last poll (0 if not fitted).
   uint8_t ValveFlags(EdsValve v) const { return vflags[v]; }
   /// ValveFlag bits of general-purpose output i from the last poll.
   uint8_t AuxFlags(uint8_t i) const { return i < VALVE_AUX_MAX ? aflags[i] : 0; }
   /// True if a chip reset itself (UVM) or the SPI echo check failed.
   bool ChipLost() const { return lost; }

private:
   Max22200 *chips[VALVE_MAX_CHIPS];
   uint8_t map[EDS_COUNT];
   uint8_t flags[VALVE_MAX_CHIPS];
   uint8_t vflags[EDS_COUNT];
   uint8_t auxMap[VALVE_AUX_MAX];
   int32_t aux[VALVE_AUX_MAX];
   uint8_t aflags[VALVE_AUX_MAX];
   bool ok;
   bool lost;
};

#endif // VALVEOUT_H
