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
#include "tle92466.h"
#include <stdint.h>

/* Glue between the valve setpoints (eds.h) and the two TLE92466ED chips.
 *
 * Which chip channel drives which valve is configuration (the ChanX
 * parameters): one entry per valve, "chip * 8 + channel" with channel
 * 0..5, or VALVE_NOT_FITTED. The map is checked before use: an
 * out-of-range entry or two valves on one channel is a configuration
 * error and nothing is driven.
 *
 * V5 board (Bob, netlist Stm32_TCU_V5):
 *   IC10 (chip 0): CH0 A, CH1 B, CH2 C, CH3 D, CH4 PHOLD, CH5 PZYL
 *   IC11 (chip 1): CH0 E, CH1 LINE (SYS), CH2 PREL (POS), CH3 TCC / K0,
 *                  CH4 and CH5 not connected (left off, faults ignored)
 * PHOLD (the park-hold magnet) is not a valve: it is "aux" output 0.
 *
 * Faults are read every 100 ms. A fault on a pressure valve (A..E, SYS)
 * means the TCU can no longer control pressure, so the caller goes to the
 * safe state (6th gear, full line pressure). A fault on K0, POS, PZYL or
 * PHOLD is reported but does not force limp.
 *
 * Park lock: PZYL and PHOLD must not drop while the car moves (see
 * parksafe.h). So on those two channels the chip's own open-load / short
 * check, which would switch the channel off, is never enabled; their
 * measured current is reported instead. If the chip still switches a
 * channel off (over-current, over-temperature) or falls back to Config
 * Mode by itself, PollFaults() turns it back on at once. Nothing here ever
 * resets the chips or pulls EN low.
 */

#define VALVE_MAX_CHIPS 2
/// Non-valve outputs on driver channels. Only one on V5: the park-hold
/// magnet PHOLD on IC10 CH4. Never part of the interlock, a fault on it
/// is only reported.
#define VALVE_AUX_MAX 1
#define VALVE_AUX_PHOLD 0
#define VALVE_NOT_FITTED 0xFF
/// Polls in a row without a good SPI answer before a chip counts as lost
/// (one bad CRC on a noisy line must not put the car in limp mode).
#define VALVE_CHIP_MISSES 3
/// Smallest setpoint / measured current for the resistance estimate, 0.1 mA.
#define RCOIL_MIN_I 2000

/// Per-valve fault flags of the last PollFaults() (ValveFlags()).
enum ValveFlag
{
   VALVE_F_OCP = 0x01,  // over-current (chip switched the channel off)
   VALVE_F_IERR = 0x02, // measured current far from the setpoint. Report
                        // only: never in PollFaults()' return value, so it
                        // never forces the safe state or switches anything off
   VALVE_F_OLF = 0x04,  // open load (ON- or OFF-state check)
   VALVE_F_SG = 0x08,   // output shorted to ground (coil always on)
   VALVE_F_CHIP = 0x10, // whole chip: over-temperature, no answer, fell to Config Mode
};

/// TLE supply / status summary of both chips (SupplyFlags()).
enum TleSupFlag
{
   TLE_SUP_NOT_MISSION0 = 0x001, // IC10 not in Mission Mode (outputs off)
   TLE_SUP_NOT_MISSION1 = 0x002, // IC11 not in Mission Mode
   TLE_SUP_VBAT = 0x004,  // VBAT under / over voltage
   TLE_SUP_VIO = 0x008,   // VIO (3.3 V) fault
   TLE_SUP_VDD = 0x010,   // VDD (5 V) fault
   TLE_SUP_TEMP = 0x020,  // chip over-temperature warning or error
   TLE_SUP_CLOCK = 0x040, // clock fault
   TLE_SUP_COMM = 0x080,  // SPI CRC / status error or critical fault frame
   TLE_SUP_RESET = 0x100, // a chip reset itself (power-on or RESN event)
   TLE_SUP_RECOVER = 0x200, // a chip was put back into Mission Mode
};

class ValveOut
{
public:
   ValveOut();
   /// Attach a chip (index 0 = IC10, 1 = IC11). Chips not attached count as absent.
   void AttachChip(uint8_t idx, Tle92466 *chip);
   /// Set the channel for a valve: chip * 8 + channel, or VALVE_NOT_FITTED.
   void SetMap(EdsValve v, uint8_t globalCh) { map[v] = globalCh; }
   uint8_t Map(EdsValve v) const { return map[v]; }
   /// Put aux output i on a channel, or VALVE_NOT_FITTED (the default).
   void SetAuxMap(uint8_t i, uint8_t globalCh) { if (i < VALVE_AUX_MAX) auxMap[i] = globalCh; }
   /// Current for aux output i in 0.1 mA; 0 switches it off. Sent by Update().
   void SetAux(uint8_t i, int32_t current_0p1mA) { if (i < VALVE_AUX_MAX) aux[i] = current_0p1mA; }
   int32_t Aux(uint8_t i) const { return i < VALVE_AUX_MAX ? aux[i] : 0; }
   /// Check the map. False if an entry is out of range, on a missing chip
   /// or shared with another valve or output. Also false if A..E or SYS is
   /// unmapped.
   bool Validate() const;

   /** Bring the chips up. Returns false on a map error or if a chip does
    * not reach Mission Mode. Every channel is left at 0 mA. From base only
    * the PWM, dither, slew and open-load threshold are used; which
    * channels are used, dithered and self-checked follows from the map.
    * openLoad = false keeps the chip's own open-load / short check off on
    * every channel. */
   bool Init(const TleConfig &base, bool openLoad);

   /// 1 ms: send the setpoints that changed to the chips.
   void Update(const Eds &eds);
   /// Every channel to 0 mA. Only when the park-lock rule allows (parksafe.h).
   void AllOff();

   /// 100 ms: read and clear chip faults, read the measured currents, and
   /// put a chip that fell out of Mission Mode back. Returns the valves
   /// with a fault as a bit mask (bit = EdsValve).
   uint16_t PollFaults();
   /// Valves whose fault forces the safe state.
   static uint16_t CriticalMask();
   /// FB_STAT bits 15:0 of a chip from the last poll, for logging.
   uint16_t ChipFlags(uint8_t idx) const { return idx < VALVE_MAX_CHIPS ? flags[idx] : 0; }
   /// TleSupFlag bits from the last poll.
   uint16_t SupplyFlags() const { return sup; }
   /// ValveFlag bits of one valve from the last poll (0 if not fitted).
   uint8_t ValveFlags(EdsValve v) const { return vflags[v]; }
   /// ValveFlag bits of aux output i from the last poll.
   uint8_t AuxFlags(uint8_t i) const { return i < VALVE_AUX_MAX ? aflags[i] : 0; }
   /** Dither amplitude (peak, 0.1 mA) for a valve at a current (0.1 mA).
    * PROVISIONAL, Ian's 330e map at 25 degC (V5_VALVE_DEFAULTS.md,
    * v5_dither_amp_330e.csv), no temperature scaling yet:
    *   A..E and TCC / K0: 0 up to 0.12 A, 0.03 A at 0.15 A, 0.07 A at
    *     0.31 A, 0.09 A at 0.39 A, 0.10 A from 0.43 to 0.55 A, 0.09 A at
    *     0.574 A, 0.07 A at 0.62 A, 0 from 0.78 A; straight lines between.
    *   LINE (SYS): a flat 0.03 A from 0.10 to 0.80 A, 0 outside.
    *   PREL (POS), PZYL and PHOLD: none. */
   static int32_t DitherFor(EdsValve v, int32_t i_0p1mA);
   /** Largest allowed |measured - setpoint| before VALVE_F_IERR, 0.1 mA:
    * 0.12 A at setpoints of 0.4 A and more, 0.17 A below (OEM feedback
    * check, Ian [C]; PROVISIONAL, the TLE read-back may allow tighter). */
   static int32_t CurrentErrLimit(int32_t set_0p1mA);
   /// Measured average current of a valve / aux output, 0.1 mA.
   int32_t Measured(EdsValve v) const;
   int32_t AuxMeasured(uint8_t i) const;
   /** Coil resistance estimate of a valve / aux output, milliohm; 0 =
    * no valid estimate right now. Read-only diagnostic: it never changes a
    * setpoint and never trips anything.
    *
    * Why: the coils cannot be measured with a meter once the valve body is
    * sealed in the gearbox, so the coil data in the firmware (5.5 ohm shift
    * valves, about 11 ohm PZYL, 24-26 ohm PHOLD, PREL unknown) are working
    * defaults. At first power-up Damien reads these values to check and
    * correct those defaults, PZYL and PREL above all.
    *
    * How: R = VBAT x duty / average current, all three from the chip's own
    * read-back of that channel over one measurement (dither) period (see
    * Tle::CoilMilliOhm). It is only updated while the channel is on with
    * a setpoint and a measured current of at least RCOIL_MIN_I that has
    * not changed since the last 100 ms poll and is within the current-error
    * limit, and it is filtered (1/8 of each new reading). Otherwise it
    * shows 0. The estimate includes the harness and the chip's switch, so
    * it reads a little above the bare coil. */
   int32_t CoilMilliOhm(EdsValve v) const { return rShow[v]; }
   int32_t AuxCoilMilliOhm(uint8_t i) const { return i < VALVE_AUX_MAX ? rShow[EDS_COUNT + i] : 0; }
   /// True if a chip stopped answering or could not be put back into
   /// Mission Mode.
   bool ChipLost() const { return lost; }
   /// Chip temperature (hottest chip, degC), IC10's VBAT reading (mV) and
   /// the SPI error count of both chips.
   int32_t ChipTempC() const;
   int32_t VbatmV() const { return chips[0] ? chips[0]->VbatmV() : 0; }
   uint32_t CommErrors() const;

private:
   int32_t ChCurrent(uint8_t g) const;
   uint8_t CurrentCheck(int idx, uint8_t g, bool underOk);

   Tle92466 *chips[VALVE_MAX_CHIPS];
   TleConfig cfg[VALVE_MAX_CHIPS];
   uint8_t map[EDS_COUNT];
   uint16_t flags[VALVE_MAX_CHIPS];
   uint8_t misses[VALVE_MAX_CHIPS]; // polls in a row without a good answer
   uint8_t vflags[EDS_COUNT];
   uint8_t auxMap[VALVE_AUX_MAX];
   int32_t aux[VALVE_AUX_MAX];
   uint8_t aflags[VALVE_AUX_MAX];
   int32_t lastSet[EDS_COUNT + VALVE_AUX_MAX]; // setpoint at the previous poll
   int32_t rFilt[EDS_COUNT + VALVE_AUX_MAX];   // filtered resistance, milliohm (0 = none yet)
   int32_t rShow[EDS_COUNT + VALVE_AUX_MAX];   // shown: rFilt while valid, else 0
   uint16_t sup;
   bool ok;
   bool lost;
};

#endif // VALVEOUT_H
