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
#ifndef SAS_H
#define SAS_H

#include <stdint.h>

/* Shift sequencer ("SAS" in the ZF firmware): the off-going (KAB) and
 * on-coming (KZU) clutch phase machines for one single-swap shift.
 *
 * Source: Ian's 330E_SAS_FILL_RAMP.md sections 2-4 and 9 and
 * 330E_OPEN_TCU_CONTROL_SPEC.md section 5.3. State numbers are the OEM
 * ones (z_kab @0xFFF8D6B1, z_kzu @0xFFF8D72C) so CAN2 logs can be laid next
 * to OEM traces. Everything runs on a 10 ms tick, like the OEM SAS.
 *
 * Which timeline is used:
 *  - upshift   -> KZU-led sequence (section 5.3 A): KZU fills and takes the
 *                 torque, KAB hands over, the PI ("GLS") controls the slip.
 *  - downshift -> KAB-led sequence (section 5.3 B): KAB slips so the input
 *                 speed can rise, KZU closes at the new synchronous speed.
 * v1 uses the power-on parameter sets for both power-on and coast shifts,
 * which Ian lists as an acceptable first option [I]. Coast-specific sets
 * (HSS/RSS "Schub") and nested/preparatory shifts are not implemented.
 *
 * Parts of the transition logic are Ian's [I] reading of FUN_0010831c, and
 * a few gaps are filled with simple choices that are marked [Sam] below.
 * These should be checked against OEM logs once a multi-gear drive is
 * captured.
 */

// KZU (on-coming) states, OEM numbering
enum KzuState
{
   KZU_IDLE = 0,
   KZU_WAIT = 1,
   KZU_QUICK_FILL = 2,
   KZU_FILL_EQ_UP = 4,
   KZU_FILL_EQ_DOWN = 5,
   KZU_RAMP1 = 9,
   KZU_RAMP2 = 10,
   KZU_HOLD_SYNC = 12,
   KZU_RAMP3 = 13,
   KZU_TIMEOUT = 14,
   KZU_CLOSE_DOWN = 18,
};

// KAB (off-going) states, OEM numbering
enum KabState
{
   KAB_IDLE = 0,
   KAB_DELAY = 1,
   KAB_HOLD_UP = 2,
   KAB_RAMP_UP = 3,
   KAB_SHIFT_P_UP = 4,
   KAB_TAKEOVER = 9,
   KAB_HOLD_DOWN = 10,
   KAB_RAMP_DOWN = 11,
   KAB_SLIP_DOWN = 12,
   KAB_RAMP_SYNC = 13,
   KAB_HOLD_SYNC = 14,
   KAB_OVERLAP = 15,
   KAB_END = 16,
};

/// Inputs refreshed every 10 ms by the controller.
struct SasInputs
{
   int32_t m_t;      // input torque, Nm (positive = drive)
   int32_t n_t;      // input (turbine) speed, rpm
   int32_t n_mot;    // motor speed, rpm (same as n_t for a direct EV drive)
   int32_t temp;     // ATF temperature, degC
   int32_t ndSyn;    // n_t - sync speed of the target gear, rpm
   int32_t ndSynIst; // n_t - sync speed of the current gear, rpm
   int32_t tSyn;     // predicted ms until the target sync, SAS_TSYN_NONE if not approaching
   int32_t dnt;      // filtered input speed gradient, rpm/s
};

#define SAS_TSYN_NONE 0x7FFFFFFF

/// Tuning that is not in the OEM calibration.
struct SasConfig
{
   int32_t pMax;        // full apply pressure, mbar (OEM ramps end at 12.75 bar)
   int32_t syncBand;    // |ndSyn| below this counts as synchronous, rpm
   int32_t glsGainPct;  // scale on the OEM GLS gains, 0 = GLS off
   int32_t rampMaxDownMs; // timeout ramp for downshifts (no OEM value in the down set)
};

struct SasOutputs
{
   int32_t pKzu;      // on-coming clutch pressure, mbar
   int32_t pKab;      // off-going clutch pressure, mbar
   int32_t pSysMin;   // minimum line pressure during this shift, mbar
   int32_t tqDelta;   // torque intervention request, Nm (negative = reduce)
   int32_t gls;       // GLS correction currently applied, mbar
   uint8_t zKzu;      // KZU state (OEM numbering)
   uint8_t zKab;      // KAB state (OEM numbering)
   bool inertia;      // inertia (speed change) phase is running
   bool done;         // both clutches finished: the new gear is in
   bool timedOut;     // shift ran past KL_TMAXCG
};

class SasShift
{
public:
   SasShift();

   /** Start a single-swap shift.
    * \param from,to      gears (adjacent, 1..8)
    * \param offClutch    element released (Clutch enum)
    * \param onClutch     element applied
    * \param cfg          tuning
    * \param in           inputs at the moment the shift starts
    * \return false if the pair is not adjacent (no calibration)
    */
   bool Start(int from, int to, int offClutch, int onClutch,
              const SasConfig &cfg, const SasInputs &in);
   /// Run one 10 ms step.
   void Tick(const SasInputs &in);
   /// Abort: both outputs go to idle. The caller decides the safe state.
   void Abort();

   bool Active() const { return active; }
   bool IsUpshift() const { return up; }
   int From() const { return from; }
   int To() const { return to; }
   int OffClutch() const { return offCl; }
   int OnClutch() const { return onCl; }
   const SasOutputs &Out() const { return out; }

private:
   void TickUpKzu(const SasInputs &in);
   void TickUpKab(const SasInputs &in);
   void TickDownKzu(const SasInputs &in);
   void TickDownKab(const SasInputs &in);
   void EnterKzu(uint8_t s);
   void EnterKab(uint8_t s);
   int32_t GlsStep(const SasInputs &in);

   SasConfig cfg;
   SasOutputs out;
   bool active;
   bool up;
   int from, to, offCl, onCl, pair;
   int32_t tShift;  // ms since shift start
   int32_t tKzu;    // ms in current KZU state
   int32_t tKab;    // ms in current KAB state
   int32_t fillStart;   // tShift when quick fill began
   int32_t tFillEnd;    // predicted tShift of KZU fill end (upshift)
   int32_t tKabRampEnd; // tShift at which the KAB ramp must end (upshift)
   int32_t pKzuStart;   // pressure at the start of a ramp
   int32_t pKabStart;
   int32_t pKzuSync;    // pressure when the final closing phase began
   int32_t tInSync;     // ms spent inside the sync band
   int32_t glsInt;      // GLS integrator, mbar
   int32_t glsTarget;   // target input speed gradient, rpm/s
   bool closingFinal;   // upshift: past T_SYN, ramping to max
};

#endif // SAS_H
