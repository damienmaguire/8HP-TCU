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
#ifndef ENGAGE_H
#define ENGAGE_H

#include <stdint.h>

/* Gear-in from neutral: N -> 1 (D) and N -> R.
 *
 * Source: 330E_OPEN_TCU_CONTROL_SPEC.md section 5.7 and
 * 330E_SAS_FILL_RAMP.md section 6 [C names]. The OEM fills the two
 * non-power-flow elements first (V1, V2: KL_TFV1_n, KL_TFCGV2_n, closing
 * gradients KL_PGRASYNV1/V2_n) and then closes the power-flow element, the
 * last clutch in (KL_TVKZU_n delay, KF_PKZU_n, KL_PGRASYN_n).
 *
 * Which clutch closes last depends on the gearbox variant (Ian, QA2):
 *   330e (table 0x4C214): B for both N->1 and N->R; A+C pre-filled for 1st,
 *                         A+D for R. This matches the calibration in use.
 *   F11 8HP (table 0x5BA08): C for 1st and D for R, with A+B pre-filled.
 * The variant is the Variant parameter; only this table differs for now.
 *
 * Closing (Ian, QA2):
 *  - Normal: the OEM gradients (1-15 bar/s against torque), and the gear
 *    only counts as in once there has been no slip for 200 ms after sync.
 *  - Fast: a faster, rate-limited close (default 30 bar/s, 20-50) that
 *    reaches full pressure well inside 1.5 s. Only allowed when no slip has
 *    been confirmed for 50-100 ms on valid speed sensors, the motor torque
 *    is near 0 (the TCU asks for it to be held there until the gear is
 *    clamped, see TorqueHold()), and line pressure is adequate.
 *
 * The OEM closed-loop gear-in (KF_WNGTNDSYNCGGE_n) is not implemented.
 */

enum EngageVariant
{
   ENG_VAR_330E = 0, // OEM table 0x4C214: B last for 1st and R
   ENG_VAR_F11 = 1,  // OEM table 0x5BA08: C last for 1st, D last for R
};

enum EngagePhase
{
   ENG_IDLE = 0,
   ENG_FILL,  // V1/V2 filling, power-flow element waiting
   ENG_POWER, // power-flow element filling and ramping
   ENG_CLOSE, // power-flow element closing to full pressure
   ENG_DONE,
};

struct EngageConfig
{
   int32_t pMax;       // full apply pressure, mbar
   int32_t fillEqMs;   // fill-equalise time for the power-flow element
   int32_t rampMs;     // ramp to KF_PKZU_n
   int32_t syncBand;   // |n_t - n_ab x ratio| below this = no slip, rpm
   int32_t fastCloseGrad; // fast closing gradient, mbar/s (Ian: ~30000, 20000-50000)
   int32_t syncConfirmMs; // no slip this long before the fast close, ms (Ian: 50-100)
   int32_t postSyncMs;    // normal close: no slip this long before "in gear", ms (OEM 200)
   int32_t tqZeroNm;      // |m_t| at or below this counts as "held near 0", Nm
   uint8_t variant;       // EngageVariant
};

struct EngageInputs
{
   int32_t m_t, n_t, n_ab, temp; // Nm, rpm, rpm, degC
   bool speedsValid;             // input and output speed sensors working
   bool lineOk;                  // commanded line pressure is adequate
};

class Engage
{
public:
   Engage();
   /// gear is GEAR_1 or GEAR_R. Returns false for any other gear.
   bool Start(int gear, const EngageConfig &cfg);
   void Tick(const EngageInputs &in); // 10 ms
   void Abort() { phase = ENG_IDLE; }

   bool Active() const { return phase != ENG_IDLE && phase != ENG_DONE; }
   bool Done() const { return phase == ENG_DONE; }
   uint8_t Phase() const { return (uint8_t)phase; }
   int Gear() const { return gear; }
   /// Pressure for each of A..E, mbar.
   int32_t Pressure(int clutch) const { return p[clutch]; }
   int PowerClutch() const { return clPower; }
   /// The motor torque should be held near 0 until the gear is clamped.
   bool TorqueHold() const { return Active(); }
   /// The fast close is running (for logging/tests).
   bool FastClose() const { return fast; }

   /// Last clutch in for gear (GEAR_1 or GEAR_R) on a variant, -1 if none.
   static int PowerFlowClutch(int gear, int variant);

private:
   int32_t FillPressure(int clutch, int32_t tInPhase, int32_t fillTotal,
                        const EngageInputs &in) const;

   EngageConfig cfg;
   EngagePhase phase;
   int gear, idx, clV1, clV2, clPower;
   int32_t t, tPower;
   int32_t tNoSlip;  // ms with no slip on valid sensors
   int32_t tAtMax;   // ms with every element at full pressure
   int32_t fillV1, fillV2; // total fill times, ms
   int32_t p[5];
   int32_t pStart;
   bool v1Closing, v2Closing;
   bool fast;
};

#endif // ENGAGE_H
