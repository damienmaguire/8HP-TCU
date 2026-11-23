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
 * gradients KL_PGRASYNV1/V2_n) and then closes the power-flow element
 * (KL_TVKZU_n delay, KF_PKZU_n, KL_PGRASYN_n).
 *
 * Which element is the power-flow one is not stated in Ian's notes. This
 * code assumes it is the element that tells 1st and R apart: C for 1st
 * (A B C) and D for R (A B D), with A and B as V1 and V2 [H, ask Ian].
 * It is a table at the top of engage.cpp so it is easy to change.
 *
 * The OEM closed-loop gear-in (KF_WNGTNDSYNCGGE_n) is not implemented; the
 * power-flow element is closed open-loop with the OEM gradients. The OEM
 * low-torque closing gradients are slow (1 bar/s), which suits an engine
 * dragging the turbine but not an EV motor sitting at 0 rpm. So once the
 * input speed matches the gear (no slip) the elements close with the
 * faster of the OEM gradient and syncCloseGrad [Sam].
 */

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
   int32_t syncCloseGrad; // closing gradient once there is no slip, mbar/s
};

struct EngageInputs
{
   int32_t m_t, n_t, n_ab, temp; // Nm, rpm, rpm, degC
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

   static int PowerFlowClutch(int gear);

private:
   int32_t FillPressure(int clutch, int32_t tInPhase, int32_t fillTotal,
                        const EngageInputs &in) const;

   EngageConfig cfg;
   EngagePhase phase;
   int gear, idx, clV1, clV2, clPower;
   int32_t t, tPower;
   int32_t fillV1, fillV2; // total fill times, ms
   int32_t p[5];
   int32_t pStart;
   bool v1Closing, v2Closing;
};

#endif // ENGAGE_H
