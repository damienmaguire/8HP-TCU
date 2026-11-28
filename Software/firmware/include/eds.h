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
#ifndef EDS_H
#define EDS_H

#include <stdint.h>

/* Hydraulic output layer: pressure command -> solenoid current setpoint.
 *
 * Source: Ian's 330E_EDS_PWM.md. The OEM chain is
 *   pressure (mbar) -> KL_EDSKV<x> curve -> temperature correction
 *   -> slew limit (0.03 A per 5 ms) -> 125 Hz dither -> current regulator.
 * On the V5 board the current regulator is the TLE92466ED (tle92466.h),
 * which also makes the dither, so this layer stops at a current setpoint
 * per valve. ValveOut sends it to the chips every 1 ms when it changes.
 *
 * Valve sense [C]: A, B and K0 are normally low (more current = more
 * pressure). C, D, E and SYS are normally high (more current = less
 * pressure). With no current C+D+E apply (6th gear) and line pressure is
 * at its maximum. The curves already encode this, so callers only ever
 * deal in pressure.
 *
 * Units: current in 0.1 mA, pressure in mbar, temperature in degC.
 */

enum EdsValve
{
   EDS_A = 0,
   EDS_B,
   EDS_C,
   EDS_D,
   EDS_E,
   EDS_SYS,  // line pressure regulator
   EDS_K0,   // hybrid disconnect clutch, or the TCC lock-up (IC11 CH3, "TCC")
   EDS_POS,  // position valve, current command only (IC11 CH2, "PREL")
   EDS_PZYL, // park-lock cylinder valve (MvPZyl), current command only (IC10 CH5)
   EDS_COUNT
   // There is no Sol1 valve. The park-hold magnet PHOLD (IC10 CH4) is not
   // a valve: ValveOut drives it as an aux output.
};

/// PZYL is an on / off valve with a pull-in current above the valve
/// ceiling (IMaxCoil), so it has its own limit: the chip's 1.5 A.
#define EDS_PZYL_IMAX 15000

/// Valves A..K0 have a pressure-to-current curve.
#define EDS_NUM_PRESSURE 7

struct EdsConfig
{
   int32_t iMin;          // 0.1 mA, bottom "parking" current (500 = 50 mA)
   int32_t iMax;          // 0.1 mA, top current (8500 = 850 mA)
   int32_t slewPer10ms;   // 0.1 mA per 10 ms. OEM 0.03 A per 5 ms = 600
   bool tempCorrection;   // apply KF_KIEDSCGSTACE / KF_KIEDSCGFAACE
};

class Eds
{
public:
   Eds();
   void Configure(const EdsConfig &c) { cfg = c; }
   const EdsConfig &Config() const { return cfg; }

   /// Pressure command for A..K0 in mbar. Called by the 10 ms layer.
   void SetPressure(EdsValve v, int32_t p_mbar, int32_t atfTemp);
   /// Direct current command in 0.1 mA (POS, PZYL, tests).
   void SetCurrent(EdsValve v, int32_t i_0p1mA);
   /// 10 ms: move each setpoint towards its target, slew limited. PZYL
   /// is not slewed (on / off valve with a pull-in pulse).
   void Tick10ms();

   /// Current to send to the driver for a valve, 0.1 mA (the chip adds
   /// the dither).
   int32_t Output(EdsValve v) const { return setpoint[v]; }
   /// Slewed setpoint, 0.1 mA (same as Output()).
   int32_t Setpoint(EdsValve v) const { return setpoint[v]; }
   int32_t Target(EdsValve v) const { return target[v]; }

   /** Drop every valve to 0 A at once (no slew). 0 A = 6th gear limp.
    * SAFETY: PZYL is left alone. It keeps whatever the caller sets with
    * SetCurrent(EDS_PZYL, ...), which in the safe state comes from the
    * park-lock rule (parksafe.h), so the park lock cannot drop in while
    * the car moves. */
   void SafeState();
   bool InSafeState() const { return safe; }
   /// Leave the safe state; outputs restart from 0 under the slew limit.
   void Release() { safe = false; }

   /// Pressure -> current through the OEM curve (no slew, no dither).
   static int32_t PressureToCurrent(EdsValve v, int32_t p_mbar);
   /// OEM temperature correction to add to a current, 0.1 mA.
   static int32_t TempCorrection(EdsValve v, int32_t i_0p1mA, int32_t atfTemp);
   /** Target current for a pressure command, 0.1 mA: OEM curve, optional
    * temperature correction, clamped to 0..iMax. This is exactly what
    * SetPressure() loads as the target, so the interlock can check the
    * same numbers. A command of 0 mbar is the valve's end stop and gets no
    * temperature correction (so "open" stays clearly open). */
   static int32_t CommandCurrent(EdsValve v, int32_t p_mbar, int32_t atfTemp,
                                 bool tempCorrection, int32_t iMax);

private:
   EdsConfig cfg;
   int32_t target[EDS_COUNT];
   int32_t setpoint[EDS_COUNT];
   bool safe;
};

#endif // EDS_H
