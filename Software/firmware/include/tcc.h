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
#ifndef TCC_H
#define TCC_H

/* Torque converter lock-up clutch (TCC, channel 7 on the F11), the
 * scheduled mode (TccMode = Schedule; TccMode = Simple keeps the plain
 * on / off rpm rule in tcu.cpp).
 *
 * 1. Schedule: per gear, the input speed above which the clutch closes,
 *    as a function of pedal (0..100 %); it opens again TccHyst rpm below
 *    that. Below TccMinRpm it is always open. PLACEHOLDER table: later
 *    lock-up with more pedal, earlier in high gears; not measured.
 * 2. Control: PI on the slip (engine speed - turbine speed). Target slip
 *    0 (closed), or TccSlipTgt (controlled slip) at or below TccSlipGear
 *    and below TccSlipRpm. Pressure = start pressure + PI, limited to
 *    0..TccPress. Opening is immediate (pressure 0, integrator reset).
 * Never during a shift, gear-in, N / P or R: the caller only asks with
 * `allowed` in a steady forward gear. */

#include <stdint.h>

enum TccMode
{
   TCCMODE_SIMPLE = 0,   // TccOnRpm / TccOffRpm rule (tcu.cpp)
   TCCMODE_SCHEDULE = 1, // this module
};

struct TccConfig
{
   int32_t minRpm;     // always open below this input speed, rpm
   int32_t hystRpm;    // off = on - hyst, rpm
   int32_t kp10;       // P gain, 0.1 mbar per rpm of slip error
   int32_t ki10;       // I gain, 0.1 mbar per rpm per second
   int32_t startP;     // pressure at the start of closing, mbar
   int32_t maxP;       // TccPress, mbar
   int32_t slipTgt;    // controlled slip target, rpm (0 = off)
   int32_t slipGear;   // controlled slip at or below this gear
   int32_t slipRpm;    // ... and below this input speed, rpm
};

class Tcc
{
public:
   Tcc() : closed(false), integ(0), p(0) {}
   void Reset() { closed = false; integ = 0; p = 0; }
   /** Every 10 ms. allowed: steady forward gear (see above). Returns the
    * channel 7 pressure, mbar. */
   int32_t Tick10ms(const TccConfig &c, bool allowed, int gear, int32_t pedalPct,
                    int32_t nT, int32_t nMot);
   bool Closed() const { return closed; }
   int32_t Target(const TccConfig &c, int gear, int32_t nT) const;
   /// Schedule: lock-up on speed for a gear and pedal, rpm.
   static int32_t OnRpm(int gear, int32_t pedalPct);

private:
   bool closed;
   int32_t integ; // integrator, 0.001 mbar
   int32_t p;
};

#endif // TCC_H
