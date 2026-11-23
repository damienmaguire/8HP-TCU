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
#ifndef TORQUEIN_H
#define TORQUEIN_H

/* Input torque source. The shift pressures are driven by input torque, so
 * the TCU always needs a number it can trust:
 *  1. the torque the VCU / engine ECU sends on CAN (default);
 *  2. if that value is the "not available" code 0x8000 or 0x7FFF (an ECU
 *     convention) or frozen, and COMM_TQ_MODEL = true: an estimate from engine
 *     speed and manifold pressure (MAP);
 *  3. otherwise the safe substitute CanLossTq (600 Nm by default, the OEM
 *     substitute): high torque means high clamping pressure, so a clutch
 *     never slips for lack of torque information.
 *
 * Model: Nm = table(rpm, MAP up to 100 kPa) x COMM_TQ_MODEL_SCALE %; above
 * 100 kPa (boost) the 100 kPa value is scaled by MAP / 100 and by the
 * boost multiplier COMM_TQ_BOOST_PCT %. The table is a PLACEHOLDER shape for a
 * generic engine, 1.0 = 100 % of a 300 Nm peak; scale it with
 * COMM_TQ_MODEL_SCALE = (real peak Nm / 300) x 100, then check it reads about
 * friction torque (10-20 Nm) at idle and at steady free revs. */

#include <stdint.h>

enum TorqueSource
{
   TQSRC_CAN = 0,        // CAN value used as sent
   TQSRC_MODEL = 1,      // rpm x MAP model
   TQSRC_SUBSTITUTE = 2, // CanLossTq
   TQSRC_PARAM = 3,      // CmdSource = Param (bench)
};

struct TorqueModelIn
{
   int32_t rpm;       // engine / motor speed, rpm
   int32_t mapKpa;    // manifold pressure, kPa absolute
   int32_t scalePct;  // COMM_TQ_MODEL_SCALE, %
   int32_t boostPct;  // COMM_TQ_BOOST_PCT, % (100 = MAP ratio only)
};

namespace TorqueIn
{
/// False for the "not available" codes 0x8000 and 0x7FFF.
bool RawValid(int16_t raw);
/// Model estimate, Nm.
int32_t Model(const TorqueModelIn &m);
/** Pick the torque to use. canNm = the CAN value; canUsable = CAN alive,
 * not a "not available" code and not frozen. Returns the source
 * (TorqueSource) and writes the Nm. */
uint8_t Select(int32_t canNm, bool canUsable, bool modelOn,
               const TorqueModelIn &m, int32_t substituteNm, int32_t *nm);
}

/* Frozen-signal check: a value that does not change at all for timeMs
 * while the engine speed moves by more than rpmDelta is taken as stuck
 * (a dead sensor or a frozen CAN producer). timeMs = 0 switches it off. */
class FrozenCheck
{
public:
   FrozenCheck() : last(0), rpmMin(0), rpmMax(0), ms(0), frozen(false), init(false) {}
   /// Every 10 ms. Returns true while the value is frozen.
   bool Tick(int32_t value, int32_t rpm, int32_t timeMs, int32_t rpmDelta);
   bool Frozen() const { return frozen; }

private:
   int32_t last, rpmMin, rpmMax;
   int32_t ms;
   bool frozen, init;
};

#endif // TORQUEIN_H
