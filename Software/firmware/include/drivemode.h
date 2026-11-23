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
#ifndef DRIVEMODE_H
#define DRIVEMODE_H

#include "calset.h"
#include "tcu.h"
#include <stdint.h>

/* DriveMode (Normal / Sport / Track) as parameter-driven modifiers on the
 * Normal calibration. Normal leaves everything as it is.
 *
 * Sport and Track change, from their own parameters:
 *  - the Auto up / down shift speeds (an rpm offset on AutoUpRpm and
 *    AutoDownRpm);
 *  - the shift time scale (SasConfig::timePct, see sas.cpp TScale): the
 *    fill-equalise time and the ramps run quicker, never below
 *    DRIVEMODE_TIME_MIN_PCT, and the quick-fill pulse, sync detection and
 *    the shift timeout are never touched;
 *  - earlier TCC lock-up (an rpm offset off TccOnRpm / TccOffRpm), only
 *    where channel 7 runs as TCC.
 * Track also holds the gear to TrackRedline in Auto and makes no upshift
 * on lift-off (input torque at or below TrackLiftNm), except that it
 * always upshifts at ManMaxRpm so the input cannot over-rev.
 *
 * The interlock, line pressure, gear-in, the rpm limits of a tip, the
 * CAN-loss rules and every other safety limit are not touched. All
 * Sport / Track defaults are PLACEHOLDERS (not measured, not OEM), kept
 * small on purpose.
 *
 * OEM Sport tables: Cal::SelectMode() also switches the shift tables to
 * VariantCal::upMode / downMode (330e: the Q030 Sport set; F11: its
 * programs 1 and 2). Set a mode's modifiers to neutral (0 rpm, 100 %)
 * if only the tables are wanted.
 */

#define DRIVEMODE_TIME_MIN_PCT 50 // floor on the shift time scale

struct DriveModeMod
{
   int32_t upOfsRpm;    // added to AutoUpRpm
   int32_t downOfsRpm;  // added to AutoDownRpm
   int32_t timePct;     // shift time scale, %, 100 = unchanged
   int32_t tccOfsRpm;   // TCC locks this much earlier (TccOnRpm / TccOffRpm)
   bool holdToRedline;  // Track: Auto upshifts at redlineRpm only
   int32_t redlineRpm;  // Track redline (input speed)
   int32_t liftTqNm;    // Track: no upshift at or below this input torque
};

namespace DriveModes
{
/// Normal: no change.
DriveModeMod Neutral();
/// Apply a mode's modifiers to the Normal configuration.
TcuConfig Apply(const TcuConfig &normal, const DriveModeMod &m);
} // namespace DriveModes

#endif // DRIVEMODE_H
