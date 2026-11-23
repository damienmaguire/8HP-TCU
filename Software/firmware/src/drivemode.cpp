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
#include "drivemode.h"

namespace DriveModes {

DriveModeMod Neutral() {
  DriveModeMod m = {0, 0, 100, 0, false, 0, 0};
  return m;
}

TcuConfig Apply(const TcuConfig &n, const DriveModeMod &m) {
  TcuConfig c = n;
  // Shift speeds, kept below the over-rev limit and with the down point
  // under the up point (the lugging rule needs AutoUpRpm - 200 above it).
  c.autoUpRpm = n.autoUpRpm + m.upOfsRpm;
  if (m.holdToRedline)
    c.autoUpRpm = m.redlineRpm;
  if (c.autoUpRpm > n.manMaxRpm)
    c.autoUpRpm = n.manMaxRpm;
  c.autoDownRpm = n.autoDownRpm + m.downOfsRpm;
  if (c.autoDownRpm > c.autoUpRpm - 500)
    c.autoDownRpm = c.autoUpRpm - 500;
  // Shift time: quicker only, never below the floor.
  int32_t t = m.timePct;
  if (t > 100)
    t = 100;
  if (t < DRIVEMODE_TIME_MIN_PCT)
    t = DRIVEMODE_TIME_MIN_PCT;
  c.sas.timePct = t;
  // Earlier lock-up (only used when channel 7 is TCC).
  if (m.tccOfsRpm > 0) {
    c.tccOnRpm = n.tccOnRpm - m.tccOfsRpm;
    c.tccOffRpm = n.tccOffRpm - m.tccOfsRpm;
    if (c.tccOffRpm < 0)
      c.tccOffRpm = 0;
    if (c.tccOnRpm < c.tccOffRpm)
      c.tccOnRpm = c.tccOffRpm;
  }
  c.noLiftUpshift = m.holdToRedline;
  c.liftTqNm = m.liftTqNm;
  return c;
}

} // namespace DriveModes
