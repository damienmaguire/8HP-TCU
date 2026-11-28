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
/*
 * Pressure valve (EDS) layer: pressure to coil current through the
 * calibration curves, oil-temperature correction and slew limit (the
 * driver chip adds the dither).
 * Currents are in 0.1 mA, pressures in mbar.
 */
#include "eds.h"
#include "calset.h"
#include "caltypes.h"

// Valves whose current rises with pressure use the rising correction map.
static bool IsRising(EdsValve v) {
  return v == EDS_A || v == EDS_B || v == EDS_K0;
}

// Default settings (overwritten by Configure); every valve at 0.
Eds::Eds() : safe(false) {
  cfg.iMin = 500;
  cfg.iMax = 8500;
  cfg.slewPer10ms = 600;
  cfg.tempCorrection = true;
  for (int i = 0; i < EDS_COUNT; i++)
    target[i] = setpoint[i] = 0;
}

// Pressure to coil current from the valve's curve, 0.1 mA.
int32_t Eds::PressureToCurrent(EdsValve v, int32_t p_mbar) {
  if (v >= EDS_NUM_PRESSURE)
    return 0;
  if (p_mbar < 0)
    p_mbar = 0;
  return Cal::Curve(Cal::Eds()[v], p_mbar);
}

// Correct a coil current for oil temperature (OEM KF_KIEDSCG* maps).
int32_t Eds::TempCorrection(EdsValve v, int32_t i_0p1mA, int32_t atfTemp) {
  if (v >= EDS_NUM_PRESSURE)
    return 0;
  // The correction maps have their current axis in mA.
  const CalMap &m =
      IsRising(v) ? Cal::Active().global->kieds_rising : Cal::Active().global->kieds_falling;
  return Cal::Map(m, i_0p1mA / 10, atfTemp);
}

// Pressure to the current actually commanded: curve, temperature, ceiling.
int32_t Eds::CommandCurrent(EdsValve v, int32_t p_mbar, int32_t atfTemp,
                            bool tempCorrection, int32_t iMax) {
  if (v >= EDS_NUM_PRESSURE)
    return 0;
  int32_t i = PressureToCurrent(v, p_mbar);
  if (tempCorrection && p_mbar > 0)
    i += TempCorrection(v, i, atfTemp);
  if (i < 0)
    i = 0;
  if (i > iMax)
    i = iMax;
  return i;
}

// Ask for a pressure on a valve; the current follows at the slew rate.
void Eds::SetPressure(EdsValve v, int32_t p_mbar, int32_t atfTemp) {
  if (v >= EDS_NUM_PRESSURE)
    return;
  SetCurrent(v, CommandCurrent(v, p_mbar, atfTemp, cfg.tempCorrection, cfg.iMax));
}

// Ask for a current directly (POS, PZYL, bench test).
void Eds::SetCurrent(EdsValve v, int32_t i) {
  if (v >= EDS_COUNT)
    return;
  if (i < 0)
    i = 0;
  int32_t lim = v == EDS_PZYL ? EDS_PZYL_IMAX : cfg.iMax;
  if (i > lim)
    i = lim;
  target[v] = i;
}

// Move each setpoint towards its target, limited by the slew rate.
void Eds::Tick10ms() {
  for (int v = 0; v < EDS_COUNT; v++) {
    if (v == EDS_PZYL) {
      setpoint[v] = target[v]; // on / off, never slewed, kept in the safe state
      continue;
    }
    if (safe) {
      setpoint[v] = 0;
      continue;
    }
    int32_t d = target[v] - setpoint[v];
    // A zero slew setting switches straight through.
    if (cfg.slewPer10ms <= 0) {
      setpoint[v] = target[v];
    } else if (d > cfg.slewPer10ms) {
      setpoint[v] += cfg.slewPer10ms;
    } else if (d < -cfg.slewPer10ms) {
      setpoint[v] -= cfg.slewPer10ms;
    } else {
      setpoint[v] = target[v];
    }
  }
}

// Every valve to 0 at once (coils off = mechanical limp gear), except
// PZYL: see eds.h and parksafe.h.
void Eds::SafeState() {
  safe = true;
  for (int v = 0; v < EDS_COUNT; v++) {
    if (v != EDS_PZYL)
      target[v] = setpoint[v] = 0;
  }
}
