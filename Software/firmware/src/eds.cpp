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
#include "eds.h"
#include "cal_330e.h"
#include "caltypes.h"

static bool IsRising(EdsValve v) {
  return v == EDS_A || v == EDS_B || v == EDS_K0;
}

Eds::Eds() : ms(0), safe(false) {
  cfg.iMin = 500;
  cfg.iMax = 8500;
  cfg.slewPer10ms = 600;
  cfg.ditherAmp = 300;
  cfg.ditherPeriodMs = 8;
  cfg.ditherShape = DITHER_TRIANGLE;
  cfg.tempCorrection = true;
  for (int i = 0; i < EDS_COUNT; i++) {
    target[i] = setpoint[i] = out[i] = 0;
    ditherOn[i] = i < EDS_POS; // proportional pressure valves dither
  }
}

int32_t Eds::PressureToCurrent(EdsValve v, int32_t p_mbar) {
  if (v >= EDS_NUM_PRESSURE)
    return 0;
  if (p_mbar < 0)
    p_mbar = 0;
  return Cal::Curve(calEds[v], p_mbar);
}

int32_t Eds::TempCorrection(EdsValve v, int32_t i_0p1mA, int32_t atfTemp) {
  if (v >= EDS_NUM_PRESSURE)
    return 0;
  // The correction maps have their current axis in mA.
  const CalMap &m =
      IsRising(v) ? calGlobal.kieds_rising : calGlobal.kieds_falling;
  return Cal::Map(m, i_0p1mA / 10, atfTemp);
}

void Eds::SetPressure(EdsValve v, int32_t p_mbar, int32_t atfTemp) {
  if (v >= EDS_NUM_PRESSURE)
    return;
  int32_t i = PressureToCurrent(v, p_mbar);
  if (cfg.tempCorrection)
    i += TempCorrection(v, i, atfTemp);
  SetCurrent(v, i);
}

void Eds::SetCurrent(EdsValve v, int32_t i) {
  if (v >= EDS_COUNT)
    return;
  if (i < 0)
    i = 0;
  if (i > cfg.iMax)
    i = cfg.iMax;
  target[v] = i;
}

void Eds::Tick10ms() {
  for (int v = 0; v < EDS_COUNT; v++) {
    if (safe) {
      setpoint[v] = 0;
      continue;
    }
    int32_t d = target[v] - setpoint[v];
    // On/off outputs and a zero slew setting switch straight through.
    if (cfg.slewPer10ms <= 0 || v >= EDS_SOL1) {
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

int32_t Eds::DitherValue(uint32_t t, int32_t amp, uint8_t periodMs,
                         uint8_t shape) {
  if (amp <= 0 || periodMs < 2)
    return 0;
  uint32_t p = t % periodMs;
  int32_t half = periodMs / 2;
  if (shape == DITHER_SQUARE)
    return (int32_t)p < half ? amp : -amp;
  // Triangle: -amp at p = 0, +amp at p = period/2, back to -amp.
  int32_t up = (int32_t)p <= half ? (int32_t)p : (int32_t)periodMs - (int32_t)p;
  return -amp + (2 * amp * up) / half;
}

void Eds::Tick1ms() {
  ms++;
  int32_t d = DitherValue(ms, cfg.ditherAmp, cfg.ditherPeriodMs, cfg.ditherShape);
  for (int v = 0; v < EDS_COUNT; v++) {
    int32_t i = safe ? 0 : setpoint[v];
    // Dither only inside the working range, as the OEM does (off at the
    // 50 mA and 850 mA end points), so it never pushes past either end.
    if (!safe && ditherOn[v] && v < EDS_SOL1 && i > cfg.iMin + cfg.ditherAmp &&
        i < cfg.iMax - cfg.ditherAmp)
      i += d;
    out[v] = i;
  }
}

void Eds::SafeState() {
  safe = true;
  for (int v = 0; v < EDS_COUNT; v++) {
    target[v] = setpoint[v] = out[v] = 0;
  }
}
