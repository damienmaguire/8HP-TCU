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
#include "engage.h"
#include "cal_330e.h"
#include "caltypes.h"
#include "gears.h"

#define ENG_TICK_MS 10

/* Power-flow element per engage gear [H]. See engage.h. */
int Engage::PowerFlowClutch(int g) {
  if (g == GEAR_1)
    return CL_C;
  if (g == GEAR_R)
    return CL_D;
  return -1;
}

Engage::Engage()
    : cfg(), phase(ENG_IDLE), gear(0), idx(0), clV1(CL_A), clV2(CL_B),
      clPower(-1), t(0), tPower(0), fillV1(0), fillV2(0), pStart(0),
      v1Closing(false), v2Closing(false) {
  for (int i = 0; i < 5; i++)
    p[i] = 0;
}

bool Engage::Start(int g, const EngageConfig &c) {
  clPower = PowerFlowClutch(g);
  if (clPower < 0)
    return false;
  cfg = c;
  gear = g;
  idx = g == GEAR_1 ? 0 : 1;
  // The two other elements of the gear are V1 and V2, lowest letter first.
  uint8_t rest = Gears::ClutchMask(g) & ~CLUTCH_BIT(clPower);
  clV1 = clV2 = -1;
  for (int cl = 0; cl < CL_COUNT; cl++) {
    if (rest & CLUTCH_BIT(cl)) {
      if (clV1 < 0)
        clV1 = cl;
      else
        clV2 = cl;
    }
  }
  t = tPower = 0;
  v1Closing = v2Closing = false;
  fillV1 = fillV2 = -1;
  for (int i = 0; i < 5; i++)
    p[i] = 0;
  phase = ENG_FILL;
  return true;
}

/* Quick fill then fill-equalise, the same per-element data the shifts use. */
int32_t Engage::FillPressure(int cl, int32_t tIn, int32_t fillTotal,
                             const EngageInputs &in) const {
  const ClutchFillCal &c = calClutch[cl];
  int32_t tsf = Cal::Curve(c.tsfcg, in.temp);
  if (tIn < tsf && tIn < fillTotal)
    return Cal::Map(c.psfcgmt, in.temp, in.m_t);
  return Cal::Curve(c.pfcg, in.temp);
}

void Engage::Tick(const EngageInputs &in) {
  if (!Active())
    return;
  const EngageCal &e = calEngage[idx];

  if (fillV1 < 0) {
    // Fill times are taken once, at the start.
    fillV1 = Cal::Curve(e.tfv1, in.n_ab);
    fillV2 = Cal::Curve(e.tfcgv2, in.temp);
    // A zero fill time (KL_TFV1 at speed) still needs the quick fill.
    int32_t q1 = Cal::Curve(calClutch[clV1].tsfcg, in.temp);
    int32_t q2 = Cal::Curve(calClutch[clV2].tsfcg, in.temp);
    if (fillV1 < q1)
      fillV1 = q1;
    if (fillV2 < q2)
      fillV2 = q2;
  }

  int32_t slip = in.n_t - Gears::SyncSpeed(gear, in.n_ab);
  bool synced = slip < cfg.syncBand && slip > -cfg.syncBand;
  int32_t g1 = Cal::Curve(e.pgrasynv1, in.m_t);
  int32_t g2 = Cal::Curve(e.pgrasynv2, in.m_t);
  int32_t gp = Cal::Curve(e.pgrasyn, in.m_t);
  if (synced) {
    if (g1 < cfg.syncCloseGrad)
      g1 = cfg.syncCloseGrad;
    if (g2 < cfg.syncCloseGrad)
      g2 = cfg.syncCloseGrad;
    if (gp < cfg.syncCloseGrad)
      gp = cfg.syncCloseGrad;
  }

  // V1 and V2: fill, then close with their own gradients.
  if (!v1Closing) {
    p[clV1] = FillPressure(clV1, t, fillV1, in);
    if (t >= fillV1)
      v1Closing = true;
  } else if (p[clV1] < cfg.pMax) {
    p[clV1] += (g1 * ENG_TICK_MS) / 1000;
  }
  if (!v2Closing) {
    p[clV2] = FillPressure(clV2, t, fillV2, in);
    if (t >= fillV2)
      v2Closing = true;
  } else if (p[clV2] < cfg.pMax) {
    p[clV2] += (g2 * ENG_TICK_MS) / 1000;
  }

  // Power-flow element starts once both others are closing.
  int32_t delay = Cal::Curve(e.tvkzu, in.n_ab);
  int32_t tsf = Cal::Curve(calClutch[clPower].tsfcg, in.temp);
  switch (phase) {
  case ENG_FILL:
    p[clPower] = 0;
    if (v1Closing && v2Closing) {
      phase = ENG_POWER;
      tPower = 0;
    }
    break;
  case ENG_POWER:
    if (tPower < delay) {
      p[clPower] = 0;
    } else if (tPower < delay + tsf + cfg.fillEqMs) {
      p[clPower] = FillPressure(clPower, tPower - delay, tsf + cfg.fillEqMs, in);
      pStart = p[clPower];
    } else {
      int32_t tr = tPower - delay - tsf - cfg.fillEqMs;
      int32_t target = Cal::Map(e.pkzu, in.n_t, in.m_t);
      p[clPower] = cfg.rampMs > 0 && tr < cfg.rampMs
                       ? pStart + ((target - pStart) * tr) / cfg.rampMs
                       : target;
      if (tr >= cfg.rampMs)
        phase = ENG_CLOSE;
    }
    tPower += ENG_TICK_MS;
    break;
  case ENG_CLOSE:
    p[clPower] += (gp * ENG_TICK_MS) / 1000;
    if (p[clPower] >= cfg.pMax && p[clV1] >= cfg.pMax && p[clV2] >= cfg.pMax)
      phase = ENG_DONE;
    break;
  default:
    break;
  }

  for (int i = 0; i < 5; i++) {
    if (p[i] > cfg.pMax)
      p[i] = cfg.pMax;
    if (p[i] < 0)
      p[i] = 0;
  }
  t += ENG_TICK_MS;
}
