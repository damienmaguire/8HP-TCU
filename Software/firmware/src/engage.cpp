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
#include "calset.h"
#include "caltypes.h"
#include "gears.h"

#define ENG_TICK_MS 10

/* Gear-in roles per variant, from the byte tables Ian read out (QA2
 * answer 1) [C]. Each entry is {last (power-flow), V1, V2} for 1st and R.
 *   330e: power-flow 0x4C214, V1 0x4C21E, V2 0x4C228
 *   F11:  power-flow 0x5BA08, V1 0x5BA12, V2 0x5BA1C
 */
static const int8_t roles[2][2][3] = {
    // 1st                 R
    {{CL_B, CL_A, CL_C}, {CL_B, CL_A, CL_D}}, // ENG_VAR_330E
    {{CL_C, CL_A, CL_B}, {CL_D, CL_A, CL_B}}, // ENG_VAR_F11
};

static const int8_t *Roles(int g, int variant) {
  if ((g != GEAR_1 && g != GEAR_R) || variant < 0 || variant > 1)
    return 0;
  return roles[variant][g == GEAR_1 ? 0 : 1];
}

int Engage::PowerFlowClutch(int g, int variant) {
  const int8_t *r = Roles(g, variant);
  return r ? r[0] : -1;
}

Engage::Engage()
    : cfg(), phase(ENG_IDLE), gear(0), idx(0), clV1(CL_A), clV2(CL_B),
      clPower(-1), t(0), tPower(0), tNoSlip(0), tAtMax(0), tMaxRamp(-1),
      pSync(0), tvV2(-1), fillV1(0),
      fillV2(0), pStart(0), v1Closing(false), v2Closing(false), fast(false) {
  for (int i = 0; i < 5; i++)
    p[i] = 0;
}

bool Engage::Start(int g, const EngageConfig &c) {
  const int8_t *r = Roles(g, c.variant);
  if (!r)
    return false;
  cfg = c;
  gear = g;
  idx = g == GEAR_1 ? 0 : 1;
  clPower = r[0];
  clV1 = r[1];
  clV2 = r[2];
  t = tPower = tNoSlip = tAtMax = 0;
  tMaxRamp = -1;
  pSync = 0;
  tvV2 = -1;
  v1Closing = v2Closing = false;
  fast = false;
  fillV1 = fillV2 = -1;
  for (int i = 0; i < 5; i++)
    p[i] = 0;
  phase = ENG_FILL;
  return true;
}

/* Quick fill then fill-equalise, the same per-element data the shifts use. */
int32_t Engage::FillPressure(int cl, int32_t tIn, int32_t fillTotal,
                             const EngageInputs &in) const {
  const ClutchFillCal &c = Cal::Active().clutch[cl];
  int32_t tsf = Cal::Curve(c.tsfcg, in.temp);
  if (tIn < tsf && tIn < fillTotal)
    return Cal::Map(c.psfcgmt, in.temp, in.m_t);
  return Cal::Curve(c.pfcg, in.temp);
}

void Engage::Tick(const EngageInputs &in) {
  if (!Active())
    return;
  const EngageCal &e = Cal::Active().engage[idx];

  if (fillV1 < 0) {
    // Fill times are taken once, at the start.
    fillV1 = Cal::Curve(e.tfv1, in.n_ab);
    fillV2 = Cal::Curve(e.tfcgv2, in.temp);
    // KL_TVKZUCGV2_n "Verzoegerungszeit 2. nichtkraftschliessende KZU":
    // V2 starts to fill this long after V1 (300 -> 50 ms over temperature).
    // The V2 fill time is counted from the end of this delay [I].
    tvV2 = Cal::Curve(e.tvkzucgv2, in.temp);
    // A zero fill time (KL_TFV1 at speed) still needs the quick fill.
    int32_t q1 = Cal::Curve(Cal::Active().clutch[clV1].tsfcg, in.temp);
    int32_t q2 = Cal::Curve(Cal::Active().clutch[clV2].tsfcg, in.temp);
    if (fillV1 < q1)
      fillV1 = q1;
    if (fillV2 < q2)
      fillV2 = q2;
  }

  int32_t slip = in.n_t - Gears::SyncSpeed(gear, in.n_ab);
  bool synced = in.speedsValid && slip < cfg.syncBand && slip > -cfg.syncBand;
  tNoSlip = synced ? tNoSlip + ENG_TICK_MS : 0;
  int32_t tq = in.m_t < 0 ? -in.m_t : in.m_t;
  // Fast close only with confirmed no-slip, torque held near 0 and enough
  // line pressure. It drops back to the OEM gradients if any of those go.
  fast = tNoSlip >= cfg.syncConfirmMs && tq <= cfg.tqZeroNm && in.lineOk &&
         cfg.fastCloseGrad > 0;
  int32_t g1 = Cal::Curve(e.pgrasynv1, in.m_t);
  int32_t g2 = Cal::Curve(e.pgrasynv2, in.m_t);
  int32_t gp = Cal::Curve(e.pgrasyn, in.m_t);
  if (fast)
    g1 = g2 = gp = cfg.fastCloseGrad; // rate limit for the fast close

  // V1 and V2: fill, then close with their own gradients.
  if (!v1Closing) {
    p[clV1] = FillPressure(clV1, t, fillV1, in);
    if (t >= fillV1)
      v1Closing = true;
  } else if (p[clV1] < cfg.pMax) {
    p[clV1] += (g1 * ENG_TICK_MS) / 1000;
  }
  if (!v2Closing) {
    int32_t t2 = t - tvV2;
    p[clV2] = t2 < 0 ? 0 : FillPressure(clV2, t2, fillV2, in);
    if (t2 >= fillV2)
      v2Closing = true;
  } else if (p[clV2] < cfg.pMax) {
    p[clV2] += (g2 * ENG_TICK_MS) / 1000;
  }

  // Power-flow element starts once both others are closing.
  int32_t delay = Cal::Curve(e.tvkzu, in.n_ab);
  int32_t tsf = Cal::Curve(Cal::Active().clutch[clPower].tsfcg, in.temp);
  // B is kept at fill pressure in neutral (OEM). When B is the last clutch
  // in (330e) it stays there while V1/V2 fill instead of being dumped.
  int32_t preFill = clPower == CL_B ? Cal::Curve(Cal::Active().clutch[CL_B].pfcg, in.temp) : 0;

  switch (phase) {
  case ENG_FILL:
    p[clPower] = preFill;
    if (v1Closing && v2Closing) {
      phase = ENG_POWER;
      tPower = 0;
    }
    break;
  case ENG_POWER:
    if (tPower < delay) {
      p[clPower] = preFill;
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
  case ENG_CLOSE: {
    // OEM (QA2 answers 1 and 6) [C values]: close with KL_PGRASYN_n
    // (1-15 bar/s over torque) until sync, stay in sync for T_SYN_n
    // (200 ms, "Verweilzeit in Synchron zum Start Max.Rampe"), then run
    // KL_PORASYN_n (+0.5 bar by 950 ms, PMAX at 1500 ms on N->1).
    // KL_PORASYN_n is read as an offset on the pressure at its start with
    // the last point meaning PMAX [I]. The fast close replaces all of this
    // with one rate-limited gradient while its conditions hold.
    const CalCurve &r = e.porasyn;
    if (fast) {
      p[clPower] += (gp * ENG_TICK_MS) / 1000;
    } else if (tMaxRamp >= 0) {
      p[clPower] = tMaxRamp >= r.x[r.n - 1] ? cfg.pMax
                                            : pSync + Cal::Curve(r, tMaxRamp);
      tMaxRamp += ENG_TICK_MS;
    } else {
      p[clPower] += (gp * ENG_TICK_MS) / 1000;
      // Start the max ramp after T_SYN_n in sync. Without working speed
      // sensors sync cannot be seen, so start it T_SYN_n after this phase
      // began instead.
      bool dwell = in.speedsValid ? tNoSlip >= e.t_syn : tAtMax >= e.t_syn;
      if (!in.speedsValid)
        tAtMax += ENG_TICK_MS;
      if (dwell) {
        tMaxRamp = 0;
        pSync = p[clPower];
      }
    }
    if (p[clPower] >= cfg.pMax && p[clV1] >= cfg.pMax && p[clV2] >= cfg.pMax &&
        (fast || tMaxRamp > r.x[r.n - 1]))
      phase = ENG_DONE;
    break;
  }
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
