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
#include "sas.h"
#include "cal_330e.h"
#include "caltypes.h"

#define SAS_TICK_MS 10

/* Gradient (mbar/s) times a duration (ms), in mbar. */
static int32_t GradMs(int32_t mbarPerS, int32_t ms) {
  return (int32_t)(((int64_t)mbarPerS * ms) / 1000);
}

/* Linear ramp from a to b, t of total ms. */
static int32_t Ramp(int32_t a, int32_t b, int32_t t, int32_t total) {
  if (total <= 0 || t >= total)
    return b;
  if (t <= 0)
    return a;
  return a + (int32_t)(((int64_t)(b - a) * t) / total);
}

static int32_t Clamp(int32_t v, int32_t lo, int32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static int32_t Abs(int32_t v) { return v < 0 ? -v : v; }

/* OEM "time before sync" cals are stored as negative ms ("-30 = 30 ms
 * before sync"). Return true once the predicted time to sync is at or
 * below that lead time. A missing prediction never triggers.
 */
static bool BeforeSync(int32_t tSyn, int32_t calNegMs) {
  int32_t lead = calNegMs < 0 ? -calNegMs : calNegMs;
  return tSyn != SAS_TSYN_NONE && tSyn <= lead;
}

SasShift::SasShift()
    : cfg(), out(), active(false), up(false), from(0), to(0), offCl(0),
      onCl(0), pair(0), tShift(0), tKzu(0), tKab(0), fillStart(0),
      tFillEnd(0), tKabRampEnd(0), pKzuStart(0), pKabStart(0), pKzuSync(0),
      pKabBase(0), glsAcc(0), glsTarget(0), glsDir(0), tGls(0), glsOn(false),
      glsOnKab(false), coast(false), syncPhase(0) {}

bool SasShift::Start(int f, int t, int offClutch, int onClutch,
                     const SasConfig &c, const SasInputs &in) {
  if (f < 1 || f > 8 || t < 1 || t > 8)
    return false;
  if (t - f != 1 && f - t != 1)
    return false;
  if (offClutch < 0 || offClutch > 4 || onClutch < 0 || onClutch > 4)
    return false;

  cfg = c;
  from = f;
  to = t;
  offCl = offClutch;
  onCl = onClutch;
  up = t > f;
  pair = up ? f - 1 : t - 1; // 1->2 and 2->1 are both index 0
  tShift = tKzu = tKab = 0;
  fillStart = 0;
  pKabBase = 0;
  glsAcc = 0;
  glsTarget = 0;
  glsDir = 0;
  tGls = 0;
  glsOn = false;
  syncPhase = 0;
  // zug_schub_sas: coast when the input torque is negative at the start.
  coast = in.m_t < 0;
  // The slip controller sits on KZU for power-on upshifts and coast
  // downshifts, and on KAB for coast upshifts and power-on downshifts.
  glsOnKab = up ? coast : !coast;
  out = SasOutputs();
  active = true;

  if (up) {
    const UpShiftCal &u = calUp[pair];
    out.pSysMin = u.p_sysmin;
    // KAB times are referenced to the end of the KZU fill, which is
    // T_VKZU + total fill time. Work it out once at the start.
    tFillEnd = u.t_vkzu + Cal::Map(u.tfcgnt, in.temp, in.n_t);
    EnterKzu(KZU_WAIT);
    EnterKab(KAB_DELAY);
  } else {
    out.pSysMin = calDown[pair].p_sysmin;
    tFillEnd = 0;
    EnterKzu(KZU_WAIT);
    EnterKab(KAB_HOLD_DOWN);
  }
  out.pKab = cfg.pMax; // the off-going clutch is fully applied right now
  Tick(in);
  return true;
}

void SasShift::Abort() {
  active = false;
  out.zKzu = KZU_IDLE;
  out.zKab = KAB_IDLE;
  out.inertia = false;
  out.tqDelta = 0;
}

void SasShift::EnterKzu(uint8_t s) {
  out.zKzu = s;
  tKzu = 0;
  pKzuStart = out.pKzu;
}

void SasShift::EnterKab(uint8_t s) {
  out.zKab = s;
  tKab = 0;
  pKabStart = out.pKab;
}

/* GLS ("Gradientenlageregler"): PI on the input speed gradient so the slip
 * lasts the target time KF_WTSGLSxy. Scaling, from Ian's QA2 answers:
 *   KP: 1 count = 0.02 mbar per (rpm/s)
 *   KI: 1 count = 0.02/64 mbar per (rpm/s), added every 10 ms
 *   output = (P + I) x 50/64, clamped to +/-3 bar (P_MAXOBGLS/P_MAXUNGLS)
 * Everything is kept in 1/3200 mbar (= 0.02/64 mbar) so it stays integer:
 * P = KP x err x 64, I = sum of KI x err, and the scaled output in mbar is
 * (P + I) x 50 / (64 x 3200) = (P + I) / 4096.
 *
 * The error is positive when the input speed moves more slowly than wanted
 * (towards the new sync). The return value is the extra "push" in mbar: the
 * caller adds it to an on-coming clutch and subtracts it from an off-going
 * one (less off-going pressure lets the speed move faster).
 */
void SasShift::GlsArm(const CalMap &wtsgls, const SasInputs &in) {
  // Target gradient: cover the present slip in the target slip time.
  int32_t tw = Cal::Map(wtsgls, in.n_t, in.m_t);
  glsTarget = tw > 0 ? -(int32_t)(((int64_t)in.ndSyn * 1000) / tw) : 0;
  glsDir = in.ndSyn > 0 ? -1 : 1;
  glsAcc = 0;
  glsOn = false;
  tGls = 0;
}

#define GLS_SHIFT 4096 // (64 x 3200) / 50

int32_t SasShift::GlsScale(int32_t kp, int32_t err, int64_t acc) {
  return (int32_t)(((int64_t)kp * err * 64 + acc) / GLS_SHIFT);
}

int32_t SasShift::GlsStep(const SasInputs &in, int32_t kp, int32_t ki) {
  if (cfg.glsGainPct <= 0)
    return 0;
  int32_t err = (glsTarget - in.dnt) * glsDir; // rpm/s, + = too slow
  int32_t hi = calGlobal.p_maxobgls, lo = -calGlobal.p_maxungls;
  // Anti-windup: the integrator alone may not exceed the output clamp.
  int64_t accHi = (int64_t)hi * GLS_SHIFT, accLo = (int64_t)lo * GLS_SHIFT;
  int64_t acc = (int64_t)glsAcc + (int64_t)ki * err;
  acc = acc > accHi ? accHi : (acc < accLo ? accLo : acc);
  glsAcc = (int32_t)acc;
  int32_t p = GlsScale(kp, err, acc);
  p = (int32_t)(((int64_t)p * cfg.glsGainPct) / 100);
  return Clamp(p, lo, hi);
}

/* Run the GLS for the clutch that owns it this shift. "start" lets the
 * caller begin early (slip threshold) before the start delay has passed.
 * Returns 0 until it has started.
 */
int32_t SasShift::GlsRun(const SasInputs &in, bool start, int32_t kp, int32_t ki) {
  if (start)
    glsOn = true;
  tGls += SAS_TICK_MS;
  out.gls = glsOn ? GlsStep(in, kp, ki) : 0;
  return out.gls;
}

void SasShift::TickUpKzu(const SasInputs &in) {
  const UpShiftCal &u = calUp[pair];
  const ClutchFillCal &cl = calClutch[onCl];
  int32_t p = out.pKzu;

  switch (out.zKzu) {
  case KZU_WAIT:
    p = 0;
    if (tShift >= u.t_vkzu) {
      fillStart = tShift;
      EnterKzu(KZU_QUICK_FILL);
    }
    break;
  case KZU_QUICK_FILL:
    p = Cal::Map(cl.psfcgmt, in.temp, in.m_t);
    if (tKzu >= Cal::Curve(cl.tsfcg, in.temp) + Cal::Map(cl.tosfhs, in.n_t, in.m_t))
      EnterKzu(KZU_FILL_EQ_UP);
    break;
  case KZU_FILL_EQ_UP:
    p = Cal::Curve(cl.pfcg, in.temp) + Cal::Map(u.pofmtn, in.n_t, in.m_t);
    if (tShift - fillStart >= Cal::Map(u.tfcgnt, in.temp, in.n_t)) {
      out.pKzu = p;
      EnterKzu(KZU_RAMP1);
    }
    break;
  case KZU_RAMP1: {
    int32_t target = Cal::Map(u.pkzu, in.n_t, in.m_t);
    int32_t len = Cal::Map(u.tramtnt, in.m_t, in.n_t);
    p = Ramp(pKzuStart, target, tKzu, len);
    if (tKzu >= len) {
      out.pKzu = p;
      EnterKzu(KZU_RAMP2);
      if (!glsOnKab)
        GlsArm(u.wtsgls, in);
    }
    break;
  }
  case KZU_RAMP2: {
    // On-coming gradient KF_PGOKZUMTNT. Power-on: the GLS on KZU starts
    // KL_TVGLSKZUCG (180-200 ms) after the start of ramp 2 (Ian, QA2).
    p = pKzuStart + GradMs(Cal::Map(u.pgokzumtnt, in.m_t, in.n_t), tKzu);
    if (!glsOnKab)
      p += GlsRun(in, tGls >= Cal::Curve(u.tvglskzucg, in.temp), u.kp_glskzu,
                  u.ki_glskzu);
    if (BeforeSync(in.tSyn, Cal::Curve(u.tsynmtrakzu, in.m_t)) ||
        Abs(in.ndSyn) < cfg.syncBand) {
      out.pKzu = p;
      EnterKzu(KZU_RAMP3);
    }
    break;
  }
  case KZU_RAMP3:
    // No third ramp (Ian, QA2): until sync the on-coming clutch keeps the
    // ramp-2 gradient KF_PGOKZUMTNT. After sync it uses KL_PGRASYN for
    // T_SYN (300 ms on 1->2), then ramps to full pressure over T_RAMAX.
    if (syncPhase == 0) {
      p += GradMs(Cal::Map(u.pgokzumtnt, in.m_t, in.n_t), SAS_TICK_MS);
      if (Abs(in.ndSyn) < cfg.syncBand) {
        syncPhase = 1;
        tKzu = 0;
      }
    } else if (syncPhase == 1) {
      p += GradMs(Cal::Curve(u.pgrasyn, in.m_t), SAS_TICK_MS);
      if (tKzu + SAS_TICK_MS >= u.t_syn) {
        syncPhase = 2;
        pKzuSync = p;
        tKzu = -SAS_TICK_MS; // the ramp starts on the next tick
      }
    } else {
      p = Ramp(pKzuSync, cfg.pMax, tKzu, u.t_ramax);
      if (tKzu >= u.t_ramax) {
        p = cfg.pMax;
        out.zKzu = KZU_IDLE; // finished
      }
    }
    break;
  case KZU_TIMEOUT:
    p = Ramp(pKzuStart, cfg.pMax, tKzu, u.t_ramax);
    if (tKzu >= u.t_ramax) {
      p = cfg.pMax;
      out.zKzu = KZU_IDLE;
    }
    break;
  default:
    break;
  }
  out.pKzu = Clamp(p, 0, cfg.pMax);
}

void SasShift::TickUpKab(const SasInputs &in) {
  const UpShiftCal &u = calUp[pair];
  int32_t p = out.pKab;

  switch (out.zKab) {
  case KAB_DELAY:
    p = cfg.pMax;
    if (tShift >= u.t_vkabhs)
      EnterKab(KAB_HOLD_UP);
    break;
  case KAB_HOLD_UP:
    p = Cal::Curve(u.phkabmt, in.m_t) + (tKab < u.to_hkab ? u.po_hkab : 0);
    // KF_THKABMTCG is negative: the hold ends that long before KZU fill end.
    if (tShift >= tFillEnd + Cal::Map(u.thkabmtcg, in.m_t, in.temp)) {
      out.pKab = p;
      tKabRampEnd = tFillEnd + Cal::Map(u.tpkabmtcg, in.m_t, in.temp) +
                    Cal::Curve(u.topkabnd, Abs(in.ndSynIst));
      if (tKabRampEnd < tShift)
        tKabRampEnd = tShift;
      EnterKab(KAB_RAMP_UP);
    }
    break;
  case KAB_RAMP_UP:
    p = Ramp(pKabStart, Cal::Map(u.pkab, in.n_t, in.m_t), tKab,
             tKabRampEnd - (tShift - tKab));
    if (tShift >= tKabRampEnd) {
      out.pKab = p;
      EnterKab(KAB_SHIFT_P_UP);
      if (glsOnKab)
        GlsArm(u.wtsgls, in);
    }
    break;
  case KAB_SHIFT_P_UP:
    // Off-going gradient KL_PGOMTKAB. This is the only KAB gradient on an
    // upshift; the takeover state below just carries it on.
    pKabBase = Cal::Map(u.pkab, in.n_t, in.m_t) +
               GradMs(Cal::Curve(u.pgomtkab, in.m_t), tKab);
    p = pKabBase;
    // Coast: the GLS sits on KAB, starting KL_TVGLSKABCG (250/150/50 ms)
    // into this phase, or earlier once the slip reaches 350 rpm (Ian, QA2).
    if (glsOnKab)
      p -= GlsRun(in, tGls >= Cal::Curve(u.tvglskabcg, in.temp) ||
                          Abs(in.ndSynIst) >= cfg.glsKabSlipUp,
                  u.kp_glskab, u.ki_glskab);
    // Inertia phase starts once the input speed has left the old gear's
    // sync by NS_SYNISTE (350 rpm), or the on-coming clutch is closing.
    if (Abs(in.ndSynIst) > u.ns_synist || out.zKzu == KZU_RAMP3 ||
        out.zKzu == KZU_IDLE) {
      out.pKab = p;
      EnterKab(KAB_TAKEOVER);
      pKabStart = pKabBase;
    }
    break;
  case KAB_TAKEOVER:
    // Same KL_PGOMTKAB gradient, carried on from the shift phase.
    pKabBase = pKabStart + GradMs(Cal::Curve(u.pgomtkab, in.m_t), tKab);
    p = pKabBase;
    if (glsOnKab)
      p -= GlsRun(in, tGls >= Cal::Curve(u.tvglskabcg, in.temp) ||
                          Abs(in.ndSynIst) >= cfg.glsKabSlipUp,
                  u.kp_glskab, u.ki_glskab);
    if (BeforeSync(in.tSyn, u.t_synkabe) || out.zKzu == KZU_RAMP3 ||
        out.zKzu == KZU_IDLE)
      EnterKab(KAB_END);
    break;
  case KAB_END:
    p = 0;
    break;
  default:
    break;
  }
  out.pKab = Clamp(p, 0, cfg.pMax);
}

void SasShift::TickDownKab(const SasInputs &in) {
  const DownShiftCal &d = calDown[pair];
  int32_t p = out.pKab;
  int32_t pShift = Cal::Map(d.pkab, in.n_t, in.m_t);

  switch (out.zKab) {
  case KAB_HOLD_DOWN:
    // Hold = KF_PKAB + KL_POHKAB (0.4-1.0 bar) for the KAB delay
    // KF_TVKABMTNMOTRS, then a KL_TRAHKAB (100 ms) ramp down to KF_PKAB.
    // Confirmed by Ian (QA2). Upshifts use the absolute KL_PHKABMT instead.
    p = pShift + Cal::Curve(d.pohkab, in.m_t);
    if (tKab >= Cal::Map(d.tvkabmtnmotrs, in.m_t, in.n_mot)) {
      out.pKab = p;
      EnterKab(KAB_RAMP_DOWN);
    }
    break;
  case KAB_RAMP_DOWN: {
    int32_t len = Cal::Curve(d.trahkab, in.m_t);
    p = Ramp(pKabStart, pShift, tKab, len);
    if (tKab >= len) {
      out.pKab = p;
      EnterKab(KAB_SLIP_DOWN);
      if (glsOnKab)
        GlsArm(d.wtsgls, in);
    }
    break;
  }
  case KAB_SLIP_DOWN:
    // Controlled slip: the input speed rises towards the new sync.
    p = pShift + GradMs(Cal::Curve(d.pgokab, in.m_t), tKab);
    // Power-on: the GLS sits on KAB, starting KL_TVGLSKABCG into this
    // phase, or earlier once the slip reaches 150 rpm (Ian, QA2).
    if (glsOnKab)
      p -= GlsRun(in, tGls >= Cal::Curve(d.tvglskabcg, in.temp) ||
                          Abs(in.ndSynIst) >= cfg.glsKabSlipDown,
                  d.kp_glskab, d.ki_glskab);
    out.inertia = true;
    if (BeforeSync(in.tSyn, Cal::Map(d.tsyncgmthinsyn, in.temp, in.m_t)) ||
        Abs(in.ndSyn) < cfg.syncBand) {
      out.pKab = p;
      EnterKab(KAB_RAMP_SYNC);
    }
    break;
  case KAB_RAMP_SYNC:
    p = pShift + Cal::Map(d.pohinsynkab, in.n_t, in.m_t);
    EnterKab(KAB_HOLD_SYNC);
    break;
  case KAB_HOLD_SYNC:
    p = pShift + Cal::Map(d.pohinsynkab, in.n_t, in.m_t);
    if (tKab >= d.t_hinsyn || out.zKzu == KZU_CLOSE_DOWN) {
      out.pKab = p;
      EnterKab(KAB_OVERLAP);
    }
    break;
  case KAB_OVERLAP:
    p = pKabStart + GradMs(Cal::Curve(d.pgrasynkab, in.m_t), tKab);
    if (p <= 0 || tKab >= Cal::Curve(d.tramthinsynkab, in.m_t))
      EnterKab(KAB_END);
    break;
  case KAB_END:
    p = 0;
    break;
  default:
    break;
  }
  out.pKab = Clamp(p, 0, cfg.pMax);
}

void SasShift::TickDownKzu(const SasInputs &in) {
  const DownShiftCal &d = calDown[pair];
  const ClutchFillCal &cl = calClutch[onCl];
  int32_t p = out.pKzu;

  switch (out.zKzu) {
  case KZU_WAIT:
    p = 0;
    if (tShift >= d.t_vkzu)
      EnterKzu(KZU_QUICK_FILL);
    break;
  case KZU_QUICK_FILL:
    p = Cal::Map(cl.psfcgmt, in.temp, in.m_t);
    if (tKzu >= Cal::Curve(cl.tsfcg, in.temp) + Cal::Map(cl.tosfrs, in.n_t, in.m_t))
      EnterKzu(KZU_FILL_EQ_DOWN);
    break;
  case KZU_FILL_EQ_DOWN:
    p = Cal::Curve(cl.pfcg, in.temp);
    if (tKzu >= d.t_f) {
      out.pKzu = p;
      EnterKzu(KZU_RAMP1);
    }
    break;
  case KZU_RAMP1: {
    int32_t len = Cal::Curve(d.tra, in.m_t);
    p = Ramp(pKzuStart, Cal::Map(d.pkzu, in.n_t, in.m_t), tKzu, len);
    if (tKzu >= len) {
      out.pKzu = p;
      EnterKzu(KZU_HOLD_SYNC);
      if (!glsOnKab)
        GlsArm(d.wtsgls, in);
    }
    break;
  }
  case KZU_HOLD_SYNC:
    p = Cal::Map(d.pkzu, in.n_t, in.m_t) + Cal::Map(d.pohinsynkzu, in.n_t, in.m_t);
    // Coast: the GLS sits on KZU, starting KL_TVGLSKZUCG into this phase.
    if (!glsOnKab)
      p += GlsRun(in, tGls >= Cal::Curve(d.tvglskzucg, in.temp), d.kp_glskzu,
                  d.ki_glskzu);
    if (BeforeSync(in.tSyn, Cal::Curve(d.tsyncghinsynekzu, in.temp)) ||
        Abs(in.ndSyn) < cfg.syncBand) {
      out.pKzu = p;
      EnterKzu(KZU_CLOSE_DOWN);
    }
    break;
  case KZU_CLOSE_DOWN:
    p += GradMs(Cal::Curve(d.pgrasyn, in.m_t), SAS_TICK_MS);
    if (p >= cfg.pMax) {
      p = cfg.pMax;
      out.zKzu = KZU_IDLE;
    }
    break;
  case KZU_TIMEOUT:
    p = Ramp(pKzuStart, cfg.pMax, tKzu, cfg.rampMaxDownMs);
    if (tKzu >= cfg.rampMaxDownMs) {
      p = cfg.pMax;
      out.zKzu = KZU_IDLE;
    }
    break;
  default:
    break;
  }
  out.pKzu = Clamp(p, 0, cfg.pMax);
}

void SasShift::Tick(const SasInputs &in) {
  if (!active)
    return;
  out.gls = 0; // set again below by whichever phase runs the GLS

  // Timeout: shift taking longer than KL_TMAXCG -> close the on-coming
  // clutch to max and vent the off-going one.
  int32_t tMax = up ? Cal::Curve(calUp[pair].tmaxcg, in.temp)
                    : Cal::Curve(calDown[pair].tmaxcg, in.temp);
  if (!out.timedOut && tShift > tMax && out.zKzu != KZU_IDLE) {
    out.timedOut = true;
    EnterKzu(KZU_TIMEOUT);
    EnterKab(KAB_END);
  }

  if (up) {
    TickUpKzu(in);
    TickUpKab(in);
    // Inertia phase: from ramp 2 until the input speed reaches the new sync.
    out.inertia = out.zKzu == KZU_RAMP2 ||
                  (out.zKzu == KZU_RAMP3 && syncPhase == 0) ||
                  out.zKab == KAB_TAKEOVER;
    // Torque intervention during the inertia phase (KF_MEHAUPTxy).
    out.tqDelta = out.inertia ? Cal::Map(calUp[pair].mehaupt, in.n_t, in.m_t) : 0;
  } else {
    TickDownKab(in);
    TickDownKzu(in);
    if (out.zKab != KAB_SLIP_DOWN)
      out.inertia = false;
    // Downshift intervention after sync (KF_MENACHxy, torque reduction).
    out.tqDelta = (out.zKzu == KZU_CLOSE_DOWN)
                      ? Cal::Map(calDown[pair].menach, in.n_t, in.m_t)
                      : 0;
  }

  if (out.zKzu == KZU_IDLE && (out.zKab == KAB_END || out.zKab == KAB_IDLE)) {
    out.done = true;
    out.inertia = false;
    out.tqDelta = 0;
    active = false;
  }

  tShift += SAS_TICK_MS;
  tKzu += SAS_TICK_MS;
  tKab += SAS_TICK_MS;
}
