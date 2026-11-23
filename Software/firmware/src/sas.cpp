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
#include "calset.h"
#include "caltypes.h"

#define SAS_TICK_MS 10 // ms, Tick period

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

// Limit v to lo..hi.
static int32_t Clamp(int32_t v, int32_t lo, int32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Absolute value.
static int32_t Abs(int32_t v) { return v < 0 ? -v : v; }

/* OEM "time before sync" cals are stored as negative ms ("-30 = 30 ms
 * before sync"). Return true once the predicted time to sync is at or
 * below that lead time. A missing prediction never triggers.
 */
static bool BeforeSync(int32_t tSyn, int32_t calNegMs) {
  int32_t lead = calNegMs < 0 ? -calNegMs : calNegMs;
  return tSyn != SAS_TSYN_NONE && tSyn <= lead;
}

/* DriveMode shift time scale (SasConfig::timePct, 100 = cal unchanged).
 * Applied to the fill-equalise time, the ramp lengths and the KAB hold /
 * ramp points that are referenced to the fill end, so the two clutches
 * stay in step. Not scaled: the quick-fill pulse (clutch volume), T_VKZU,
 * sync detection and T_SYN, the GLS delays and the KL_TMAXCG timeout.
 */
static int32_t TScale(const SasConfig &c, int32_t ms) {
  if (c.timePct <= 0 || c.timePct >= 100)
    return ms;
  return (int32_t)(((int64_t)ms * c.timePct) / 100);
}

// Idle until a shift is started.
SasShift::SasShift()
    : cfg(), out(), active(false), up(false), from(0), to(0), offCl(0),
      onCl(0), pair(0), tShift(0), tKzu(0), tKab(0), fillStart(0),
      tFillEnd(0), tKabRampEnd(0), pKzuStart(0), pKabStart(0), pKzuSync(0),
      pKabBase(0), glsAcc(0), glsTarget(0), glsDir(0), tGls(0), glsOn(false),
      glsOnKab(false), glsWaitDone(false), glsPhase(false), glsRa1(false),
      coast(false), syncPhase(0), tSfEnd(0) {}

// Start a one-clutch-swap shift with its calibration set.
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
  glsWaitDone = glsPhase = glsRa1 = false;
  syncPhase = 0;
  // zug_schub_sas: coast when the input torque is negative at the start.
  coast = in.m_t < 0;
  // The slip controller sits on KZU for power-on upshifts and coast
  // downshifts, and on KAB for coast upshifts and power-on downshifts.
  glsOnKab = up ? coast : !coast;
  out = SasOutputs();
  active = true;

  if (up) {
    const UpShiftCal &u = Cal::Up()[pair];
    out.pSysMin = u.p_sysmin;
    // KAB times are referenced to the end of the KZU fill, which is
    // T_VKZU + total fill time. Work it out once at the start.
    tFillEnd = u.t_vkzu + TScale(cfg, Cal::Map(u.tfcgnt, in.temp, in.n_t));
    EnterKzu(KZU_WAIT);
    EnterKab(KAB_DELAY);
  } else {
    const DownShiftCal &d = Cal::Down()[pair];
    const ClutchFillCal &cl = Cal::Clutch()[onCl];
    out.pSysMin = d.p_sysmin;
    tFillEnd = 0;
    // End of the KZU quick fill ("Ende SF"), the reference for the KAB
    // hold phase (KF_TOHKABMTNT, a negative offset).
    tSfEnd = d.t_vkzu + Cal::Curve(cl.tsfcg, in.temp) +
             Cal::Map(cl.tosfrs, in.n_t, in.m_t);
    EnterKzu(KZU_WAIT);
    EnterKab(KAB_DELAY);
  }
  out.pKab = cfg.pMax; // the off-going clutch is fully applied right now
  Tick(in);
  return true;
}

// Stop the shift; both clutch outputs go idle, the caller decides what next.
void SasShift::Abort() {
  active = false;
  out.zKzu = KZU_IDLE;
  out.zKab = KAB_IDLE;
  out.inertia = false;
  out.tqDelta = 0;
}

// On-coming clutch to a new state (timer restarts).
void SasShift::EnterKzu(uint8_t s) {
  out.zKzu = s;
  tKzu = 0;
  pKzuStart = out.pKzu;
}

// Off-going clutch to a new state (timer restarts).
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
}

#define GLS_SHIFT 4096 // (64 x 3200) / 50

// OEM GLS scaling of a gain times an error, mbar.
int32_t SasShift::GlsScale(int32_t kp, int32_t err, int64_t acc) {
  return (int32_t)(((int64_t)kp * err * 64 + acc) / GLS_SHIFT);
}

// One PI step of the GLS speed-change controller.
int32_t SasShift::GlsStep(const SasInputs &in, int32_t kp, int32_t ki) {
  if (cfg.glsGainPct <= 0)
    return 0;
  int32_t err = (glsTarget - in.dnt) * glsDir; // rpm/s, + = too slow
  int32_t hi = Cal::Active().global->p_maxobgls, lo = -Cal::Active().global->p_maxungls;
  // Anti-windup: the integrator alone may not exceed the output clamp.
  int64_t accHi = (int64_t)hi * GLS_SHIFT, accLo = (int64_t)lo * GLS_SHIFT;
  int64_t acc = (int64_t)glsAcc + (int64_t)ki * err;
  acc = acc > accHi ? accHi : (acc < accLo ? accLo : acc);
  glsAcc = (int32_t)acc;
  int32_t p = GlsScale(kp, err, acc);
  p = (int32_t)(((int64_t)p * cfg.glsGainPct) / 100);
  return Clamp(p, lo, hi);
}

/* GLS phase machine, after FUN_0010f12e (z_gls, sequence table @0x58998)
 * in Ian's QA2 answer 3 [C]:
 *
 *   shift type          wait exit (state 1-4)            delay timer from   GLS on
 *   ZUGHOCH  (power up) ramp 2 started                   KZU ramp 2         KZU
 *   SCHUBHOCH (coast up) KAB shift-pressure phase started KAB shift phase    KAB
 *   ZUGRUECK (power dn) KAB shift-pressure phase started KAB shift phase    KAB
 *   SCHUBRUECK (coast dn) KZU "ramp 2" started           KZU slip phase     KZU
 *
 * Each wait state can also be left by the start criterion FUN_0010e60e
 * once KZU ramp 1 has started: on upshifts nd_synistgang >= KL_NDGLSPOSHS
 * (350 rpm) or <= KL_NDGLSNEGHS (-160..-300 rpm, over torque); on
 * downshifts |nd_synistgang| >= ND_GLSRS (150 rpm on 2->1). Then the delay
 * state 6 waits for KL_TVGLSKZUCG (180-200 ms on 1->2) or KL_TVGLSKABCG
 * (250/150/50 ms) on a timer that is held at 0 until the phase in the
 * third column has started [C]. After that the PI runs (state 8).
 *
 * Simplifications [I]: on ZUGHOCH the OEM also needs the main torque
 * intervention (or a follow-up shift) before the wait exit and the timer;
 * that is not required here, so the GLS still works with TqInterv off. On
 * downshifts the OEM "ramp 2" of the KZU is taken to be this code's
 * KZU_HOLD_SYNC phase. The turbine-gradient start criterion
 * (FUN_0010e9a2, KF_NGTGLSHS) is disabled in the cal (-32000) and left out.
 * Returns the GLS correction for this tick, 0 while not running.
 */
int32_t SasShift::GlsUpdate(const SasInputs &in) {
  bool kzuRa1, phase;
  int32_t delay, kp, ki;
  bool crit;
  const CalMap *wts;
  if (up) {
    const UpShiftCal &u = Cal::Up()[pair];
    kzuRa1 = out.zKzu == KZU_RAMP1 || out.zKzu == KZU_RAMP2 || out.zKzu == KZU_RAMP3;
    crit = in.ndSynIst >= Cal::Curve(u.ndglsposhs, in.m_t) ||
           in.ndSynIst <= Cal::Curve(u.ndglsneghs, in.m_t);
    wts = &u.wtsgls;
    if (glsOnKab) {
      phase = out.zKab == KAB_SHIFT_P_UP || out.zKab == KAB_TAKEOVER;
      delay = Cal::Curve(u.tvglskabcg, in.temp);
      kp = u.kp_glskab;
      ki = u.ki_glskab;
    } else {
      phase = out.zKzu == KZU_RAMP2;
      delay = Cal::Curve(u.tvglskzucg, in.temp);
      kp = u.kp_glskzu;
      ki = u.ki_glskzu;
    }
  } else {
    const DownShiftCal &d = Cal::Down()[pair];
    kzuRa1 = out.zKzu == KZU_RAMP1 || out.zKzu == KZU_HOLD_SYNC ||
             out.zKzu == KZU_CLOSE_DOWN;
    crit = Abs(in.ndSynIst) >= d.nd_glsrs;
    wts = &d.wtsgls;
    if (glsOnKab) {
      phase = out.zKab == KAB_SLIP_DOWN;
      delay = Cal::Curve(d.tvglskabcg, in.temp);
      kp = d.kp_glskab;
      ki = d.ki_glskab;
    } else {
      phase = out.zKzu == KZU_HOLD_SYNC;
      delay = Cal::Curve(d.tvglskzucg, in.temp);
      kp = d.kp_glskzu;
      ki = d.ki_glskzu;
    }
  }
  if (kzuRa1)
    glsRa1 = true;
  if (phase)
    glsPhase = true;
  if (!glsWaitDone && (glsPhase || (crit && glsRa1)))
    glsWaitDone = true;
  if (!glsPhase)
    return 0; // delay timer held at 0 until the phase has started
  if (!glsOn) {
    if (glsWaitDone && tGls >= delay) {
      glsOn = true;
      GlsArm(*wts, in);
    } else {
      tGls += SAS_TICK_MS;
      return 0;
    }
  }
  // Runs only while its clutch is in the phase it acts in.
  return phase ? GlsStep(in, kp, ki) : 0;
}

// Upshift, on-coming clutch: wait, quick fill, fill, ramps to full.
void SasShift::TickUpKzu(const SasInputs &in) {
  const UpShiftCal &u = Cal::Up()[pair];
  const ClutchFillCal &cl = Cal::Clutch()[onCl];
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
    p = Cal::Curve(cl.pfcg, in.temp) + Cal::Map(u.pofmtn, in.n_t, in.m_t) + FillOfs();
    if (tShift - fillStart >= TScale(cfg, Cal::Map(u.tfcgnt, in.temp, in.n_t))) {
      out.pKzu = p;
      EnterKzu(KZU_RAMP1);
    }
    break;
  case KZU_RAMP1: {
    int32_t target = Cal::Map(u.pkzu, in.n_t, in.m_t);
    int32_t len = TScale(cfg, Cal::Map(u.tramtnt, in.m_t, in.n_t));
    p = Ramp(pKzuStart, target, tKzu, len);
    if (tKzu >= len) {
      out.pKzu = p;
      EnterKzu(KZU_RAMP2);
    }
    break;
  }
  case KZU_RAMP2: {
    // On-coming gradient KF_PGOKZUMTNT. Power-on: the GLS on KZU starts
    // KL_TVGLSKZUCG (180-200 ms) after the start of ramp 2 (Ian, QA2).
    p = pKzuStart + GradMs(Cal::Map(u.pgokzumtnt, in.m_t, in.n_t), tKzu);
    if (!glsOnKab)
      p += out.gls;
    if (BeforeSync(in.tSyn, Cal::Curve(u.tsynmtrakzu, in.m_t)) ||
        Abs(in.ndSyn) < cfg.syncBand) {
      out.pKzu = p;
      EnterKzu(KZU_RAMP3);
    }
    break;
  }
  case KZU_RAMP3:
    // Ramp 3 starts KL_TSYNMTRAKZU (30-50 ms) before sync. It closes with
    // the "Schliessrampengradient" KL_PGRASYN, stays in sync for T_SYN
    // (300 ms on 1->2, "Verweilzeit in Synchron zum Start Max.Rampe"), then
    // runs KL_PORASYN ("Rampe n.Syn. -> letz. Stuetz. y PMAX", 0.1/0.5 bar
    // at 0/400 ms, PMAX at 450 ms) [C names and values]. KL_PORASYN is read
    // as an offset on the pressure at the start of the ramp, with its last
    // point meaning PMAX [I]. The n_ab-gradient factor KL_FPGRASYN is not
    // applied [I]. There is no separate takeover ramp (QA2 answer 5).
    if (syncPhase == 0) {
      p += GradMs(Cal::Curve(u.pgrasyn, in.m_t), SAS_TICK_MS);
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
      const CalCurve &r = u.porasyn;
      p = pKzuSync + Cal::Curve(r, tKzu);
      if (tKzu >= r.x[r.n - 1]) {
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

// Upshift, off-going clutch: hold, ramp down, open at sync.
void SasShift::TickUpKab(const SasInputs &in) {
  const UpShiftCal &u = Cal::Up()[pair];
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
    if (tShift >= tFillEnd + TScale(cfg, Cal::Map(u.thkabmtcg, in.m_t, in.temp))) {
      out.pKab = p;
      tKabRampEnd = tFillEnd + TScale(cfg, Cal::Map(u.tpkabmtcg, in.m_t, in.temp) +
                                           Cal::Curve(u.topkabnd, Abs(in.ndSynIst)));
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
    }
    break;
  case KAB_SHIFT_P_UP:
    // Off-going gradient KL_PGOMTKAB ("Gradientenoffset fuer
    // Schaltdruckphase bei HS", -5..-40 bar/s). The takeover state below
    // just carries it on. QA2 answer 5: there is no dedicated takeover map
    // [C for its absence from the A2L, found by a name/description search;
    // I for the behaviour, not traced in code]. The A2L calls it a gradient
    // *offset*; the base gradient it is added to is not identified and is
    // taken as 0 here [H].
    pKabBase = Cal::Map(u.pkab, in.n_t, in.m_t) +
               GradMs(Cal::Curve(u.pgomtkab, in.m_t), tKab);
    p = pKabBase;
    // Coast: the GLS sits on KAB (see GlsUpdate).
    if (glsOnKab)
      p -= out.gls;
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
    // Same KL_PGOMTKAB gradient, carried on from the shift phase. OEM KAB
    // state 9 is "Lastuebernahme GLS bei HS" [C name]; on a coast upshift
    // the GLS runs on KAB here (see GlsUpdate) [I].
    pKabBase = pKabStart + GradMs(Cal::Curve(u.pgomtkab, in.m_t), tKab);
    p = pKabBase;
    if (glsOnKab)
      p -= out.gls;
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

// Downshift, off-going clutch: drop to hold, let the speed rise, open.
void SasShift::TickDownKab(const SasInputs &in) {
  const DownShiftCal &d = Cal::Down()[pair];
  int32_t p = out.pKab;
  int32_t pShift = Cal::Map(d.pkab, in.n_t, in.m_t);

  switch (out.zKab) {
  case KAB_DELAY:
    // KF_TVKABMTNMOTRS "Verzoegerungszeit abschaltende Kupplung bei RS":
    // the off-going clutch stays fully applied this long [C name, I use].
    p = cfg.pMax;
    if (tKab >= Cal::Map(d.tvkabmtnmotrs, in.m_t, in.n_mot))
      EnterKab(KAB_HOLD_DOWN);
    break;
  case KAB_HOLD_DOWN:
    // Hold = KF_PKAB + KL_POHKAB (0.4-1.0 bar) [I, QA2 answer 4: the sum is
    // not traced in code; candidate FUN_0010d270 reads KF_PKAB, but whether
    // it works on the KAB or the KZU object is unconfirmed]. The hold ends
    // KF_TOHKABMTNT ("Zeitoffset auf Ende SF fuer Haltephase", <= 0) before
    // the end of the KZU quick fill [I: taken as the OEM t_ph_rs].
    p = pShift + Cal::Curve(d.pohkab, in.m_t);
    if (tShift >= tSfEnd + Cal::Map(d.tohkabmtnt, in.n_t, in.m_t)) {
      out.pKab = p;
      EnterKab(KAB_RAMP_DOWN);
    }
    break;
  case KAB_RAMP_DOWN: {
    // Ramp to the shift pressure over KL_TRAHKAB (100 ms) x F_TRAHKAB, to
    // KF_PKAB x F_PRAHKAB (both factors 1.0 in this cal) [C].
    int32_t len = TScale(cfg, (int32_t)(((int64_t)Cal::Curve(d.trahkab, in.m_t) * d.f_trahkab) / 100));
    int32_t pEnd = (int32_t)(((int64_t)pShift * d.f_prahkab) / 100);
    p = Ramp(pKabStart, pEnd, tKab, len);
    if (tKab >= len) {
      out.pKab = p;
      EnterKab(KAB_SLIP_DOWN);
    }
    break;
  }
  case KAB_SLIP_DOWN:
    // Controlled slip: the input speed rises towards the new sync.
    p = pShift + GradMs(Cal::Curve(d.pgokab, in.m_t), tKab);
    // Power-on: the GLS sits on KAB (see GlsUpdate).
    if (glsOnKab)
      p -= out.gls;
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

// Downshift, on-coming clutch: fill, wait for sync, close.
void SasShift::TickDownKzu(const SasInputs &in) {
  const DownShiftCal &d = Cal::Down()[pair];
  const ClutchFillCal &cl = Cal::Clutch()[onCl];
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
    p = Cal::Curve(cl.pfcg, in.temp) + FillOfs();
    if (tKzu >= d.t_f) {
      out.pKzu = p;
      EnterKzu(KZU_RAMP1);
    }
    break;
  case KZU_RAMP1: {
    int32_t len = TScale(cfg, Cal::Curve(d.tra, in.m_t));
    p = Ramp(pKzuStart, Cal::Map(d.pkzu, in.n_t, in.m_t), tKzu, len);
    if (tKzu >= len) {
      out.pKzu = p;
      EnterKzu(KZU_HOLD_SYNC);
    }
    break;
  }
  case KZU_HOLD_SYNC:
    p = Cal::Map(d.pkzu, in.n_t, in.m_t) + Cal::Map(d.pohinsynkzu, in.n_t, in.m_t);
    // Coast: the GLS sits on KZU (see GlsUpdate).
    if (!glsOnKab)
      p += out.gls;
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

// Run both clutch state machines one 10 ms step.
void SasShift::Tick(const SasInputs &in) {
  if (!active)
    return;
  // GLS first, from the states at the start of this tick; the phase that
  // owns it adds (KZU) or subtracts (KAB) the result below.
  out.gls = GlsUpdate(in);

  // Timeout: shift taking longer than KL_TMAXCG -> close the on-coming
  // clutch to max and vent the off-going one.
  int32_t tMax = up ? Cal::Curve(Cal::Up()[pair].tmaxcg, in.temp)
                    : Cal::Curve(Cal::Down()[pair].tmaxcg, in.temp);
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
    out.tqDelta = out.inertia ? Cal::Map(Cal::Up()[pair].mehaupt, in.n_t, in.m_t) : 0;
  } else {
    TickDownKab(in);
    TickDownKzu(in);
    if (out.zKab != KAB_SLIP_DOWN)
      out.inertia = false;
    // Downshift intervention after sync (KF_MENACHxy, torque reduction).
    out.tqDelta = (out.zKzu == KZU_CLOSE_DOWN)
                      ? Cal::Map(Cal::Down()[pair].menach, in.n_t, in.m_t)
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
