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
#include "tcu.h"
#include "cal_330e.h"
#include "caltypes.h"

#define TCU_TICK_MS 10

static int32_t Abs(int32_t v) { return v < 0 ? -v : v; }

/* First-order low pass, k = tau / tick + 1. Steps smaller than one count
 * jump straight to the input so the output settles exactly. */
static int32_t Pt1(int32_t filt, int32_t in, int32_t k) {
  int32_t diff = in - filt;
  int32_t step = diff / k;
  return filt + (step != 0 ? step : diff);
}
static int32_t Max(int32_t a, int32_t b) { return a > b ? a : b; }

Tcu::Tcu()
    : cfg(), out(), dntFilt(0), lastNt(0), ndSynFilt(0), lastNdSyn(0),
      tqRamp(0), tqRampStep(0), gearErrMs(0), firstTick(true),
      fromPark(false) {
  out.mode = TCU_INIT;
  out.gear = GEAR_N;
  out.target = GEAR_N;
  out.safeOutputs = true;
  out.tSyn = SAS_TSYN_NONE;
}

int32_t Tcu::FillLevel(int cl, int32_t temp) {
  if (cl < 0 || cl >= CL_COUNT)
    return 0;
  return Cal::Curve(calClutch[cl].pfcg, temp) +
         Cal::Curve(calClutch[cl].pfhyscg, temp);
}

void Tcu::ForceSafe(TcuFault f) {
  sas.Abort();
  engage.Abort();
  out.mode = TCU_SAFE;
  if (out.fault == TF_NONE)
    out.fault = (uint8_t)f;
  out.safeOutputs = true;
  for (int c = 0; c < CL_COUNT; c++)
    out.p[c] = 0;
  out.pSys = 0;
  out.tqDelta = 0;
  tqRamp = 0;
  out.gear = GEAR_LIMP; // what the hydraulics fall into with no current
  out.target = GEAR_LIMP;
}

void Tcu::SetNeutral(const TcuInputs &in) {
  for (int c = 0; c < CL_COUNT; c++)
    out.p[c] = 0;
  // OEM neutral keeps B at fill pressure, ready for 1st or R [C].
  out.p[CL_B] = Cal::Curve(calClutch[CL_B].pfcg, in.temp);
}

void Tcu::HoldGear(int gear) {
  uint8_t m = Gears::ClutchMask(gear);
  for (int c = 0; c < CL_COUNT; c++)
    out.p[c] = (m & CLUTCH_BIT(c)) ? cfg.applyP : 0;
}

int32_t Tcu::LinePressure(const TcuInputs &in, int32_t minP) const {
  int32_t p;
  int g = out.gear;
  if ((out.mode == TCU_DRIVE || out.mode == TCU_SHIFT) && Gears::IsDriveGear(g)) {
    const CalMap &m = calPsys[g == GEAR_R ? 0 : g];
    p = Cal::Map(m, in.m_t, in.n_mot);
  } else {
    p = Cal::Map(calGlobal.psysn, in.n_mot, in.n_ab);
  }
  p = Max(p, minP) + cfg.lineOffset;
  if (p > cfg.lineMax)
    p = cfg.lineMax;
  return p;
}

bool Tcu::GearPlausible(const TcuInputs &in, int gear) const {
  if (!Gears::IsDriveGear(gear) || in.n_ab < cfg.gearCheckMinNab)
    return true;
  int32_t ratio = (int32_t)(((int64_t)in.n_t * 1000) / in.n_ab);
  int32_t want = Gears::RatioX1000(gear);
  return Abs(ratio - want) * 100 <= want * cfg.gearCheckPct;
}

int Tcu::ChooseTarget(const TcuInputs &in) const {
  int g = out.gear;
  int top = cfg.topGear < 1 ? 1 : (cfg.topGear > 8 ? 8 : cfg.topGear);
  if (g == GEAR_R || !Gears::IsDriveGear(g))
    return g;
  if (cfg.gearSelMode == GSEL_VCU) {
    int r = in.gearReq;
    if (r < 1)
      return g; // 0 = hold
    return r > top ? top : r;
  }
  // GSEL_AUTO: upshift on input speed, downshift when the input speed is
  // low and the lower gear will not immediately want to upshift again.
  if (g < top && in.n_t > cfg.autoUpRpm)
    return g + 1;
  if (g > 1 && in.n_t < cfg.autoDownRpm &&
      Gears::SyncSpeed(g - 1, in.n_ab) < cfg.autoUpRpm - 200)
    return g - 1;
  return g;
}

SasInputs Tcu::MakeSasInputs(const TcuInputs &in) {
  SasInputs s;
  s.m_t = in.m_t;
  s.n_t = in.n_t;
  s.n_mot = in.n_mot;
  s.temp = in.temp;
  s.dnt = dntFilt;
  int tgt = sas.Active() ? sas.To() : out.target;
  int cur = sas.Active() ? sas.From() : out.gear;
  s.ndSyn = in.n_t - Gears::SyncSpeed(tgt, in.n_ab);
  s.ndSynIst = in.n_t - Gears::SyncSpeed(cur, in.n_ab);
  // Filtered rate of change of the slip to the target gear.
  int32_t d = (s.ndSyn - lastNdSyn) * (1000 / TCU_TICK_MS);
  lastNdSyn = s.ndSyn;
  ndSynFilt = Pt1(ndSynFilt, d, cfg.dntTauMs / TCU_TICK_MS + 1);
  // Time to sync: slip / closing rate, only if the slip is shrinking.
  if ((s.ndSyn > 0 && ndSynFilt < 0) || (s.ndSyn < 0 && ndSynFilt > 0)) {
    int64_t t = ((int64_t)s.ndSyn * 1000) / -ndSynFilt;
    s.tSyn = t > 60000 ? 60000 : (int32_t)t;
  } else {
    s.tSyn = SAS_TSYN_NONE;
  }
  out.tSyn = s.tSyn;
  return s;
}

void Tcu::StartShift(const TcuInputs &in, int next) {
  int off, on;
  if (!Gears::SingleSwap(out.gear, next, off, on))
    return;
  SasInputs s = MakeSasInputs(in);
  int from = out.gear;
  out.target = (uint8_t)next;
  s.ndSyn = in.n_t - Gears::SyncSpeed(next, in.n_ab);
  if (sas.Start(from, next, off, on, cfg.sas, s))
    out.mode = TCU_SHIFT;
}

void Tcu::Tick10ms(const TcuInputs &in) {
  // Input speed gradient, PT1 filtered (OEM T_FNGTGLS = 70 ms).
  if (firstTick) {
    lastNt = in.n_t;
    firstTick = false;
  }
  dntFilt = Pt1(dntFilt, (in.n_t - lastNt) * (1000 / TCU_TICK_MS),
                cfg.dntTauMs / TCU_TICK_MS + 1);
  lastNt = in.n_t;

  // ---- Faults -------------------------------------------------------
  if (out.mode != TCU_SAFE && out.mode != TCU_INIT) {
    if (in.configError)
      ForceSafe(TF_CONFIG);
    else if (in.driverFault)
      ForceSafe(TF_DRIVER);
    else if (cfg.uvLimit > 0 && in.vbat < cfg.uvLimit)
      ForceSafe(TF_UNDERVOLT);
  }
  if (out.mode == TCU_DRIVE && in.speedsValid) {
    gearErrMs = GearPlausible(in, out.gear) ? 0 : gearErrMs + TCU_TICK_MS;
    if (cfg.gearCheckMs > 0 && gearErrMs >= cfg.gearCheckMs)
      ForceSafe(TF_SPEED);
  } else {
    gearErrMs = 0;
  }

  if (out.mode == TCU_SAFE) {
    ForceSafe((TcuFault)out.fault);
    bool causesGone = !in.configError && !in.driverFault &&
                      !(cfg.uvLimit > 0 && in.vbat < cfg.uvLimit);
    if (in.faultClear && causesGone && in.n_ab < 50 &&
        (in.selector == SEL_N || in.selector == SEL_P)) {
      out.fault = TF_NONE;
      out.mode = TCU_NEUTRAL;
      out.gear = out.target = GEAR_N;
    }
    return;
  }

  if (out.mode == TCU_INIT) {
    for (int c = 0; c < CL_COUNT; c++)
      out.p[c] = 0;
    out.pSys = 0;
    out.safeOutputs = true;
    return;
  }
  out.safeOutputs = false;
  int32_t shiftTq = 0;

  // ---- Mode logic ---------------------------------------------------
  bool wantN = in.selector == SEL_N || in.selector == SEL_P;
  int wantDir = in.selector == SEL_D ? GEAR_1 : (in.selector == SEL_R ? GEAR_R : GEAR_N);
  int32_t minLine = 0;
  uint8_t allowed = 0;

  switch (out.mode) {
  case TCU_PARK:
  case TCU_NEUTRAL:
    out.gear = out.target = GEAR_N;
    out.mode = in.selector == SEL_P ? TCU_PARK : TCU_NEUTRAL;
    if (out.mode == TCU_PARK)
      fromPark = true;
    SetNeutral(in);
    allowed = CLUTCH_BIT(CL_B);
    if (!wantN && in.n_ab <= cfg.engageMaxNab && engage.Start(wantDir, cfg.engage)) {
      out.mode = TCU_ENGAGE;
      out.target = (uint8_t)wantDir;
    }
    break;
  case TCU_ENGAGE:
    if (wantN || out.target != wantDir) {
      engage.Abort();
      out.mode = TCU_NEUTRAL;
      SetNeutral(in);
      allowed = CLUTCH_BIT(CL_B);
      break;
    }
    {
      EngageInputs ei = {in.m_t, in.n_t, in.n_ab, in.temp};
      engage.Tick(ei);
    }
    for (int c = 0; c < CL_COUNT; c++)
      out.p[c] = engage.Pressure(c);
    allowed = Gears::ClutchMask(out.target) | CLUTCH_BIT(CL_B);
    minLine = out.target == GEAR_R ? calGlobal.p_sysmin_r : calGlobal.p_sysmin_d;
    if (fromPark)
      minLine = Max(minLine, calGlobal.p_sysminpsaus); // park release needs 17 bar
    if (engage.Done()) {
      out.gear = out.target;
      out.mode = TCU_DRIVE;
      fromPark = false;
      HoldGear(out.gear);
    }
    break;
  case TCU_DRIVE: {
    if (wantN || (out.gear == GEAR_R) != (wantDir == GEAR_R)) {
      // N, P or a change of direction: release everything and go via N.
      out.mode = TCU_NEUTRAL;
      out.gear = out.target = GEAR_N;
      SetNeutral(in);
      allowed = CLUTCH_BIT(CL_B);
      break;
    }
    HoldGear(out.gear);
    allowed = Gears::ClutchMask(out.gear);
    int tgt = ChooseTarget(in);
    out.target = (uint8_t)tgt;
    if (tgt != out.gear && in.canOk && in.speedsValid) {
      int next = Gears::NextStep(out.gear, tgt);
      StartShift(in, next);
      if (out.mode == TCU_SHIFT)
        allowed = Gears::ClutchMask(sas.From()) | Gears::ClutchMask(sas.To());
    }
    break;
  }
  case TCU_SHIFT:
    break;
  default:
    break;
  }

  if (out.mode == TCU_SHIFT) {
    if (in.selector == SEL_N || in.selector == SEL_P) {
      // Leaving the shift for neutral is always safe: open everything.
      sas.Abort();
      out.mode = TCU_NEUTRAL;
      out.gear = out.target = GEAR_N;
      SetNeutral(in);
      allowed = CLUTCH_BIT(CL_B);
    } else {
      SasInputs s = MakeSasInputs(in);
      sas.Tick(s);
      const SasOutputs &so = sas.Out();
      HoldGear(sas.From());
      out.p[sas.OffClutch()] = so.pKab;
      out.p[sas.OnClutch()] = so.pKzu;
      allowed = Gears::ClutchMask(sas.From()) | Gears::ClutchMask(sas.To());
      minLine = so.pSysMin;
      out.zKzu = so.zKzu;
      out.zKab = so.zKab;
      out.gls = so.gls;
      int32_t tq = so.tqDelta;
      if (cfg.tqInterv == 0 || (cfg.tqInterv == 1 && !sas.IsUpshift()))
        tq = 0;
      shiftTq = tq;
      tqRamp = tq; // ramped out from here if the shift ends now
      tqRampStep = 0;
      if (so.done) {
        out.gear = (uint8_t)sas.To();
        out.mode = TCU_DRIVE;
        HoldGear(out.gear);
        allowed = Gears::ClutchMask(out.gear);
        int steps = cfg.tqRampMs / TCU_TICK_MS;
        tqRampStep = steps > 0 ? (tqRamp + (tqRamp > 0 ? steps - 1 : -(steps - 1))) / steps : tqRamp;
      }
    }
  }
  if (out.mode != TCU_SHIFT) {
    out.zKzu = 0;
    out.zKab = 0;
    out.gls = 0;
  }

  // ---- Torque intervention, ramped out after the shift (T_RAMEE) ------
  if (out.mode == TCU_SHIFT) {
    out.tqDelta = shiftTq;
  } else if (tqRamp != 0) {
    int32_t next = tqRamp - (tqRampStep != 0 ? tqRampStep : tqRamp);
    if ((tqRamp > 0 && next < 0) || (tqRamp < 0 && next > 0))
      next = 0;
    tqRamp = next;
    out.tqDelta = tqRamp;
  } else {
    out.tqDelta = 0;
  }

  // ---- Line pressure --------------------------------------------------
  out.pSys = LinePressure(in, minLine);
  out.engPhase = engage.Phase();
  out.parkRequest = out.mode == TCU_PARK && in.n_ab < 5;

  // ---- Interlock: last check before anything reaches the valves ------
  InterlockInput il;
  for (int c = 0; c < CL_COUNT; c++) {
    il.pressure[c] = out.p[c];
    il.fillLevel[c] = FillLevel(c, in.temp);
  }
  il.allowedMask = allowed;
  il.shifting = out.mode == TCU_SHIFT;
  InterlockResult r = Interlock::Check(il);
  out.interlock = (uint8_t)r;
  out.applied = Interlock::AppliedMask(il);
  if (r != IL_OK)
    ForceSafe(TF_INTERLOCK);
}
