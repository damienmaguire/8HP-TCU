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
#include "calset.h"
#include "caltypes.h"
#include "eds.h"

#define TCU_TICK_MS 10 // ms, Tick10ms period

// Absolute value.
static int32_t Abs(int32_t v) { return v < 0 ? -v : v; }

/* First-order low pass, k = tau / tick + 1. Steps smaller than one count
 * jump straight to the input so the output settles exactly. */
static int32_t Pt1(int32_t filt, int32_t in, int32_t k) {
  int32_t diff = in - filt;
  int32_t step = diff / k;
  return filt + (step != 0 ? step : diff);
}
// Larger of two values.
static int32_t Max(int32_t a, int32_t b) { return a > b ? a : b; }

// Power-up: INIT mode, neutral, outputs off until Ready().
Tcu::Tcu()
    : cfg(), out(), dntFilt(0), lastNt(0), ndSynFilt(0), lastNdSyn(0),
      tqRamp(0), tqRampStep(0), gearErrMs(0), firstTick(true),
      fromPark(false), pnMs(0), npMs(0), relMs(-1), tipLeft(0),
      fromDrive(false), tccLocked(false) {
  out.mode = TCU_INIT;
  out.gear = GEAR_N;
  out.target = GEAR_N;
  out.safeOutputs = true;
  out.tSyn = SAS_TSYN_NONE;
  out.parkState = PARK_UNHELD;
}

// Current the driver will get for a clutch pressure (for the interlock).
int32_t Tcu::CommandCurrent(int cl, int32_t p, int32_t temp) const {
  // Clutch A..E map straight onto EDS_A..EDS_E.
  return Eds::CommandCurrent((EdsValve)cl, p, temp, cfg.tempCorrection, cfg.iMax);
}

// Go to the safe state: all coils off. The hydraulics then hold the limp gear
// (6th with full line pressure when moving). Only the first fault is kept.
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
  out.tqHold = false;
  out.canLossHold = false;
  out.pCh7 = 0;
  out.nMotReq = 0;
  out.pzylI = 0;
  out.parkMagnet = false;
  out.parkWait = false;
  out.parkState = PARK_UNHELD;
  out.inertia = false;
  out.gearInWait = false;
  out.tipAuto = false;
  tipLeft = 0;
  tccLocked = false;
  out.tccLocked = false;
  tccCtl.Reset();
  tqRamp = 0;
  out.gear = GEAR_LIMP; // what the hydraulics fall into with no current
  out.target = GEAR_LIMP;
}

// Neutral: every clutch open except B kept filled, ready for 1st or R.
void Tcu::SetNeutral(const TcuInputs &in) {
  for (int c = 0; c < CL_COUNT; c++)
    out.p[c] = 0;
  // OEM neutral keeps B at fill pressure, ready for 1st or R [C].
  out.p[CL_B] = Cal::Curve(Cal::Clutch()[CL_B].pfcg, in.temp);
}

// Full apply pressure on the gear's clutches, the others open.
void Tcu::HoldGear(int gear) {
  uint8_t m = Gears::ClutchMask(gear);
  for (int c = 0; c < CL_COUNT; c++)
    out.p[c] = (m & CLUTCH_BIT(c)) ? cfg.applyP : 0;
}

// Line pressure from the gear's map (torque and motor speed), with limits.
int32_t Tcu::LinePressure(const TcuInputs &in, int32_t minP) const {
  int32_t p;
  int g = out.gear;
  if ((out.mode == TCU_DRIVE || out.mode == TCU_SHIFT) && Gears::IsDriveGear(g)) {
    const CalMap &m = Cal::Active().psys[g == GEAR_R ? 0 : g];
    p = Cal::Map(m, in.m_t, in.n_mot);
  } else {
    p = Cal::Map(Cal::Active().global->psysn, in.n_mot, in.n_ab);
  }
  p = Max(p, minP) + cfg.lineOffset;
  if (p > cfg.lineMax)
    p = cfg.lineMax;
  return p;
}

// Does the measured speed ratio match the gear? (wrong ratio = slipping or bad sensor)
bool Tcu::GearPlausible(const TcuInputs &in, int gear) const {
  if (!Gears::IsDriveGear(gear) || in.n_ab < cfg.gearCheckMinNab)
    return true;
  int32_t ratio = (int32_t)(((int64_t)in.n_t * 1000) / in.n_ab);
  int32_t want = Gears::RatioX1000(gear);
  return Abs(ratio - want) * 100 <= want * cfg.gearCheckPct;
}

// Gear to aim for: Auto, Manual or VCU request, CAN-loss and over-rev rules.
int Tcu::ChooseTarget(const TcuInputs &in, bool manual) const {
  int g = out.gear;
  int top = cfg.topGear < 1 ? 1 : (cfg.topGear > 8 ? 8 : cfg.topGear);
  if (g == GEAR_R || !Gears::IsDriveGear(g))
    return g;
  if (!in.canOk && cfg.canLossMode == CANLOSS_HOLD) {
    // CAN lost, hold mode: no upshifts. Still step down as the car slows
    // so it does not lug in a tall gear (same rule as SHIFT_AUTO).
    if (g > 1 && in.n_t < cfg.autoDownRpm &&
        Gears::SyncSpeed(g - 1, in.n_ab) < cfg.autoUpRpm - 200)
      return g - 1;
    return g;
  }
  // Lugging protection (Auto and Manual): step down when the input speed
  // is low and the lower gear will not immediately want to go up again.
  bool lug = g > 1 && in.n_t < cfg.autoDownRpm &&
             Gears::SyncSpeed(g - 1, in.n_ab) < cfg.autoUpRpm - 200;
  if (cfg.shiftMode == SHIFT_VCU) {
    int r = in.gearReq;
    if (r < 1)
      return g; // 0 = hold
    return r > top ? top : r;
  }
  if (manual) {
    // Driver tips, one gear per tip, inside the rpm limits. The interlock
    // and the shift sequences are the same as in Auto. A tip that would
    // over-rev (down) or lug (up) the input is ignored, not queued.
    if (lug)
      return g - 1;
    if (in.manualGear >= 1 && in.manualGear <= 8) {
      // ManualGear picked a gear: step towards it one gear at a time, each
      // step only inside the same rpm limits as a tip. A gear the limits
      // do not allow yet is waited for, not forced.
      int want = in.manualGear > top ? top : in.manualGear;
      if (want > g && Gears::SyncSpeed(g + 1, in.n_ab) >= cfg.manMinRpm)
        return g + 1;
      if (want < g && Gears::SyncSpeed(g - 1, in.n_ab) <= cfg.manMaxRpm)
        return g - 1;
      return g;
    }
    if (in.tipUp && g < top &&
        Gears::SyncSpeed(g + 1, in.n_ab) >= cfg.manMinRpm)
      return g + 1;
    if (in.tipDown && g > 1 &&
        Gears::SyncSpeed(g - 1, in.n_ab) <= cfg.manMaxRpm)
      return g - 1;
    return g;
  }
  // SHIFT_AUTO: upshift on input speed, downshift when lugging. Track
  // (noLiftUpshift): no upshift with the driver off the throttle, except
  // at the over-rev limit.
  bool lift = cfg.noLiftUpshift && in.m_t <= cfg.liftTqNm;
  if (g < top && in.n_t > cfg.autoUpRpm && (!lift || in.n_t >= cfg.manMaxRpm))
    return g + 1;
  if (lug)
    return g - 1;
  return g;
}

// Inputs for the shift code: slip to the target gear and time to sync.
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

/// Leaving P: true when the motor turns fast enough or the measured line
/// pressure is high enough to release the park lock.
static bool ParkReleaseOk(const TcuConfig &cfg, const TcuInputs &in) {
  if (cfg.parkRelRpm <= 0)
    return true;
  return in.n_mot >= cfg.parkRelRpm || (in.pMeas >= 0 && in.pMeas >= cfg.parkRelP);
}

/* N -> P delay KL_TVPARKSPCGNP [C]: 1000 / 500 / 300 / 150 ms at -30 / -15 /
 * 0 / 40 degC. From a drive range the OEM uses KL_TVPSEIN instead. */
static const int32_t npT[4] = {-30, -15, 0, 40};
static const int32_t npMsTab[4] = {1000, 500, 300, 150};
/* PZYL cold offset [C values, I: PSV = PZYL]: +190 / 170 / 50 / 0 mA at
 * -30 / -20 / 0 / 20 degC, in 0.1 mA. */
static const int32_t pzT[4] = {-30, -20, 0, 20};
static const int32_t pzOfs[4] = {1900, 1700, 500, 0};

// Gear-in only once something makes oil pressure (pump running or motor turning).
bool Tcu::PressureSource(const TcuInputs &in) const {
  if (cfg.gearInPumpMs <= 0 && cfg.gearInMotRpm <= 0)
    return true;
  if (cfg.gearInPumpMs > 0 && in.pumpDeliverMs >= cfg.gearInPumpMs)
    return true;
  return cfg.gearInMotRpm > 0 && in.n_mot >= cfg.gearInMotRpm;
}

// Park release help: motor speed request, park cylinder valve and hold magnet.
void Tcu::ParkOutputs(const TcuInputs &in) {
  if (relMs >= 0 && relMs < 0x7FFFFFF)
    relMs += TCU_TICK_MS;
  bool leavingP = out.mode == TCU_PARK && in.selector != SEL_P;
  int32_t req = 0;
  if (leavingP && cfg.parkRelRpm > 0)
    req = cfg.parkRelRpm; // spin the motor (clutches open) for the release
  if (relMs >= 0 && relMs < cfg.parkRelHoldMs && cfg.parkRelRpm > 0)
    req = Max(req, cfg.parkRelRpm); // and hold it for T_NMINEMPAUS
  if (in.pumpFault && cfg.pumpFaultAct == PUMPFAULT_MOTOR &&
      (out.mode == TCU_PARK || out.mode == TCU_NEUTRAL))
    req = Max(req, cfg.pumpFaultRpm);
  out.nMotReq = req;

  // PZYL energised while the lock is being released and while it is out
  // (not in P); park magnet on after the release for parkMagMinMs ..
  // parkMagMaxMs [C times, I sequence].
  bool released = out.mode != TCU_PARK || (leavingP && pnMs > 0);
  out.pzylI = cfg.parkCtrl && released
                  ? cfg.pzylI + Cal::Interp(pzT, pzOfs, 4, in.temp)
                  : 0;
  // The magnet only latches the piston for at most 1 s after a release.
  // It is never kept on to hold N with the motor (and so the oil pressure)
  // off: without pressure or magnet the spring puts the lock in, which is
  // the safe way round.
  int32_t magMax = cfg.parkMagMaxMs < PARK_MAG_LIMIT_MS ? cfg.parkMagMaxMs : PARK_MAG_LIMIT_MS;
  out.parkMagnet = cfg.parkCtrl && relMs >= 0 && relMs < magMax &&
                   (out.mode != TCU_PARK || relMs < cfg.parkMagMinMs);
}

// Park lock state for display from the mode and selector.
static uint8_t ParkStateOf(uint8_t mode, uint8_t selector) {
  switch (mode) {
  case TCU_PARK:
    return selector == SEL_P ? PARK_ENGAGED : PARK_RELEASING;
  case TCU_NEUTRAL:
    return selector == SEL_P ? PARK_ENGAGING : PARK_RELEASED;
  case TCU_ENGAGE:
  case TCU_DRIVE:
  case TCU_SHIFT:
    return PARK_RELEASED;
  default:
    return PARK_UNHELD;
  }
}

// Channel 7: K0 pressure, or the torque converter lock-up clutch (TCC).
void Tcu::Ch7(const TcuInputs &in) {
  int32_t p = 0;
  if (cfg.ch7Mode == CH7_K0) {
    p = cfg.k0P;
  } else if (cfg.ch7Mode == CH7_TCC) {
    // Lock-up only in a steady drive gear, never during a shift, gear-in
    // or in N/P, and never with a lock-up slip fault. Opens at once.
    bool can = out.mode == TCU_DRIVE && Gears::IsDriveGear(out.gear) &&
               out.gear != GEAR_R && out.gear >= cfg.tccMinGear && !in.tccFault;
    if (cfg.tccMode == TCCMODE_SCHEDULE) {
      // Schedule + PI on slip (tcc.h). Without an engine / motor speed
      // the slip is unknown: run on the start pressure and integrator only.
      TccConfig tc = cfg.tcc;
      tc.maxP = cfg.tccP;
      int32_t nMot = in.n_mot > 0 ? in.n_mot : in.n_t;
      p = tccCtl.Tick10ms(tc, can, out.gear, in.pedal, in.n_t, nMot);
      tccLocked = tccCtl.Closed();
    } else {
      // Simple: on / off input speeds, apply along tccRamp [I: TBD values].
      tccCtl.Reset();
      if (!can)
        tccLocked = false;
      else if (tccLocked ? in.n_t < cfg.tccOffRpm : in.n_t >= cfg.tccOnRpm)
        tccLocked = !tccLocked;
      if (tccLocked) {
        p = cfg.tccRamp > 0 ? out.pCh7 + cfg.tccRamp : cfg.tccP;
        if (p > cfg.tccP)
          p = cfg.tccP;
      }
    }
  } else {
    tccLocked = false;
  }
  out.pCh7 = p;
  out.tccLocked = tccLocked;
}

// Start a shift to the next gear, or hold if the pair has no calibration.
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

// The TCU mode machine, every 10 ms (safety checks first, outputs last).
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
  // In plain words: a bad configuration, a valve driver fault, low battery
  // voltage or a gear ratio that does not match the selected gear (clutch
  // slipping or a speed sensor lying) all switch every valve off. With no
  // current the hydraulics fall back to their built-in limp gear, so the car
  // still has drive and cannot be put into two gears at once by software.
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

  // Safe state is only left on request (fault clear), with the causes gone,
  // the car (almost) stopped and the lever in N or P: never while driving.
  if (out.mode == TCU_SAFE) {
    ForceSafe((TcuFault)out.fault);
    out.parkState = PARK_UNHELD;
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
    out.pCh7 = 0;
    out.nMotReq = 0;
    out.pzylI = 0;
    out.parkMagnet = false;
    out.parkState = PARK_UNHELD;
    out.safeOutputs = true;
    return;
  }
  out.safeOutputs = false;
  out.tqHold = false;
  int32_t shiftTq = 0;
  // CAN loss: without the VCU frames we do not know the torque or the
  // driver's intent, so no gear-in and no upshifts. With CanLossMode = hold
  // (default) the gear is kept and only steps down as the car slows, so it
  // can pull away again; with CanLossMode = neutral stm32_tcu.cpp turns the
  // selector to N and the clutches open.
  bool canHold = !in.canOk && cfg.canLossMode == CANLOSS_HOLD;

  // Manual shifting: always in Manual; in Auto for tipOverrideMs after a
  // tip (the 12 V up/down inputs or the shifter tips). Never in VCU mode.
  bool tip = in.tipUp || in.tipDown;
  if (cfg.shiftMode == SHIFT_AUTO && cfg.tipOverrideMs > 0 && tip &&
      out.mode == TCU_DRIVE)
    tipLeft = cfg.tipOverrideMs;
  else if (tipLeft > 0 && (out.mode == TCU_DRIVE || out.mode == TCU_SHIFT))
    tipLeft -= TCU_TICK_MS;
  else if (out.mode != TCU_DRIVE && out.mode != TCU_SHIFT)
    tipLeft = 0;
  if (cfg.shiftMode != SHIFT_AUTO)
    tipLeft = 0;
  out.tipAuto = tipLeft > 0;
  bool manual = cfg.shiftMode == SHIFT_MANUAL || out.tipAuto;

  // ---- Mode logic ---------------------------------------------------
  bool wantN = in.selector == SEL_N || in.selector == SEL_P;
  int wantDir = in.selector == SEL_D ? GEAR_1 : (in.selector == SEL_R ? GEAR_R : GEAR_N);
  int32_t minLine = 0;
  uint8_t allowed = 0;
  out.parkInhibit = false;

  switch (out.mode) {
  case TCU_PARK:
    out.gear = out.target = GEAR_N;
    fromPark = true;
    SetNeutral(in);
    allowed = CLUTCH_BIT(CL_B);
    out.parkWait = false;
    out.parkBrakeWait = false;
    if (in.selector == SEL_P) {
      pnMs = 0;
      break;
    }
    // In plain words: the park pawl is pushed out by oil pressure, so the
    // TCU asks for line pressure and waits until the motor turns (pump
    // running) or the pressure is measured before it counts as released.
    // Until then the driver sees "waiting" and the car stays in P.
    // Leaving P: the lock comes out only with the motor turning or enough
    // measured pressure (11 Q9d), then T_VPARKSPPN to N.
    // The driver must also hold the footbrake (TIS release conditions:
    // brake pressed and R, D or N selected). Both have to stay true for the
    // whole P -> N delay, so 17 bar must be stable for that long.
    minLine = cfg.parkRelP;
    out.parkWait = !ParkReleaseOk(cfg, in);
    out.parkBrakeWait = cfg.parkNeedBrake && !in.brake;
    if (out.parkWait || out.parkBrakeWait) {
      pnMs = 0;
      break;
    }
    pnMs += TCU_TICK_MS;
    if (pnMs >= cfg.pnDelayMs) {
      pnMs = 0;
      relMs = 0;
      npMs = 0;
      out.mode = TCU_NEUTRAL;
    }
    break;
  case TCU_NEUTRAL:
    out.gear = out.target = GEAR_N;
    SetNeutral(in);
    allowed = CLUTCH_BIT(CL_B);
    out.gearInWait = false;
    if (in.selector == SEL_P) {
      // In plain words: P is only engaged once the car has (almost) stopped.
      // Dropping the pawl into a turning output would be refused by the
      // pawl or break it, so above ParkEngKmh, or while the output shaft
      // still turns at ParkEngNab or more, the TCU stays in N: clutches
      // open, PZYL still holding the lock out. Once stopped, the OEM delay
      // runs and then PZYL and the magnet are switched off so the spring
      // pushes the pawl in.
      int32_t nab = Abs(in.n_ab);
      bool moving = (cfg.parkEngNab > 0 && nab >= cfg.parkEngNab) ||
                    (cfg.parkEngKmh > 0 && cfg.nabPerKmh > 0 &&
                     nab > cfg.parkEngKmh * cfg.nabPerKmh);
      if (moving) {
        npMs = 0;
        out.parkInhibit = true;
        break;
      }
      // N -> P after the OEM delay (from a drive range KL_TVPSEIN).
      npMs += TCU_TICK_MS;
      int32_t d = fromDrive ? Cal::Curve(Cal::Active().global->tvpsein, in.temp)
                            : Cal::Interp(npT, npMsTab, 4, in.temp);
      if (npMs >= d) {
        npMs = 0;
        fromDrive = false;
        out.mode = TCU_PARK;
        fromPark = true;
      }
      break;
    }
    npMs = 0;
    fromDrive = false;
    // No gear-in without the VCU frames, without a pressure source, or
    // (PumpFaultAct = Stop) with a pump fault.
    out.gearInWait = !wantN && (!PressureSource(in) ||
                                (in.pumpFault && cfg.pumpFaultAct == PUMPFAULT_STOP));
    if (!wantN && in.canOk && in.n_ab <= cfg.engageMaxNab && !out.gearInWait &&
        engage.Start(wantDir, cfg.engage)) {
      out.mode = TCU_ENGAGE;
      out.target = (uint8_t)wantDir;
      out.tqHold = engage.TorqueHold(); // ask for 0 Nm from the first tick
    }
    break;
  case TCU_ENGAGE:
    if (wantN || out.target != wantDir) {
      engage.Abort();
      fromDrive = in.selector == SEL_P;
      out.mode = TCU_NEUTRAL;
      SetNeutral(in);
      allowed = CLUTCH_BIT(CL_B);
      break;
    }
    minLine = out.target == GEAR_R ? Cal::Active().global->p_sysmin_r : Cal::Active().global->p_sysmin_d;
    minLine = Max(minLine, cfg.gearInMinP); // P_SYSMINIEP 8 bar
    if (fromPark)
      minLine = Max(minLine, Cal::Active().global->p_sysminpsaus); // park release needs 17 bar
    {
      // Line pressure is adequate if the command can reach the OEM minimum
      // for gear-in (it cannot if LineMax is set below it).
      EngageInputs ei = {in.m_t, in.n_t, in.n_ab, in.temp, in.speedsValid,
                         LinePressure(in, minLine) >= minLine};
      engage.Tick(ei);
    }
    for (int c = 0; c < CL_COUNT; c++)
      out.p[c] = engage.Pressure(c);
    out.tqHold = engage.TorqueHold();
    allowed = Gears::ClutchMask(out.target) | CLUTCH_BIT(CL_B);
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
      fromDrive = in.selector == SEL_P;
      out.mode = TCU_NEUTRAL;
      out.gear = out.target = GEAR_N;
      SetNeutral(in);
      allowed = CLUTCH_BIT(CL_B);
      break;
    }
    HoldGear(out.gear);
    allowed = Gears::ClutchMask(out.gear);
    int tgt = ChooseTarget(in, manual);
    out.target = (uint8_t)tgt;
    // Shifts need the VCU frames, except the downshifts CanLossMode 1
    // makes as the car slows.
    if (tgt != out.gear && (in.canOk || (canHold && tgt < out.gear)) &&
        in.speedsValid) {
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
      fromDrive = in.selector == SEL_P;
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
      out.inertia = so.inertia;
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
    out.inertia = false;
    out.zKzu = 0;
    out.zKab = 0;
    out.gls = 0;
    out.coast = false;
  } else {
    out.coast = sas.IsCoast();
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
  // CanLossMode 1: hold the gear at high line pressure so nothing slips.
  out.canLossHold = canHold && (out.mode == TCU_DRIVE || out.mode == TCU_SHIFT);
  if (out.canLossHold)
    out.pSys = cfg.lineMax;
  out.engPhase = engage.Phase();
  out.parkRequest = out.mode == TCU_PARK && in.n_ab < 5;
  ParkOutputs(in);
  out.parkState = ParkStateOf(out.mode, in.selector);
  Ch7(in);

  // ---- Interlock: last check before anything reaches the valves ------
  InterlockInput il;
  // Checked on the coil currents the EDS layer will be given for these
  // pressures (targets, before slew and dither).
  for (int c = 0; c < CL_COUNT; c++)
    il.current[c] = CommandCurrent(c, out.p[c], in.temp);
  il.openLow = cfg.ilOpenLow;
  il.openHigh = cfg.ilOpenHigh;
  il.allowedMask = allowed;
  il.shifting = out.mode == TCU_SHIFT;
  InterlockResult r = Interlock::Check(il);
  out.interlock = (uint8_t)r;
  out.applied = Interlock::AppliedMask(il);
  if (r != IL_OK)
    ForceSafe(TF_INTERLOCK);
}

// Allow an output on for at most maxMs per request (see tcu.h).
bool OnTimeLimit::Tick(bool request, int32_t maxMs, int32_t tickMs) {
  if (!request) {
    onMs = 0;
    return false;
  }
  if (onMs >= maxMs)
    return false;
  onMs += tickMs;
  return true;
}

// Footbrake from the ParkBrake source (see tcu.h).
bool FootbrakeFrom(int src, bool canOk, bool canBrake, bool paramBrake,
                   bool pinFitted, bool pinBrake) {
  switch (src) {
  case PARKBRAKE_PARAM:
    return paramBrake;
  case PARKBRAKE_BYPASS:
    return true;
  case PARKBRAKE_INPUT:
    return pinFitted && pinBrake;
  default: // PARKBRAKE_CAN
    return canOk && canBrake;
  }
}

// Change state only after n equal samples that differ from it.
bool Debounce::Tick(bool raw, int n) {
  if (raw == state) {
    count = 0;
  } else if (++count >= n) {
    state = raw;
    count = 0;
  }
  return state;
}
