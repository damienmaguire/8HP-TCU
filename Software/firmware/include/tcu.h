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
#ifndef TCU_H
#define TCU_H

#include "engage.h"
#include "gears.h"
#include "interlock.h"
#include "sas.h"
#include "tcc.h"
#include <stdint.h>

/* TCU top-level state machine, run every 10 ms.
 *
 * It decides the target gear, runs the gear-in (engage.h) and shift
 * (sas.h) sequences, works out line pressure, and checks the interlock.
 * Its outputs are pressures per element plus line pressure; the caller
 * turns those into currents (eds.h) and sends them to the MAX22200s.
 *
 * Nothing here touches hardware or parameters, so it runs in the host
 * unit tests. stm32_tcu.cpp copies parameters into TcuConfig and sensor
 * readings into TcuInputs.
 */

enum TcuMode
{
   TCU_INIT = 0,   // waiting for the drivers to come up
   TCU_PARK,       // selector P: elements open, park lock requested
   TCU_NEUTRAL,    // selector N: B pre-filled, others open
   TCU_ENGAGE,     // N -> 1 or N -> R running
   TCU_DRIVE,      // in gear
   TCU_SHIFT,      // single-swap shift running
   TCU_SAFE,       // fault: every coil at 0 A -> mechanical 6th + max line
};

/// Park lock as the TCU drives it (spot value parkSt). Not a sensor
/// reading: park_in / notpark_in are separate.
/// Longest the park-hold magnet may be on per energise, ms (OEM 1 s [C]).
/// It only latches the piston after a release; it is never used to hold
/// N with no oil pressure (TIS describes that, we do not support it).
#define PARK_MAG_LIMIT_MS 1000

enum ParkState
{
   PARK_ENGAGED = 0,   // in P
   PARK_RELEASING = 1, // leaving P: waiting for the source / P->N delay
   PARK_RELEASED = 2,  // out, PZYL holding it out
   PARK_ENGAGING = 3,  // N -> P delay running
   PARK_UNHELD = 4,    // INIT / SAFE: PZYL and magnet off, lock not held out
};

enum Selector
{
   SEL_P = 0,
   SEL_R = 1,
   SEL_N = 2,
   SEL_D = 3,
};

/// Mode parameter: who picks the gear in D.
enum TcuShiftMode
{
   SHIFT_AUTO = 0,   // TCU picks the gear on simple input-speed thresholds
   SHIFT_MANUAL = 1, // driver tips up/down on the shifter (tipUp/tipDown)
   SHIFT_VCU = 2,    // the VCU requests a gear over CAN1 (gearReq)
};

enum TcuFault
{
   TF_NONE = 0,
   TF_INTERLOCK = 1,   // interlock rejected a command
   TF_DRIVER = 2,      // MAX22200 fault on a pressure valve
   TF_UNDERVOLT = 3,   // battery below the load-off threshold
   TF_SPEED = 4,       // speed sensors disagree with the engaged gear
   TF_CONFIG = 5,      // valve map or driver configuration invalid
};

struct TcuConfig
{
   SasConfig sas;
   EngageConfig engage;
   uint8_t shiftMode;     // Mode (TcuShiftMode)
   int32_t applyP;        // pressure command for an applied element, mbar
   int32_t lineOffset;    // added to the OEM line pressure, mbar
   int32_t lineMax;       // ceiling on line pressure, mbar
   int32_t engageMaxNab;  // highest output speed for N->D/R, rpm
   int32_t autoUpRpm;     // Auto: upshift above this input speed
   int32_t autoDownRpm;   // Auto (and Manual protection): downshift if the
                          // input speed falls below this
   int32_t manMaxRpm;     // Manual: refuse a tip down that would put the
                          // input above this, rpm
   int32_t manMinRpm;     // Manual: refuse a tip up that would put the
                          // input below this, rpm
   int32_t uvLimit;       // loads off below this, mV (OEM 7400)
   int32_t gearCheckPct;  // allowed n_t/n_ab ratio error in gear, %
   int32_t gearCheckMinNab; // gear check only above this output speed, rpm
   int32_t gearCheckMs;   // ratio error must last this long
   int32_t tqRampMs;      // torque intervention ramp-out (OEM T_RAMEE = 100 ms)
   int32_t dntTauMs;      // input speed gradient filter (OEM T_FNGTGLS = 70 ms)
   uint8_t topGear;       // highest gear allowed (1..8)
   uint8_t tqInterv;      // 0 = no torque intervention, 1 = upshifts, 2 = both
   uint8_t canLossMode;   // CanLossMode: what to do when the VCU frames stop
   // Interlock, on commanded coil currents (see interlock.h)
   int32_t ilOpenLow;     // A/B open below this, 0.1 mA (Ian: 60 mA)
   int32_t ilOpenHigh;    // C/D/E open above this, 0.1 mA (Ian: 790 mA)
   int32_t iMax;          // coil current ceiling, 0.1 mA (same as the EDS layer)
   bool tempCorrection;   // EDS temperature correction on (same as the EDS layer)

   // Park lock (Ian, 330E_LIN_OIL_PUMP.md 11 Q9d / 12.1). Leaving P waits
   // until the motor turns at parkRelRpm or the measured line pressure is
   // parkRelP; the motor speed is asked for (nMotReq) while waiting and
   // for parkRelHoldMs after the release. parkRelRpm = 0 skips the wait.
   int32_t parkRelRpm;    // KL_NMINEMPARKSP_LLS_HAS 700 rpm [C]
   int32_t parkRelP;      // P_SYSMINPSAUS 17000 mbar [C]
   int32_t parkRelHoldMs; // T_NMINEMPAUS_LLS_HAS 2000 ms [C]
   int32_t pnDelayMs;     // P -> N state delay, T_VPARKSPPN 150 ms [C]
   bool parkCtrl;         // drive PZYL and the park magnet (off = stub)
   int32_t pzylI;         // PZYL current while released, 0.1 mA (I_EDSPSV
                          // 550 mA [C]; cold offset added, I: PSV = PZYL)
   int32_t parkMagMinMs;  // park magnet on at least (100 ms [C])
   int32_t parkMagMaxMs;  // and at most (1000 ms [C]) per release; capped at
                          // PARK_MAG_LIMIT_MS whatever the parameter says
   // Park engage (TIS cross-check T.3 / T.4). N -> P only once the car is
   // below parkEngKmh (TIS: never above 5 km/h) AND the output shaft is
   // below parkEngNab (OEM N_ABBRTVPSEIN 5 rpm [C], the stricter of the
   // two). Until then the TCU stays in N with PZYL still energised.
   int32_t parkEngKmh;    // km/h, 0 = no km/h check
   int32_t nabPerKmh;     // output rpm per km/h (axle ratio and tyre [I])
   int32_t parkEngNab;    // rpm output speed, 0 = no rpm check
   // Park release needs the footbrake (TIS: footbrake + R/D/N). false =
   // bypassed (bench only).
   bool parkNeedBrake;

   // Gear-in (11 Q9d): minimum line pressure, and a pressure source: the
   // pump has delivered for gearInPumpMs or the motor turns at gearInMotRpm
   // [I]. Both 0 = no source check.
   int32_t gearInMinP;    // P_SYSMINIEP 8000 mbar [C]
   int32_t gearInPumpMs;  // 1000 ms [I]
   int32_t gearInMotRpm;  // [I]

   // LIN pump fault reaction (11 Q9a), PumpFaultAct.
   uint8_t pumpFaultAct;  // PumpFaultAction
   int32_t pumpFaultRpm;  // minimum motor speed to ask for (500-700 [I])

   // Shift up / down inputs in Auto: a tip shifts one gear and keeps
   // Auto off for this long (0 = tips ignored in Auto).
   int32_t tipOverrideMs;

   // Channel 7 (K0 / TCC slot), see calset.h Ch7Mode.
   uint8_t ch7Mode;       // resolved: CH7_TCC, CH7_K0 or CH7_OPEN
   int32_t k0P;           // K0 mode pressure, mbar (0 = open: no engine)
   int32_t tccP;          // TCC lock-up pressure, mbar (TBD)
   int32_t tccOnRpm;      // lock above this input speed (TBD)
   int32_t tccOffRpm;     // open below this input speed (TBD)
   uint8_t tccMinGear;    // lowest gear with lock-up (TBD)
   int32_t tccRamp;       // apply ramp, mbar per 10 ms (TBD)
   uint8_t tccMode;       // TCCMODE_SIMPLE (the rule above) or TCCMODE_SCHEDULE
   TccConfig tcc;         // TCCMODE_SCHEDULE settings (tcc.h); maxP = tccP

   // DriveMode Track (drivemode.h): no Auto upshift on lift-off.
   bool noLiftUpshift;
   int32_t liftTqNm;      // input torque at or below this counts as lift-off
};

/// PumpFaultAct parameter: what the TCU does on a LIN pump fault class.
enum PumpFaultAction
{
   PUMPFAULT_REPORT = 0, // report only (default)
   PUMPFAULT_MOTOR = 1,  // ask the VCU for pumpFaultRpm in P/N (clutches
                         // open) so the mechanical pump makes pressure
   PUMPFAULT_STOP = 2,   // no new gear-in until the fault is gone
};

enum CanLossMode
{
   CANLOSS_NEUTRAL = 0, // drop to neutral
   CANLOSS_HOLD = 1,    // default (Damien, 2026-11-23): hold the gear at
                        // LineMax, no upshifts, step down as the car slows
};

struct TcuInputs
{
   int32_t m_t;       // Nm, actual input torque from the inverter
   int32_t n_t;       // rpm, input speed
   int32_t n_ab;      // rpm, output speed
   int32_t n_mot;     // rpm, motor speed (for line pressure maps)
   int32_t temp;      // degC
   int32_t vbat;      // mV
   uint8_t selector;  // Selector
   uint8_t gearReq;   // SHIFT_VCU: requested gear 1..8, 0 = hold
   bool tipUp;        // SHIFT_MANUAL: one-shot tip up from the shifter
   bool tipDown;      // SHIFT_MANUAL: one-shot tip down from the shifter
   bool canOk;        // torque/request frames arriving
   bool speedsValid;  // input and output speed sensors working
   bool driverFault;  // a pressure valve driver reported a fault
   bool configError;  // valve map invalid
   bool faultClear;   // request to leave TCU_SAFE (only at standstill)
   uint8_t manualGear; // SHIFT_MANUAL: gear picked with ManualGear, 1..8,
                       // 0 = none (tips only)
   int32_t pMeas;     // measured line pressure, mbar, < 0 = no sensor
   int32_t pumpDeliverMs; // LIN pump delivering this long, ms (0 = not)
   bool pumpFault;    // LIN pump reports a fault class
   int32_t pedal;     // accelerator pedal, % (TCC schedule)
   bool tccFault;     // lock-up slip fault: keep the TCC open
   bool brake;        // footbrake pressed (VCU CAN or BrakeIn parameter)
};

struct TcuOutputs
{
   int32_t p[CL_COUNT]; // per element, mbar
   int32_t pSys;        // line pressure, mbar
   int32_t tqDelta;     // torque intervention request to the inverter, Nm
   uint8_t mode;        // TcuMode
   uint8_t gear;        // engaged gear (enum Gear)
   uint8_t target;      // gear being aimed for
   uint8_t zKzu, zKab;  // OEM state numbers of the running shift
   uint8_t engPhase;    // EngagePhase
   uint8_t fault;       // TcuFault (latched)
   uint8_t interlock;   // last InterlockResult
   uint8_t applied;     // CLUTCH_BIT mask of clutches not open (interlock)
   bool parkRequest;    // park lock should be engaged
   bool safeOutputs;    // caller must switch every coil off
   bool tqHold;         // gear-in: hold motor torque near 0 until clamped
   bool canLossHold;    // CAN lost, holding the gear (CanLossMode 1)
   bool coast;          // the running shift is a coast (negative torque) shift
   int32_t tSyn;        // predicted ms to sync, for logging
   int32_t gls;         // GLS correction, mbar
   int32_t pCh7;        // channel 7 (K0 / TCC) pressure, mbar
   int32_t nMotReq;     // minimum motor speed asked of the VCU, rpm (0 = none)
   int32_t pzylI;       // park cylinder valve current, 0.1 mA
   bool parkMagnet;     // park-hold magnet on
   bool parkWait;       // leaving P, waiting for motor speed or pressure
   bool parkBrakeWait;  // leaving P, waiting for the footbrake
   bool parkInhibit;    // P selected but the car still moves: held in N
   bool tipAuto;        // Auto with a tip override running
   bool gearInWait;     // gear-in held: no pressure source yet
   bool inertia;        // the running shift is in its inertia phase
   uint8_t parkState;   // ParkState
   bool tccLocked;      // TCC lock-up commanded closed
};

/// One energise of an output never lasts longer than maxMs: after that it
/// is switched off and stays off until the request goes away and comes
/// back. Used on the park magnet so a stuck request (or the bench test)
/// cannot cook the coil or pretend to hold N without oil pressure.
/// Where the footbrake for the park release comes from (ParkBrake).
/// Bypass is for the bench only: on a car the lock must not come out
/// without the brake.
enum ParkBrakeSrc
{
   PARKBRAKE_CAN = 0,    // VCU frame byte 6 bit 2 (lost with CAN)
   PARKBRAKE_PARAM = 1,  // the BrakeIn parameter
   PARKBRAKE_BYPASS = 2, // not asked for at all
   PARKBRAKE_INPUT = 3   // the 12 V brake switch input (rev C, PD1)
};

/// Footbrake state for the TCU from the selected source. canOk: the VCU
/// frame is arriving. pinFitted: this board has the brake input; on a
/// board without it the Input source always reads "not pressed", so P
/// stays in rather than coming out without a brake signal.
bool FootbrakeFrom(int src, bool canOk, bool canBrake, bool paramBrake,
                   bool pinFitted, bool pinBrake);

/// A switch input that must read the same for n samples in a row before
/// its state changes (contact bounce). Starts released.
class Debounce
{
public:
   Debounce() : state(false), count(0) {}
   bool Tick(bool raw, int n);
   bool State() const { return state; }

private:
   bool state;
   int count;
};

class OnTimeLimit
{
public:
   OnTimeLimit() : onMs(0) {}
   /// Call every tickMs; returns whether the output may be on now.
   bool Tick(bool request, int32_t maxMs, int32_t tickMs);

private:
   int32_t onMs;
};

class Tcu
{
public:
   Tcu();
   void Configure(const TcuConfig &c) { cfg = c; }
   /// The drivers are up; leave TCU_INIT.
   void Ready() { if (out.mode == TCU_INIT) out.mode = TCU_NEUTRAL; }
   void Tick10ms(const TcuInputs &in);
   /// Enter the safe state from outside (e.g. driver ISR).
   void ForceSafe(TcuFault f);
   const TcuOutputs &Out() const { return out; }

   /// Coil current the EDS layer will be given for a pressure, 0.1 mA.
   int32_t CommandCurrent(int clutch, int32_t p_mbar, int32_t temp) const;

private:
   void SetNeutral(const TcuInputs &in);
   void HoldGear(int gear);
   int32_t LinePressure(const TcuInputs &in, int32_t minP) const;
   int ChooseTarget(const TcuInputs &in, bool manual) const;
   void ParkOutputs(const TcuInputs &in);
   void Ch7(const TcuInputs &in);
   bool PressureSource(const TcuInputs &in) const;
   void StartShift(const TcuInputs &in, int next);
   SasInputs MakeSasInputs(const TcuInputs &in);
   bool GearPlausible(const TcuInputs &in, int gear) const;

   TcuConfig cfg;
   TcuOutputs out;
   SasShift sas;
   Engage engage;
   int32_t dntFilt;     // rpm/s, filtered
   int32_t lastNt;
   int32_t ndSynFilt;   // filtered d(ndSyn)/dt, rpm/s
   int32_t lastNdSyn;
   int32_t tqRamp;      // torque intervention being ramped out
   int32_t tqRampStep;
   int32_t gearErrMs;
   bool firstTick;
   bool fromPark;      // engaging out of P: hold 17 bar line for the park release
   int32_t pnMs;       // ms into the P -> N release
   int32_t npMs;       // ms into the N -> P delay
   int32_t relMs;      // ms since the last park release (-1 = none yet)
   int32_t tipLeft;    // ms of tip override left (Auto)
   bool fromDrive;     // the N -> P delay started from a drive range
   bool tccLocked;
   Tcc tccCtl;         // scheduled lock-up control (TccMode = Schedule)
};

#endif // TCU_H
