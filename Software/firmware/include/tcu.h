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

enum Selector
{
   SEL_P = 0,
   SEL_R = 1,
   SEL_N = 2,
   SEL_D = 3,
};

enum GearSelMode
{
   GSEL_VCU = 0,  // gear requested over CAN1 by the VCU (GearReq)
   GSEL_AUTO = 1, // simple internal input-speed thresholds
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
   uint8_t gearSelMode;   // GearSelMode
   int32_t applyP;        // pressure command for an applied element, mbar
   int32_t lineOffset;    // added to the OEM line pressure, mbar
   int32_t lineMax;       // ceiling on line pressure, mbar
   int32_t engageMaxNab;  // highest output speed for N->D/R, rpm
   int32_t autoUpRpm;     // GSEL_AUTO: upshift above this input speed
   int32_t autoDownRpm;   // GSEL_AUTO: downshift if the lower gear stays below this
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
   uint8_t gearReq;   // GSEL_VCU: requested gear 1..8, 0 = hold
   bool canOk;        // torque/request frames arriving
   bool speedsValid;  // input and output speed sensors working
   bool driverFault;  // a pressure valve driver reported a fault
   bool configError;  // valve map invalid
   bool faultClear;   // request to leave TCU_SAFE (only at standstill)
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
   int ChooseTarget(const TcuInputs &in) const;
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
};

#endif // TCU_H
