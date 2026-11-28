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
/* Main file of the 8HP TCU firmware.
 *
 * Board: Bob's Stm32_TCU_V5 (pin map in hwdefs.h, digio_prj.h and
 * README "Pin map").
 *
 * Tasks (Stm32Scheduler on TIM4):
 *   1 ms    current setpoints to the TLE92466ED drivers, LIN schedule
 *   10 ms   inputs, shifter, TCU state machine, pressure -> current, CAN
 *   100 ms  driver faults, ATF temperature, battery voltage, CAN timeout
 * Interrupts: speed sensor edges (DMA1 channel 1 half / full buffer at
 * 2.4 kHz), UART4 (LIN bytes).
 *
 * CAN1 talks to the vehicle/VCU. CAN2 is the ZombieVerter link (PB5 /
 * PB6), on by default (Can2Enable = 1). Nothing is transmitted by
 * default (CanTxEnable = 0, COMM_LOG_ENABLE = false). SDO replies on CAN1 are only
 * sent when a tool asks for them. The LIN pump master (TJA1021) is off by
 * default (LinPumpEnable = 0).
 */
#include "canmap.h"
#include "cansdo.h"
#include "digio.h"
#include "errormessage.h"
#include "hwdefs.h"
#include "hwinit.h"
#include "param_save.h"
#include "params.h"
#include "stm32_can.h"
#include "stm32scheduler.h"
#include "terminal.h"
#include "terminalcommands.h"
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/f1/bkp.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <new>

#include "adapt.h"
#include "anaconv.h"
#include "cal_sig.h"
#include "diag.h"
#include "flashguard.h"
#include "statstream.h"
#include "torquein.h"
#include "calset.h"
#include "canif.h"
#include "drivemode.h"
#include "eds.h"
#include "linpump.h"
#include "tle92466.h"
#include "shifter.h"
#include "canlever.h"
#include "slowadc.h"
#include "speedsens.h"
#include "parksafe.h"
#include "tcu.h"
#include "valveout.h"

/// TLE92466ED pins on the STM32: two chips on SPI1, IC10 with CSN on
/// PA4 (tle_cs0) and IC11 with CSN on PE9 (tle_cs1).
/// The datasheet wants at least 250 ns between CSN low and the first
/// clock and between the last clock and CSN high, and at least 600 ns of
/// CSN high between frames (tCSN_LEAD / LAG / TD); Bob asks for at least
/// 200 ns before the other chip is selected. These waits keep 500 ns and
/// 1 us (counted on the cycle counter, 72 cycles per us).
class StmTleBus : public Tle92466Bus
{
public:
   StmTleBus() : lastHigh(0) {}
   void Select(uint8_t dev, bool active) override
   {
      if (active) {
         Wait(lastHigh, 72); // 1 us of CSN high since the last frame
         if (dev == 0)
            DigIo::tle_cs0.Clear();
         else
            DigIo::tle_cs1.Clear();
         Wait(DWT_CYCCNT, 36); // 500 ns lead
      } else {
         Wait(DWT_CYCCNT, 36); // 500 ns lag
         DigIo::tle_cs0.Set();
         DigIo::tle_cs1.Set();
         lastHigh = DWT_CYCCNT;
      }
   }
   uint16_t Transfer16(uint16_t tx) override { return tle_spi_xfer16(tx); }

private:
   static void Wait(uint32_t from, uint32_t cycles)
   {
      while (DWT_CYCCNT - from < cycles) {
      }
   }
   uint32_t lastHigh;
};

#define PRINT_JSON 0 // SDO print request sub-index for the JSON dump

/// Called if code ever calls a virtual function that has no body (a
/// "pure virtual" call, e.g. through a half-built object). It should never
/// happen; if it does, stop here so it shows up in the debugger instead of
/// running on. libopeninv's CanHardware needs this symbol and we link
/// without the C++ runtime, so we provide it, as the ZombieVerter does.
/// (GCC 10 needs it to link; newer GCC only references it weakly.)
extern "C" void __cxa_pure_virtual() {
  while (1) {
  }
}

static Stm32Scheduler *scheduler;
static CanHardware *can1;
static CanHardware *can2;
static StmTleBus tleBus;
static Tle92466 tle0(tleBus, 0); // IC10: A, B, C, D, PHOLD, PZYL
static Tle92466 tle1(tleBus, 1); // IC11: E, LINE, PREL, TCC
static ValveOut valves;
static ParkSafe parkSafe; // park-lock safety rule, see parksafe.h
static Eds eds;
static Tcu tcu;
static Shifter shifter;
static CanLever lever;
// Lever frames are copied in the CAN interrupt and decoded in the 10 ms
// task, so the lever state is only ever touched from one context.
struct LeverRxBuf
{
  uint32_t id;
  uint8_t dlc;
  uint8_t d[8];
};
static volatile uint8_t leverHead = 0, leverTail = 0;
static LeverRxBuf leverBuf[8];
static LinPump linPump;
static LinPumpRule linRule;
// Speed sensors: ISS n_t (index 0, PA0), OSS n_ab (index 1, PA1) and n6
// (index 2, PA2). All 60 ppr, 30 rpm minimum on both gearboxes [C]. n6 is an internal shaft the OEM uses
// only as a level-2 cross-check; shifting runs on n_t / n_ab (Ian 12.2).
static SpeedSensor spdIn, spdOut, spdAux;
// DMA ring of 2 halves; ScanEdges does edges and min / max levels.
static volatile uint16_t spdBuf[2 * SPEED_HALF_SCANS * SPEED_NUM];
static ScanEdges spdScan;
static AdcEdge *const spdEdge = spdScan.edge;
static uint16_t SpdLo(int i) { return spdScan.Min(i); }
static uint16_t SpdHi(int i) { return spdScan.Max(i); }
static void SpdResetLevels() { spdScan.ResetLevels(); }
static bool driversUp = false;
static bool driverFault = false;
static bool configError = false;

// Latest decoded CAN1 input (canif.h; EcuType picks the protocol).
static volatile CanRx canRx = {SEL_N, 0, 0, 0, 0, false};
static uint8_t tqCeilCounter = 0;
static volatile uint32_t msSinceRx = 0xFFFF;
// Reference-derived features (README "Torque input", "Clutch adaptation",
// "Fault detection", "Status stream").
static Adapt adapt;
static int16_t fillOfsNow[ADA_CLUTCHES]; // learned offsets at the current oil temperature, mbar
static FrozenCheck tqFrozen;
static SlipMonitor tccSlip;
static StatStream stream;
static uint8_t tqSrc = TQSRC_CAN;
static uint32_t drivenMs = 0; // time driven since power-up, ms

static const Param::PARAM_NUM chanParams[EDS_COUNT] = {
    Param::ChanA,   Param::ChanB,    Param::ChanC,    Param::ChanD,
    Param::ChanE,   Param::ChanSYS,  Param::ChanK0,   Param::ChanPOS,
    Param::ChanPZYL};

// ---------------------------------------------------------------------
// Configuration from parameters

static void ConfigureEds() {
  EdsConfig c;
  c.iMin = Param::GetInt(Param::IMinCoil) * 10;
  c.iMax = Param::GetInt(Param::IMaxCoil) * 10;
  c.slewPer10ms = Param::GetInt(Param::SlewRate) * 10;
  c.tempCorrection = Param::GetBool(Param::TempCorr);
  eds.Configure(c);
}

// LIN oil pump settings. Only on / off, the request mode, the switch-on
// speed and the pressure are parameters (LinPumpEnable, LinPumpReq,
// LinPumpRpm, LinPumpP, plus PumpFaultAct). The rest are fixed here at
// the values the parameters used to default to.
static const bool LIN_ENHANCED_CSUM = true; // enhanced checksum (LIN 2.x) [H]
static const int LIN_SLOT_MS = 10;          // one frame slot every 10 ms [I]
static const int LIN_BUS_FAULT_MS = 1800;   // no valid 0x32 this long = bus fault
static const int LIN_ARM_MS = 1000;   // "not delivering" check armed after 1 s
static const int LIN_SYM_MAX_MS = 1500; // and set after 1.5 s below LIN_DELIV_RPM
static const int LIN_DELIV_RPM = 100;   // pump speed that counts as delivering
static const bool LIN_RX_CHECK = true;  // pump replies may raise faults
static const int LIN_HYST_RPM = 50;     // on above LinPumpRpm + 50 rpm
static const int LIN_TEMP_MIN_C = 0;    // runs with the sump at 0 ..
static const int LIN_TEMP_MAX_C = 125;  // .. 125 degC
static const int LIN_P_MAX_MBAR = 20000; // and line pressure below 20 bar
static const int LIN_OFF_DELAY_MS = 800; // keeps running 800 ms after the rule drops
static const int LIN_COLD_MIN_C = -15;  // below 0 degC: may still run down to -15 degC
static const int LIN_COLD_MS = 5000;    // for at most 5 s
// Motor speed asked of the VCU when PumpFaultAct = MotorSpeed.
static const int LIN_FAULT_RPM = 600;

// Communication. Only Can1Baud, Can2Enable, Can2Baud, CanTxEnable,
// CanTimeout, CanLossTq, LeverBus and StreamEnable are parameters; the
// rest are fixed here at the values the parameters used to default to.
static const uint32_t COMM_CMD_ID = 0x300;  // VCU command frame id (768)
static const uint32_t COMM_STAT_ID = 0x301; // TCU status frame id (769); +1 = motor speed request
// On CAN loss keep the last selector and hold the gear (Damien,
// 2026-11-23) rather than going to neutral.
static const int COMM_CANLOSS_MODE = CANLOSS_HOLD;
static const int COMM_SDO_NODE_ID = 22;       // CANopen SDO node (0x600 + 22)
static const bool COMM_LOG_ENABLE = false;    // CAN2 logging frames off
static const uint32_t COMM_LOG_BASE_ID = 1520; // their first id (0x5F0) if turned on
static const bool COMM_TQ_CEIL_TX = false;    // 0x0A0 torque ceiling frame off
static const int COMM_TQ_CEIL_NM = 0;         // 0 = use the gearbox torque class
static const bool COMM_LEVER_TX = false;      // no display / LED frames to the lever
static const int COMM_LEVER_TIMEOUT_MS = 0;   // 0 = the lever type's own timeout
static const int COMM_LEVER_DEB = 2;          // lever position must repeat in 2 frames
static const int COMM_LEVER_MSUB = 1;         // lever M position sub-code
static const int COMM_LEVER_B34 = 0;          // bytes 3-4: OEM 0xFC 0xFF layout
static const bool COMM_TQ_MODEL = false;      // no engine torque model
static const int COMM_TQ_MODEL_SCALE = 100;   // model settings, used only when
static const int COMM_TQ_BOOST_PCT = 100;     // the model is on
static const int COMM_MAP_KPA = 100;
// Torque value sent when there is no reduction request.
static const int16_t COMM_TQ_NO_RED = 0;
static const int COMM_PEDAL_PCT = -1; // -1 = estimate the pedal from the torque
// CAN torque counts as frozen when it does not change for 3 s while the
// engine speed moves by more than 300 rpm.
static const int COMM_FRZ_MS = 3000;
static const int COMM_FRZ_RPM = 300;
static const uint32_t COMM_STREAM_BASE_ID = 1728; // status stream first id (0x6C0)
static const bool COMM_STREAM_ON_CAN2 = false;    // status stream goes on CAN1

// Clutch adaptation. Only AdaptEnable, AdaStep and AdaMax are parameters;
// the rest are fixed here at the values the parameters used to default to.
// A learning shift only counts when all of these hold at its start and end:
static const int ADA_TQ_MIN_NM = 55;    // engine torque 55 ..
static const int ADA_TQ_MAX_NM = 180;   // .. 180 Nm
static const int ADA_RPM_MIN = 1600;    // engine speed 1600 ..
static const int ADA_RPM_MAX = 3200;    // .. 3200 rpm
static const int ADA_OIL_MIN_C = 55;    // oil 55 ..
static const int ADA_OIL_MAX_C = 80;    // .. 80 degC (and inside a class)
static const int ADA_GEAR_MIN = 3;      // shifts between 3rd ..
static const int ADA_GEAR_MAX = 7;      // .. and 7th
static const int ADA_PEDAL_MIN_PCT = 2; // pedal above 2 %
static const int ADA_DRIVE_MIN_S = 180; // driven at least 180 s since power-up
// Learning rule: a shift longer than 500 + 100 ms, or an input-speed flare
// above 100 rpm, means the clutch filled late; shorter than 500 - 100 ms
// means harsh. Target time and band are placeholders.
static const int ADA_SHIFT_MS = 500;
static const int ADA_BAND_MS = 100;
static const int ADA_FLARE_RPM = 100;

// Copy the parameters into the TCU, shift and adaptation settings.
static void ConfigureTcu() {
  TcuConfig c = {};
  c.sas.pMax = Param::GetInt(Param::ShiftPMax);
  c.sas.syncBand = Param::GetInt(Param::SyncBand);
  c.sas.glsGainPct = Param::GetInt(Param::GlsGain);
  c.sas.rampMaxDownMs = Param::GetInt(Param::RampMaxDown);
  c.engage.pMax = Param::GetInt(Param::ShiftPMax);
  c.engage.fillEqMs = Param::GetInt(Param::EngageFillEq);
  c.engage.rampMs = Param::GetInt(Param::EngageRamp);
  c.engage.syncBand = Param::GetInt(Param::SyncBand);
  c.engage.fastCloseGrad = Param::GetInt(Param::EngageCloseGrad);
  c.engage.syncConfirmMs = Param::GetInt(Param::EngageSyncMs);
  c.engage.tqZeroNm = Param::GetInt(Param::EngageTqZero);
  c.engage.variant = Cal::Variant(Param::GetInt(Param::GearboxType)).engVar;
  c.shiftMode = Param::GetInt(Param::ShiftMode);
  c.manMaxRpm = Param::GetInt(Param::ManMaxRpm);
  c.manMinRpm = Param::GetInt(Param::ManMinRpm);
  c.applyP = Param::GetInt(Param::ClutchApplyP);
  c.lineOffset = Param::GetInt(Param::LineOffset);
  c.lineMax = Param::GetInt(Param::LineMax);
  c.engageMaxNab = Param::GetInt(Param::EngageMaxRpm);
  c.autoUpRpm = Param::GetInt(Param::AutoUpRpm);
  c.autoDownRpm = Param::GetInt(Param::AutoDownRpm);
  c.uvLimit = Param::GetInt(Param::UvLimit);
  c.gearCheckPct = Param::GetInt(Param::GearChkPct);
  c.gearCheckMinNab = Param::GetInt(Param::GearChkMinNab);
  c.gearCheckMs = Param::GetInt(Param::GearChkMs);
  c.tqRampMs = Param::GetInt(Param::TqRamp);
  c.dntTauMs = Param::GetInt(Param::DntTau);
  c.topGear = Param::GetInt(Param::TopGear);
  c.tqInterv = Param::GetInt(Param::TqInterv);
  c.canLossMode = COMM_CANLOSS_MODE;
  c.ilOpenLow = Param::GetInt(Param::IlOpenLow) * 10;
  c.ilOpenHigh = Param::GetInt(Param::IlOpenHigh) * 10;
  // Must match ConfigureEds() so the interlock sees the real targets.
  c.iMax = Param::GetInt(Param::IMaxCoil) * 10;
  c.tempCorrection = Param::GetBool(Param::TempCorr);
  c.parkRelRpm = Param::GetInt(Param::ParkRelRpm);
  c.parkRelP = Param::GetInt(Param::ParkRelP);
  c.parkRelHoldMs = Param::GetInt(Param::ParkRelHold);
  c.pnDelayMs = Param::GetInt(Param::ParkPNDelay);
  c.parkCtrl = Param::GetBool(Param::ParkCtrl);
  c.pzylI = Param::GetInt(Param::PzylCurrent) * 10;
  // Park-lock rule: hold at least PzylCurrent while moving; pull-in fixed.
  parkSafe.Configure(c.pzylI, PZYL_PULL_IN);
  c.parkMagMinMs = Param::GetInt(Param::ParkMagMin);
  c.parkMagMaxMs = Param::GetInt(Param::ParkMagMax);
  c.parkEngKmh = Param::GetInt(Param::ParkEngKmh);
  c.nabPerKmh = Param::GetInt(Param::NabPerKmh);
  c.parkEngNab = Param::GetInt(Param::ParkEngNab);
  c.parkNeedBrake = Param::GetInt(Param::ParkBrake) != PARKBRAKE_BYPASS;
  c.gearInMinP = Param::GetInt(Param::GearInMinP);
  // A pump that cannot run (disabled, F11) counts as never delivering.
  c.gearInPumpMs = Param::GetInt(Param::GearInPumpMs);
  c.gearInMotRpm = Param::GetInt(Param::GearInMotRpm);
  c.pumpFaultAct = Param::GetInt(Param::PumpFaultAct);
  c.pumpFaultRpm = LIN_FAULT_RPM;
  c.tipOverrideMs = Param::GetInt(Param::TipOverride);
  int variant = Param::GetInt(Param::GearboxType);
  Cal::Select(variant, Param::GetInt(Param::SolCurveSet));
  c.ch7Mode = Cal::Ch7Effective(Param::GetInt(Param::Ch7Mode), variant);
  c.k0P = Param::GetInt(Param::K0Press);
  c.tccP = Param::GetInt(Param::TccPress);
  c.tccOnRpm = Param::GetInt(Param::TccOnRpm);
  c.tccOffRpm = Param::GetInt(Param::TccOffRpm);
  c.tccMinGear = Param::GetInt(Param::TccMinGear);
  c.tccRamp = Param::GetInt(Param::TccRamp);
  c.tccMode = Param::GetInt(Param::TccMode);
  c.tcc.minRpm = Param::GetInt(Param::TccMinRpm);
  c.tcc.hystRpm = Param::GetInt(Param::TccHyst);
  c.tcc.kp10 = Param::GetInt(Param::TccKp);
  c.tcc.ki10 = Param::GetInt(Param::TccKi);
  c.tcc.startP = Param::GetInt(Param::TccStartP);
  c.tcc.slipTgt = Param::GetInt(Param::TccSlipTgt);
  c.tcc.slipGear = Param::GetInt(Param::TccSlipGear);
  c.tcc.slipRpm = Param::GetInt(Param::TccSlipRpm);
  // Learned fill offsets only apply with AdaptEnable on.
  c.sas.fillOfs = Param::GetBool(Param::AdaptEnable) ? fillOfsNow : nullptr;
  AdaConfig ac;
  ac.enable = Param::GetBool(Param::AdaptEnable);
  ac.tqMin = ADA_TQ_MIN_NM;
  ac.tqMax = ADA_TQ_MAX_NM;
  ac.rpmMin = ADA_RPM_MIN;
  ac.rpmMax = ADA_RPM_MAX;
  ac.oilMin = ADA_OIL_MIN_C;
  ac.oilMax = ADA_OIL_MAX_C;
  ac.gearMin = ADA_GEAR_MIN;
  ac.gearMax = ADA_GEAR_MAX;
  ac.pedalMin = ADA_PEDAL_MIN_PCT;
  ac.driveMinS = ADA_DRIVE_MIN_S;
  ac.shiftMs = ADA_SHIFT_MS;
  ac.bandMs = ADA_BAND_MS;
  ac.flareRpm = ADA_FLARE_RPM;
  ac.step = Param::GetInt(Param::AdaStep);
  ac.maxOfs = Param::GetInt(Param::AdaMax);
  adapt.Configure(ac);
  // DriveMode: Sport / Track are modifiers on this Normal set
  // (drivemode.h). Their defaults are placeholders.
  int dm = Param::GetInt(Param::DriveMode);
  DriveModeMod m = DriveModes::Neutral();
  if (dm == DM_SPORT) {
    m.upOfsRpm = Param::GetInt(Param::SportUpOfs);
    m.downOfsRpm = Param::GetInt(Param::SportDnOfs);
    m.timePct = Param::GetInt(Param::SportTime);
    m.tccOfsRpm = Param::GetInt(Param::SportTccOfs);
  } else if (dm == DM_TRACK) {
    m.downOfsRpm = Param::GetInt(Param::TrackDnOfs);
    m.timePct = Param::GetInt(Param::TrackTime);
    m.tccOfsRpm = Param::GetInt(Param::TrackTccOfs);
    m.holdToRedline = true;
    m.redlineRpm = Param::GetInt(Param::TrackRedline);
    m.liftTqNm = Param::GetInt(Param::TrackLiftNm);
  }
  Cal::SelectMode(dm);
  tcu.Configure(DriveModes::Apply(c, m));
}

// Speed sensor teeth, timeouts and ADC edge thresholds.
static void ConfigureSpeed() {
  uint16_t to = Param::GetInt(Param::SpdTimeout);
  uint16_t mn = Param::GetInt(Param::SpdMinRpm);
  spdIn.Configure(SPEED_TIMER_HZ, Param::GetInt(Param::TeethIn), to, mn);
  spdOut.Configure(SPEED_TIMER_HZ, Param::GetInt(Param::TeethOut), to, mn);
  spdAux.Configure(SPEED_TIMER_HZ, Param::GetInt(Param::TeethAux), to, mn);
  uint16_t lo = AnaConv::ToCounts(Param::GetInt(Param::SpdAdcLo));
  uint16_t hi = AnaConv::ToCounts(Param::GetInt(Param::SpdAdcHi));
  for (int i = 0; i < SPEED_NUM; i++)
    spdEdge[i].Configure(lo, hi);
}

// Shifter type and the CAN lever settings.
static void ConfigureShifter() {
  ShifterConfig c;
  // Every CAN lever type falls back to the VCU-frame selector fields
  // until its own frames are seen; Analog uses the board inputs.
  c.type = Param::GetInt(Param::ShifterType) == LEVER_ANALOG ? SHIFTER_ANALOG : SHIFTER_CAN;
  c.tip12Samples = Param::GetInt(Param::Tip12Deb) / 10;
  c.anaRMax = Param::GetInt(Param::ShAnaR);
  c.anaNMax = Param::GetInt(Param::ShAnaN);
  c.anaMin = Param::GetInt(Param::ShAnaMin);
  c.anaMax = Param::GetInt(Param::ShAnaMax);
  shifter.Configure(c);
  LeverConfig l;
  l.type = Param::GetInt(Param::ShifterType);
  l.timeoutMs = COMM_LEVER_TIMEOUT_MS;
  l.debounce = COMM_LEVER_DEB;
  l.mSub = COMM_LEVER_MSUB;
  l.b34 = COMM_LEVER_B34;
  lever.Configure(l);
}

// LIN oil pump settings (fixed values above ConfigureTcu).
static void ConfigureLin() {
  LinPumpConfig c;
  c.enhanced = LIN_ENHANCED_CSUM;
  c.slotMs = LIN_SLOT_MS;
  c.busFaultMs = LIN_BUS_FAULT_MS;
  c.diagDelayMs = 400; // D_t_DiagDelayAfterClearErr_LINIEP [C]
  c.startBlankMs = 1000; // Ian Q5: ignore status for about 1 s [I]
  c.armMs = LIN_ARM_MS;
  c.symMaxMs = LIN_SYM_MAX_MS;
  c.delivRpm = LIN_DELIV_RPM;
  c.processRx = LIN_RX_CHECK;
  linPump.Configure(c);
  LinRuleConfig r;
  r.onRpm = Param::GetInt(Param::LinPumpRpm);
  r.hystRpm = LIN_HYST_RPM;
  r.tempMin = LIN_TEMP_MIN_C;
  r.tempMax = LIN_TEMP_MAX_C;
  r.pMaxMbar = LIN_P_MAX_MBAR;
  r.offDelayMs = LIN_OFF_DELAY_MS;
  r.coldMin = LIN_COLD_MIN_C;
  r.coldMaxMs = LIN_COLD_MS;
  linRule.Configure(r);
}

// Valve-to-driver-channel map from the Chx parameters.
static void ConfigureValveMap() {
  for (int v = 0; v < EDS_COUNT; v++) {
    int ch = Param::GetInt(chanParams[v]);
    valves.SetMap((EdsValve)v, ch == CHAN_NOT_FITTED ? VALVE_NOT_FITTED : (uint8_t)ch);
  }
  valves.AttachChip(0, &tle0); // IC10
  valves.AttachChip(1, &tle1); // IC11
  // PHOLD on IC10 CH4. Validate() refuses the map if a valve ChanX
  // parameter points at it.
  valves.SetAuxMap(VALVE_AUX_PHOLD, PHOLD_CH);
}

/* TLE92466ED settings that need no tuning, so they are fixed here rather
 * than parameters (Damien: keep the parameter count down):
 *  - current-control chopping target 1 kHz, the OEM valve PWM (Ian,
 *    V5_VALVE_DEFAULTS). PROVISIONAL: bench check the ripple and the
 *    controller (PWM_CTRL_PARAM is left at 0);
 *  - open-load threshold 2/8 of the setpoint (only where MaxOpenLoad
 *    allows the chip's own check, never on PZYL / PHOLD). PROVISIONAL;
 *  - output slew 2.5 V/us, between the chip's EMC-friendly 1 V/us and
 *    its 10 V/us reset value. PROVISIONAL;
 *  - dither: 125 Hz triangle, amplitude from the fixed table in
 *    valveout.cpp (DitherFor).
 */
static TleConfig TleBase() {
  TleConfig c = TleConfig();
  c.pwmHz = 1000;
  c.olRelEighths = 2;
  c.slew = 1;
  return c;
}

/** Bring the TLE92466EDs up. Every channel stays at 0 mA until the TCU
 * asks.
 * SAFETY: this pulls EN and RESN low, which drops PZYL and PHOLD and so
 * lets the park lock in. It is only called at power-up (nothing on yet)
 * and from FaultClear at standstill, and it refuses while the park-lock
 * rule holds PZYL (parksafe.h). */
static void StartDrivers() {
  if (!parkSafe.MayRestartDrivers())
    return;
  DigIo::tle_en.Clear();
  DigIo::tle_resn.Clear();
  driversUp = false;
  ConfigureValveMap();
  configError = !valves.Validate();
  // C/D/E are only "open" above IlOpenHigh. If IMaxCoil cannot get there,
  // neutral would always look like an applied clutch to the interlock.
  if (Param::GetInt(Param::IMaxCoil) <= Param::GetInt(Param::IlOpenHigh))
    configError = true;
  if (configError) {
    ErrorMessage::Post(ERR_VALVEMAP);
    return;
  }
  // Firmware / calibration check: with CalSigExp set, the firmware must
  // carry exactly that calibration (CAL_SIG16), else nothing is driven.
  if (!Diag::CalOk(Param::GetInt(Param::CalSigExp), CAL_SIG16)) {
    configError = true;
    ErrorMessage::Post(ERR_CALSIG);
    return;
  }
  // Hold reset for 100 us, then let the chips start. Tle92466::Init waits
  // for their own start-up (INIT_DONE). EN high allows the outputs; the
  // channels stay off until Mission Mode and a setpoint.
  uint32_t t0 = DWT_CYCCNT;
  while (DWT_CYCCNT - t0 < 7200) {
  }
  DigIo::tle_resn.Set();
  DigIo::tle_en.Set();
  t0 = DWT_CYCCNT;
  while (DWT_CYCCNT - t0 < 72000) { // 1 ms
  }
  if (valves.Init(TleBase(), Param::GetBool(Param::MaxOpenLoad))) {
    driversUp = true;
    eds.Release();
    tcu.Ready();
  } else {
    // Nothing is energised yet (init failed before any setpoint), so
    // switching the chips off again cannot drop the park lock.
    ErrorMessage::Post(ERR_DRVINIT);
    DigIo::tle_en.Clear();
    DigIo::tle_resn.Clear();
    driverFault = true;
  }
}

// ---------------------------------------------------------------------
// Interrupts

/// Speed sensors: DMA1 channel 1 has filled one half of spdBuf
/// (SPEED_HALF_SCANS scans of ISS, OSS, n6) while the other half fills.
/// 416 us per half; ScanEdges takes about 70-80 us of it (README "CPU
/// load"; measured on the board as spdLoad).
/// Both flags set at once means a half was missed: process both, oldest
/// first is not known, so the periods across that gap are off once.
static volatile uint32_t spdIsrCycles = 0; // CPU cycles in this ISR (spdLoad)
extern "C" void dma1_channel1_isr(void) {
  static SpeedSensor *const sens[3] = {&spdIn, &spdOut, &spdAux};
  uint32_t t0 = DWT_CYCCNT;
  uint32_t isr = DMA_ISR(SPEED_DMA);
  if (isr & DMA_ISR_HTIF(SPEED_DMA_CH)) {
    DMA_IFCR(SPEED_DMA) = DMA_IFCR_CHTIF(SPEED_DMA_CH);
    spdScan.Block(spdBuf, SPEED_HALF_SCANS, SPEED_NUM, sens);
  }
  if (isr & DMA_ISR_TCIF(SPEED_DMA_CH)) {
    DMA_IFCR(SPEED_DMA) = DMA_IFCR_CTCIF(SPEED_DMA_CH);
    spdScan.Block(spdBuf + SPEED_HALF_SCANS * SPEED_NUM, SPEED_HALF_SCANS, SPEED_NUM, sens);
  }
  if (isr & DMA_ISR_TEIF(SPEED_DMA_CH))
    DMA_IFCR(SPEED_DMA) = DMA_IFCR_CTEIF(SPEED_DMA_CH);
  spdIsrCycles += DWT_CYCCNT - t0;
}

// ---------------------------------------------------------------------
// LIN master (aux oil pump) on UART4. The UART hears its own bytes, so
// the read-back buffer holds the header (and for 0x30 our data), then the
// pump's response. linpump.h has the protocol; this is just byte moving.
// Which SH725x LIN channel the OEM uses (and so whether its hardware does
// the checksum) was not found; the checksum type is a parameter.

// Read-back is evaluated 9 ms after the header starts: the OEM worst-case
// 8-byte frame time is 9.1 ms [C], and the next slot starts at 10 ms.
#define LIN_FRAME_MS 9 // ms

static uint8_t linTx[11];
static volatile uint8_t linTxN = 0, linTxPos = 0;
static volatile uint8_t linRx[24];
static volatile uint8_t linRxN = 0;
static uint8_t linId = 0;
static bool linPublish = false;
static uint8_t linBusyMs = 0;
static bool linActive = false; // bus on: enabled and the gearbox has a pump
static bool linAwake = false;  // U7 SLP_N high and the wake-up wait is over
static bool linWanted = false; // operating point 55
static int32_t linOilC = 40;

// Waking the TJA1021 LIN transceiver (U7). Its SLP_N pin (PD11, lin_nslp)
// has a pull-down inside the chip and no pull-up on the board, so U7 starts
// up asleep and only listens once we drive SLP_N high. Times are from the
// NXP TJA1021 data sheet, Rev. 9 (10 Feb 2023), Table 8:
//  - tgotonorm (SLP_N high until Normal mode): 2 / 5 / 10 us min / typ / max.
//  - tinit(norm) (Normal mode until transmit and receive work): 5 to 20 us.
//  - tgotosleep (SLP_N low until Sleep mode): 2 / 5 / 10 us.
// So the transmitter is ready at most 30 us after SLP_N goes high. We wait
// 100 us (more than 3 times that, and still under two LIN bits at 19200
// baud) before any break goes out.
// After a local wake-up (WAKE_N) the chip pulls TXD low as its "woken
// locally" flag (section 7.8), against R38, the 2k7 pull-up on TXD (RXD
// has a 2k7 pull-up too). A rising edge on SLP_N clears that flag, so every
// wake gives SLP_N a short low pulse and then high again. The pulse is 5
// us: inside the 2..10 us tgotosleep window, so the chip may or may not
// start to sleep, but either way the rising edge clears the flag and the
// 100 us wait that follows covers its return to Normal mode. The bus is
// idle at this point, so a few microseconds of sleep do not matter.
// This wake-up sequence only runs with GearboxType = 0 (8HP75_330e), the
// one gearbox with the LIN pump. Every other selection keeps the plain
// SLP_N handling it always had.
#define LIN_WAKE_WAIT_US 100 // us, after SLP_N goes high
#define LIN_FLAG_PULSE_US 5  // us, SLP_N low to clear the wake flag

// Busy-wait a few microseconds on the cycle counter (72 MHz).
static void WaitUs(uint32_t us) {
  uint32_t t0 = DWT_CYCCNT;
  while (DWT_CYCCNT - t0 < us * 72) {
  }
}

// Wake U7 and make it ready to transmit: SLP_N high, wait, short low pulse
// to clear the local wake flag on TXD, high again, wait. Takes about 205
// us. No LIN break may go out until this returns.
static void LinWake() {
  DigIo::lin_nslp.Set();
  WaitUs(LIN_WAKE_WAIT_US);
  DigIo::lin_nslp.Clear();
  WaitUs(LIN_FLAG_PULSE_US);
  DigIo::lin_nslp.Set();
  WaitUs(LIN_WAKE_WAIT_US);
  linAwake = true;
}

// True when the menu has the 330e selected (GearboxType 0).
static bool Lin330e() {
  return Param::GetInt(Param::GearboxType) == CALVAR_330E;
}

// LIN UART interrupt: collect the bytes of the running slot.
extern "C" void uart4_isr(void) {
  uint32_t sr = USART_SR(LIN_USART);
  if (sr & USART_SR_LBD)
    USART_SR(LIN_USART) &= ~USART_SR_LBD;
  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE)) {
    uint8_t b = USART_DR(LIN_USART) & 0xFF; // SR then DR read clears errors
    if (linRxN < sizeof(linRx))
      linRx[linRxN++] = b;
  }
  if ((USART_CR1(LIN_USART) & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
    if (linTxPos < linTxN)
      USART_DR(LIN_USART) = linTx[linTxPos++];
    else
      usart_disable_tx_interrupt(LIN_USART);
  }
}

// Start one LIN slot: break, header and (for our frames) data.
static void LinStart(const LinSlot &s) {
  uint8_t pid = Lin::Pid(s.id);
  uint8_t n = 0;
  linTx[n++] = 0x55;
  linTx[n++] = pid;
  if (s.publish) {
    for (int i = 0; i < 8; i++)
      linTx[n++] = s.data[i];
    linTx[n++] = Lin::Checksum(LIN_ENHANCED_CSUM, pid,
                               s.data, 8);
  }
  linId = s.id;
  linPublish = s.publish;
  linRxN = 0;
  linTxPos = 0;
  linTxN = n;
  linBusyMs = 1;
  USART_CR1(LIN_USART) |= USART_CR1_SBK; // break first, then the bytes
  usart_enable_tx_interrupt(LIN_USART);
}

// End of a LIN slot: hand the read-back to the pump decoder.
static void LinFinish() {
  usart_disable_tx_interrupt(LIN_USART);
  const uint8_t *rx = (const uint8_t *)linRx;
  int n = linRxN;
  int i = Lin::FindData(rx, n, Lin::Pid(linId));
  bool full = i >= 0 && n - i >= 9;
  if (linPublish) {
    // Our own data must read back unchanged (bus short / collision check).
    bool ok = full;
    for (int k = 0; ok && k < 9; k++)
      ok = rx[i + k] == linTx[2 + k];
    if (!ok)
      linPump.OnTxError();
  } else if (full) {
    linPump.OnResponse(linId, rx + i, 9);
  }
  // No response: the pump state machine times out on its own.
}

// LIN schedule step, every 1 ms.
static void LinTick1ms() {
  if (linBusyMs) {
    if (++linBusyMs > LIN_FRAME_MS) {
      LinFinish();
      linBusyMs = 0;
    }
  }
  LinSlot slot;
  bool send = linPump.Tick1ms(linActive, linWanted, linOilC, &slot);
  if (!Lin330e()) {
    // Not the 330e: exactly as before, SLP_N follows the pump state.
    if (send && !linBusyMs)
      LinStart(slot); // a slot that falls into a busy frame is skipped
    if (linPump.TransceiverAwake())
      DigIo::lin_nslp.Set();
    else
      DigIo::lin_nslp.Clear();
    linAwake = false; // so a later switch to the 330e wakes it properly
    return;
  }
  // 330e: set the transceiver before any break. Wake it (with the waits)
  // when the bus starts, put it to sleep when the bus stops.
  if (linPump.TransceiverAwake()) {
    if (!linAwake)
      LinWake();
  } else {
    DigIo::lin_nslp.Clear();
    linAwake = false;
  }
  if (send && linAwake && !linBusyMs)
    LinStart(slot); // a slot that falls into a busy frame is skipped
}

// The scheduler object is made after the interrupts are switched on, so
// ignore a timer interrupt that comes before it exists.
extern "C" void tim4_isr(void) {
  if (scheduler)
    scheduler->Run();
}

// ---------------------------------------------------------------------
// CAN

/// Queue a lever frame from the CAN interrupt (dropped if the queue is full).
static bool LeverRx(int bus, uint32_t id, const uint32_t data[2], uint8_t dlc) {
  uint32_t ids[2];
  int n = lever.IsCan() && Param::GetInt(Param::LeverBus) == bus ? lever.RxIds(ids) : 0;
  for (int i = 0; i < n; i++) {
    if (ids[i] != id)
      continue;
    uint8_t next = (uint8_t)((leverHead + 1) & 7);
    if (next != leverTail) {
      LeverRxBuf &b = leverBuf[leverHead];
      b.id = id;
      b.dlc = dlc > 8 ? 8 : dlc;
      const uint8_t *src = (const uint8_t *)data;
      for (int k = 0; k < 8; k++)
        b.d[k] = src[k];
      leverHead = next;
    }
    return true;
  }
  return false;
}

// CAN2 receive: only lever frames are taken here.
static bool Can2Callback(uint32_t id, uint32_t data[2], uint8_t dlc) {
  return LeverRx(1, id, data, dlc);
}

// CAN2 receive filters (the lever ids when it is on CAN2).
static void SetCan2Filters() {
  uint32_t ids[2];
  int n = lever.IsCan() && Param::GetInt(Param::LeverBus) == 1 ? lever.RxIds(ids) : 0;
  for (int i = 0; i < n; i++)
    can2->RegisterUserMessage(ids[i]);
}

// CAN1 receive: lever frames, then the VCU / ECU command frames.
static bool CanCallback(uint32_t id, uint32_t data[2], uint8_t dlc) {
  if (LeverRx(0, id, data, dlc))
    return true;
  CanRx rx;
  if (!CanIf::Rx(Param::GetInt(Param::EcuType), COMM_CMD_ID,
                 id, (const uint8_t *)data, dlc, &rx))
    return false;
  canRx.selector = rx.selector;
  canRx.gearReq = rx.gearReq;
  canRx.torque = rx.torque;
  canRx.motorRpm = rx.motorRpm;
  canRx.tips = rx.tips;
  msSinceRx = 0;
  return true;
}

// CAN1 receive filters for the EcuType and the lever.
static void SetCanFilters() {
  uint32_t ids[CANIF_MAX_RX_IDS];
  int n = CanIf::RxIds(Param::GetInt(Param::EcuType),
                       COMM_CMD_ID, ids);
  for (int i = 0; i < n; i++)
    can1->RegisterUserMessage(ids[i]);
  uint32_t lids[2];
  int nl = lever.IsCan() && Param::GetInt(Param::LeverBus) == 0 ? lever.RxIds(lids) : 0;
  for (int i = 0; i < nl; i++)
    can1->RegisterUserMessage(lids[i]);
}

/// Status frames for the selected EcuType (canif.cpp) and the optional
/// 0x0A0 torque ceiling (COMM_TQ_CEIL_TX, off). Only called with
/// CanTxEnable on.
static void SendStatus(const TcuOutputs &o, int32_t temp) {
  CanTxIn in = {&o, temp, linPump.Classes(),
                COMM_STAT_ID, COMM_TQ_NO_RED};
  CanFrame f[CANIF_MAX_TX];
  int n = CanIf::Tx(Param::GetInt(Param::EcuType), in, f);
  for (int i = 0; i < n; i++)
    can1->Send(f[i].id, f[i].data, f[i].dlc);
  if (COMM_TQ_CEIL_TX) {
    uint8_t d[8];
    int32_t ceil = COMM_TQ_CEIL_NM;
    if (ceil <= 0)
      ceil = Cal::Active().size->torqueClassNm; // [I] size data
    CanIf::PackTqCeiling(ceil, tqCeilCounter, d);
    tqCeilCounter = (uint8_t)((tqCeilCounter + 1) % 15);
    can1->Send(CANIF_TQ_CEIL_ID, d, 8);
  }
}

// Put a 16-bit value at word i of a log frame, little-endian.
static void Put16(uint8_t *b, int i, int32_t v) {
  b[2 * i] = v & 0xFF;
  b[2 * i + 1] = (v >> 8) & 0xFF;
}

/// CAN2 logging frames, every 10 ms. Layout in README.
static void SendLog(const TcuOutputs &o, const TcuInputs &in) {
  uint32_t base = COMM_LOG_BASE_ID;
  uint8_t b[8];
  Put16(b, 0, o.p[CL_A]); Put16(b, 1, o.p[CL_B]);
  Put16(b, 2, o.p[CL_C]); Put16(b, 3, o.p[CL_D]);
  can2->Send(base + 0, b, 8);
  Put16(b, 0, o.p[CL_E]); Put16(b, 1, o.pSys);
  Put16(b, 2, in.n_t); Put16(b, 3, in.n_ab);
  can2->Send(base + 1, b, 8);
  for (int k = 0; k < 2; k++) {
    for (int i = 0; i < 4; i++)
      Put16(b, i, eds.Output((EdsValve)(k * 4 + i)) / 10);
    can2->Send(base + 2 + k, b, 8);
  }
  b[0] = o.mode; b[1] = o.gear; b[2] = o.target; b[3] = o.zKzu;
  b[4] = o.zKab; b[5] = o.engPhase; b[6] = o.interlock; b[7] = o.fault;
  can2->Send(base + 4, b, 8);
  Put16(b, 0, in.m_t); Put16(b, 1, o.tqDelta);
  Put16(b, 2, o.tSyn); Put16(b, 3, o.gls);
  can2->Send(base + 5, b, 8);
}

// ---------------------------------------------------------------------
// Adaptation storage: its own flash page (ADAPT_BLKNUM), apart from the
// parameters and CAN map. Saved on the terminal command "adaptsave" and
// automatically when something was learned and the car stands in P / N
// (a page erase stalls the CPU for ~20 ms). "adaptreset" clears it.

static uint32_t AdaptAddr() {
  return FLASH_BASE + desig_get_flash_size() * 1024 - ADAPT_BLKNUM * FLASH_PAGE_SIZE;
}

// ---------------------------------------------------------------------
// Start-up checks

/* Start-up stage, kept in backup register 1, which survives a watchdog
 * reset. If the watchdog restarts the board, the next start blinks the
 * LED once per stage reached (see main), so a board that never gets as
 * far as the terminal still shows where it stopped. 15 = running.
 */
enum BootStages {
  BOOT_CLOCKS = 1, BOOT_DIGIO, BOOT_SPEED_ADC, BOOT_SLOW_ADC, BOOT_PARAMS,
  BOOT_ADAPT, BOOT_SPI_LIN, BOOT_PARAM_CHANGE, BOOT_NVIC, BOOT_CAN1,
  BOOT_CAN2, BOOT_CANMAP, BOOT_DRIVERS, BOOT_SCHEDULER, BOOT_RUNNING
};

static void BootStage(int stage) {
  BKP_DR1 = (uint32_t)stage;
  iwdg_reset(); // start-up is short, but keep the watchdog fed while in it
}

// Busy wait in ms on the cycle counter (72 MHz), feeding the watchdog.
static void WaitMs(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    uint32_t t0 = DWT_CYCCNT;
    while (DWT_CYCCNT - t0 < 72000) {
    }
    iwdg_reset();
  }
}

/* After a watchdog reset, blink the stage the last start got to: that
 * many short flashes (200 ms on, 300 ms off), then 1 s dark. The stage
 * is then shown as bootStage. A normal power-up does not blink. 1 = clock
 * and RTC set-up, 2 = pins, 3 = speed ADC, 4 = slow ADC, 5 = parameter
 * load, 6 = adaptation load, 7 = SPI and LIN UART, 8 = applying the
 * parameters, 9 = interrupts, 10 = terminal and CAN1, 11 = CAN2,
 * 12 = CAN map, 13 = valve drivers, 14 = scheduler, 15 = running.
 */
static int ShowLastBootStage(uint32_t prevStage) {
  int last = (int)(prevStage & 0xFF);
  bool wdg = (RCC_CSR & RCC_CSR_IWDGRSTF) != 0;
  RCC_CSR |= RCC_CSR_RMVF; // clear the reset flags for next time
  if (!wdg)
    return 0;
  if (last < 1 || last > BOOT_RUNNING)
    last = 0;
  for (int i = 0; i < last; i++) {
    DigIo::led_out.Clear(); // active low: on
    WaitMs(200);
    DigIo::led_out.Set();
    WaitMs(300);
  }
  WaitMs(1000);
  return last;
}

// One of our parameter ids, or one this firmware used before (retired):
// a page we saved with an older build still counts as ours, and
// parm_load() simply skips the retired entries.
static bool OurParamId(uint16_t id) {
  Param::PARAM_NUM i = Param::NumFromId(id);
  if (i != Param::PARAM_INVALID && Param::GetType(i) == Param::TYPE_PARAM)
    return true;
  return FlashGuard::Retired(id);
}

/* Load the saved parameters, unless the page was written by other firmware
 * (see flashguard.h: a ZombieVerter page has a valid CRC but its ids mean
 * other things here). Every loaded value is then checked against its
 * min / max, and one outside goes back to its default, so no stored value
 * can index past a table. Returns the paramSrc value: 0 defaults (blank
 * page or bad CRC), 1 loaded, 2 foreign page ignored.
 */
static int LoadParams() {
  const FlashGuard::StoredParam *page = (const FlashGuard::StoredParam *)(
      FLASH_BASE + desig_get_flash_size() * 1024 - PARAM_BLKNUM * PARAM_BLKSIZE);
  const int entries = (PARAM_BLKSIZE - 8) / sizeof(FlashGuard::StoredParam);
  if (FlashGuard::UnknownKeys(page, entries, OurParamId) > 0)
    return 2;
  if (parm_load() != 0)
    return 0;
  for (int i = 0; i < Param::PARAM_LAST; i++) {
    Param::PARAM_NUM p = (Param::PARAM_NUM)i;
    if (Param::GetType(p) != Param::TYPE_PARAM)
      continue;
    const Param::Attributes *a = Param::GetAttrib(p);
    Param::SetFixed(p, FlashGuard::InRange(Param::Get(p), a->min, a->max, a->def));
  }
  return 1;
}

// Read the adaptation page at start-up (all zero if blank or bad).
static void AdaptLoad() {
  AdaImage img;
  const uint32_t *src = (const uint32_t *)AdaptAddr();
  uint32_t *dst = (uint32_t *)&img;
  for (uint32_t i = 0; i < sizeof(img) / 4; i++)
    dst[i] = src[i];
  adapt.Unpack(img); // blank or corrupt page: all offsets 0
}

// Write the adaptation page (erase + program, ~20 ms with the CPU stalled).
void AdaptSave() {
  AdaImage img;
  adapt.Pack(&img);
  const uint32_t *w = (const uint32_t *)&img;
  uint32_t addr = AdaptAddr();
  flash_unlock();
  flash_erase_page(addr);
  for (uint32_t i = 0; i < sizeof(img) / 4; i++)
    flash_program_word(addr + i * 4, w[i]);
  flash_lock();
}

// Clear the learned offsets and store the cleared page.
void AdaptReset() {
  adapt.Reset();
  AdaptSave();
}

// ---------------------------------------------------------------------
// Tasks

static void Ms1Task() {
  LinTick1ms();
  // Always sent while the drivers are up: in the safe state the valves are
  // at 0 mA but PZYL may still be held (parksafe.h).
  if (driversUp)
    valves.Update(eds);
  if (msSinceRx < 0xFFFF)
    msSinceRx++;
}

static int32_t atfTemp = 40;
static int32_t vbatmV = 12000;
static OnTimeLimit parkMagLimit; // park magnet: at most 1 s per energise
static Debounce brakeDeb; // 12 V brake input (PD1)
static bool parkMagOn = false;   // park magnet output as driven

// Main 10 ms task: inputs, TCU, outputs, CAN and spot values.
static void Ms10Task() {
  SlowAdc::Tick10ms();
  spdIn.Tick10ms();
  spdOut.Tick10ms();

  bool fromParam = Param::GetInt(Param::CmdSource) == 1;
  bool canOk = msSinceRx < (uint32_t)Param::GetInt(Param::CanTimeout);

  // Shifter: CAN fields of the command frame, or the board inputs.
  ShifterRaw raw;
  raw.canSelector = canOk ? canRx.selector : (uint8_t)SEL_N;
  raw.canTipUp = canOk && (canRx.tips & 1);
  raw.canTipDown = canOk && (canRx.tips & 2);
  raw.up12 = DigIo::shift_up.Get();
  raw.down12 = DigIo::shift_dn.Get();
  raw.din1 = DigIo::din1.Get();
  raw.din2 = DigIo::din2.Get();
  raw.parkIn = DigIo::park_in.Get();
  raw.notParkIn = DigIo::notpark_in.Get();
  raw.anaMv = 2 * AnaConv::ToMv(SlowAdc::Get(SlowAdc::ANA)); // /2 on the board
  shifter.Update(raw);
  bool canShifter = Param::GetInt(Param::ShifterType) != LEVER_ANALOG;

  TcuInputs in = {};
  if (fromParam)
    in.selector = Param::GetInt(Param::SelectorSet);
  else if (canShifter && !canOk)
    in.selector = canRx.selector; // last frame; CAN-loss rules below
  else
    in.selector = shifter.Selector();
  in.tipUp = !fromParam && shifter.TipUp();
  in.tipDown = !fromParam && shifter.TipDown();
  // CAN gear lever (ShifterType 0, 2..5): it owns the selector once its
  // frames have been seen. A new R / D / P is only taken at or below the
  // gear-in output speed and with no TCU fault. If the lever goes quiet
  // the CAN loss rule (COMM_CANLOSS_MODE) applies: hold the last position or go to N.
  if (lever.IsCan()) {
    while (leverTail != leverHead) {
      const LeverRxBuf &b = leverBuf[leverTail];
      lever.OnFrame(b.id, b.d, b.dlc);
      leverTail = (uint8_t)((leverTail + 1) & 7);
    }
    bool engageOk = spdOut.Rpm() <= Param::GetInt(Param::EngageMaxRpm) &&
                    tcu.Out().mode != TCU_SAFE;
    lever.Tick10ms(engageOk);
    if (!fromParam && lever.Seen()) {
      if (lever.Valid()) {
        in.selector = lever.Selector();
        in.tipUp = in.tipUp || lever.TipUp();
        in.tipDown = in.tipDown || lever.TipDown();
      } else {
        in.selector = COMM_CANLOSS_MODE == CANLOSS_HOLD
                          ? lever.Selector() : (uint8_t)SEL_N;
        in.tipUp = in.tipDown = false;
      }
    }
  }
  in.gearReq = fromParam ? Param::GetInt(Param::GearSet) : canRx.gearReq;
  in.n_mot = fromParam ? Param::GetInt(Param::MotorRpmSet) : canRx.motorRpm;
  // Input torque (torquein.h): CAN value unless it is a "not available"
  // code (0x8000 / 0x7FFF) or frozen while the engine speed moves; then
  // the optional model, else the safe substitute CanLossTq (high torque
  // = high clamping pressure).
  {
    int16_t canTq = canRx.torque;
    int32_t rpm = in.n_mot < 0 ? -in.n_mot : in.n_mot;
    bool frozen = canOk && tqFrozen.Tick(canTq, rpm, COMM_FRZ_MS, COMM_FRZ_RPM);
    TorqueModelIn tm = {rpm, COMM_MAP_KPA, COMM_TQ_MODEL_SCALE, COMM_TQ_BOOST_PCT};
    if (fromParam) {
      in.m_t = Param::GetInt(Param::TorqueSet);
      tqSrc = TQSRC_PARAM;
    } else {
      tqSrc = TorqueIn::Select(canTq, TorqueIn::RawValid(canTq) && !frozen,
                               COMM_TQ_MODEL, tm,
                               Param::GetInt(Param::CanLossTq), &in.m_t);
    }
    Param::SetInt(Param::tqFrozen, frozen);
    if (frozen)
      ErrorMessage::Post(ERR_TQFROZEN);
  }
  // ManualGear (Manual mode): the dropdown picks P / R / N or a gear in
  // D. The TCU still does the gear-in, the shift limits, the input-speed
  // checks and the interlock; the CAN-loss rules below still win.
  int mg = Param::GetInt(Param::ManualGear);
  if (Param::GetInt(Param::ShiftMode) == SHIFT_MANUAL && mg > 0) {
    static const uint8_t mgSel[4] = {SEL_N, SEL_P, SEL_R, SEL_N};
    in.selector = mg <= 3 ? mgSel[mg] : (uint8_t)SEL_D;
    in.manualGear = mg >= 4 ? (uint8_t)(mg - 3) : 0;
    in.tipUp = in.tipDown = false;
  }
  if (!fromParam && !canOk) {
    // In plain words: assume high torque so line pressure stays high and
    // the clutches cannot slip whatever the engine does.
    // No VCU. COMM_CANLOSS_MODE = hold (Damien 2026-11-23): keep the last
    // selector so the TCU holds the gear (no upshifts, LineMax, step down
    // as the car slows). The other mode would go to neutral. Either way assume
    // high torque for line pressure: CanLossTq defaults to the OEM
    // substitute D_m_IstVM_Default = 600 Nm (@0x218144) [C]. The OEM
    // fault-to-reaction table (KFC -> KMN) was not found, so what exactly
    // the OEM does on KFCx_ENGSIGTRQ is [H]; hold-gear is Ian's reading
    // of the limp states [I] (QA2 answer 8).
    if (COMM_CANLOSS_MODE != CANLOSS_HOLD)
      in.selector = SEL_N;
    in.gearReq = 0;
    in.tipUp = in.tipDown = false;
    in.m_t = Param::GetInt(Param::CanLossTq);
    tqSrc = TQSRC_SUBSTITUTE; // no engine speed for the model either
  }
  in.canOk = canOk || fromParam;
  in.n_t = Param::GetInt(Param::NtSource) == 1 ? in.n_mot : spdIn.Rpm();
  in.n_ab = spdOut.Rpm();
  if (in.n_mot < 0)
    in.n_mot = -in.n_mot;
  in.temp = atfTemp;
  in.vbat = vbatmV;
  in.speedsValid = spdOut.Valid() || in.n_ab == 0;
  in.driverFault = driverFault;
  in.configError = configError;
  in.faultClear = false;
  in.pMeas = -1; // no line pressure sensor on the board
  // Pedal: COMM_PEDAL_PCT, or -1 = estimated from the input torque as a share
  // of the gearbox torque class (the VCU frame carries no pedal).
  {
    int32_t pp = COMM_PEDAL_PCT;
    if (pp < 0) {
      int32_t cls = Cal::Active().size->torqueClassNm;
      pp = cls > 0 ? in.m_t * 100 / cls : 0;
      pp = pp < 0 ? 0 : (pp > 100 ? 100 : pp);
    }
    in.pedal = pp;
  }
  in.tccFault = tccSlip.Fault();
  // Footbrake for the park release (ParkBrake): the VCU frame (lost with
  // CAN), the BrakeIn parameter, the 12 V brake input (PD1; debounced
  // over 3 samples = 30 ms), or bypassed on the bench (then the TCU does
  // not ask for it at all, see ConfigureTcu).
  bool brakePin = brakeDeb.Tick(DigIo::brake_in.Get(), 3);
  Param::SetInt(Param::brakePin, brakePin);
  in.brake = FootbrakeFrom(Param::GetInt(Param::ParkBrake), canOk, canRx.brake,
                           Param::GetBool(Param::BrakeIn), true /* pin fitted */, brakePin);
  // LIN pump: delivering time and faults (diagnosis blanked below the
  // load-off voltage, as the OEM does [C]).
  bool uv = Param::GetInt(Param::UvLimit) > 0 && vbatmV < Param::GetInt(Param::UvLimit);
  in.pumpDeliverMs = linActive ? (int32_t)linPump.DeliveringMs() : 0;
  in.pumpFault = linActive && !uv && linPump.Classes() != 0;
  if (Param::GetBool(Param::FaultClear)) {
    Param::SetInt(Param::FaultClear, 0);
    linPump.ClearErrors(); // one 0x30 with the clear flag
    // Only at standstill in P or N: restart the chips, then let the TCU
    // leave the safe state if nothing is still wrong.
    // Never while the park-lock rule holds PZYL (restarting the chips
    // pulls EN low, parksafe.h).
    if (tcu.Out().mode == TCU_SAFE && in.n_ab == 0 && parkSafe.MayRestartDrivers() &&
        (in.selector == SEL_P || in.selector == SEL_N)) {
      driverFault = false;
      tccSlip.Clear();
      StartDrivers();
      in.driverFault = driverFault;
      in.configError = configError;
      in.faultClear = true;
    }
  }

  /* Selector D with the VCU still holding a CAN-lost "N" is handled above.
   * On a CAN loss while driving the TCU goes to neutral; the VCU must not
   * expect drive torque without its own frames being seen.
   */
  tcu.Tick10ms(in);
  const TcuOutputs &o = tcu.Out();

  // TCC lock-up slip (off until TccSlipMax is set: OEM limit not traced).
  bool slipWas = tccSlip.Fault();
  tccSlip.Tick10ms(o.tccLocked, in.n_mot, in.n_t, Param::GetInt(Param::TccSlipMax),
                   Param::GetInt(Param::TccSlipMs));
  if (tccSlip.Fault() && !slipWas)
    ErrorMessage::Post(ERR_TCCSLIP);

  // Clutch adaptation: watch the learning shifts, then refresh the
  // offsets the shift code adds (only used with AdaptEnable on).
  if ((o.mode == TCU_DRIVE || o.mode == TCU_SHIFT) && in.n_ab > 0 && drivenMs < 0xF0000000u)
    drivenMs += 10;
  {
    AdaInputs ai;
    ai.mode = o.mode;
    ai.gear = o.gear;
    ai.target = o.target;
    ai.m_t = in.m_t;
    ai.n_t = in.n_t;
    ai.n_mot = in.n_mot;
    ai.n_ab = in.n_ab;
    ai.temp = atfTemp;
    ai.pedal = in.pedal;
    ai.drivenS = (int32_t)(drivenMs / 1000);
    ai.normalMode = Param::GetInt(Param::DriveMode) == DM_NORMAL &&
                    Param::GetInt(Param::ShiftMode) != SHIFT_MANUAL;
    ai.faultFree = o.fault == TF_NONE && !driverFault && !configError &&
                   tqSrc == TQSRC_CAN && !tccSlip.Fault();
    adapt.Tick10ms(ai);
    for (int c = 0; c < ADA_CLUTCHES; c++)
      fillOfsNow[c] = (int16_t)adapt.Offset(c, atfTemp);
  }

  // Aux oil pump (330e run rule, Ian's 330E_LIN_OIL_PUMP.md section 7).
  // The F11 8HP45 has no LIN pump [C], so the bus stays off for it.
  // The OEM puts LIN to sleep with the charging plug in
  // (CDD_LINBusStartStop.st_chargingPlug [C]). There is no plug input or
  // parameter here, so the plug always counts as out.
  linActive = Param::GetBool(Param::LinPumpEnable) &&
              Cal::Variant(Param::GetInt(Param::GearboxType)).linPump;
  linOilC = atfTemp;
  linWanted = linActive && linRule.Update(Param::GetInt(Param::LinPumpReq),
                                          in.n_t, atfTemp, o.pSys);

  // Park-hold magnet PHOLD (IC10 CH4, 24-26 ohm, PHOLD_CURRENT): driven
  // by the park sequence when ParkCtrl is on (Ian 12.1, 100 ms .. 1 s
  // after a release), plus the bench test at standstill in P or N. Either
  // way one energise lasts at most 1 s (PARK_MAG_LIMIT_MS); the bench test
  // has to be switched off and on again for another pulse.
  bool standstillPN = in.n_ab == 0 && (o.mode == TCU_PARK || o.mode == TCU_NEUTRAL);
  bool magReq = !o.safeOutputs &&
                (o.parkMagnet || (Param::GetBool(Param::ParkHoldTest) && standstillPN));
  parkMagOn = parkMagLimit.Tick(magReq, PARK_MAG_LIMIT_MS, 10);
  valves.SetAux(VALVE_AUX_PHOLD, parkMagOn ? PHOLD_CURRENT : 0);
  // Spare outputs OUT1 / OUT2 (NCV8402 switches): StarterLock = on while
  // the TCU is in P or N [H: the polarity the starter interlock wants is
  // not known].
  const Param::PARAM_NUM gpFunc[2] = {Param::GpOut1Func, Param::GpOut2Func};
  for (int i = 0; i < 2; i++) {
    int f = Param::GetInt(gpFunc[i]);
    bool on = f == 1 || (f == 2 && (o.mode == TCU_PARK || o.mode == TCU_NEUTRAL));
    DigIo &pin = i == 0 ? DigIo::out1 : DigIo::out2;
    if (on)
      pin.Set();
    else
      pin.Clear();
  }
  // OUT3: TIM3 PWM at GpPwmFreq (set at start-up), GpPwmDuty %.
  gp_pwm_duty((uint32_t)Param::GetInt(Param::GpPwmDuty));

  /* Park-lock safety rule (parksafe.h). The TCU's PZYL request goes
   * through it every 10 ms. Once PZYL is on and the output shaft turns
   * (or its speed is not known) it is held on until the car has stopped,
   * whatever the TCU, a fault or the safe state asks. "Known" = the
   * sensor sees edges, or reads 0 with healthy signal levels and the
   * input shaft not contradicting it (spdOutDiag). */
  bool spdKnown = spdOut.Valid() ||
                  (in.n_ab == 0 && Param::GetInt(Param::spdOutDiag) == SPD_OK);
  int32_t pzyl = parkSafe.Update(o.safeOutputs || !driversUp ? 0 : o.pzylI,
                                 in.n_ab, spdKnown);

  if (o.safeOutputs || !driversUp) {
    // Valves to 0 mA (6th gear, full line pressure). Never valves.AllOff()
    // here: that would also drop PZYL.
    eds.SafeState();
    eds.SetCurrent(EDS_PZYL, pzyl);
  } else {
    eds.Release();
    for (int c = 0; c < CL_COUNT; c++)
      eds.SetPressure((EdsValve)c, o.p[c], atfTemp);
    eds.SetPressure(EDS_SYS, o.pSys, atfTemp);
    // Channel 7: K0 (330e) or TCC lock-up (F11) or held open (Ch7Mode).
    // The curve is the active variant's (330e K0 / F11 TCC).
    eds.SetPressure(EDS_K0, o.pCh7, atfTemp);
    // POS: position valve, switch-over at 420 mA (I_EDSPOSV [C]); when to
    // switch it is not known, so it is a plain current command.
    eds.SetCurrent(EDS_POS, Param::GetInt(Param::PosCurrent) * 10);
    // PZYL (park-lock cylinder valve MvPZyl, IC10 CH5 / J5.42): from the
    // park sequence with ParkCtrl on, else 0, through the park-lock rule
    // (pull-in pulse, held while moving).
    eds.SetCurrent(EDS_PZYL, pzyl);

    // Bench valve test: only at standstill in P or N.
    if (Param::GetBool(Param::ValveTest) && in.n_ab == 0 &&
        (o.mode == TCU_NEUTRAL || o.mode == TCU_PARK)) {
      int v = Param::GetInt(Param::ValveTestSel);
      if (v >= 0 && v < EDS_COUNT)
        eds.SetCurrent((EdsValve)v, Param::GetInt(Param::ValveTestmA) * 10);
    }
  }
  eds.Tick10ms();

  // FAULTN pulled low (either chip) gets read at once rather than at
  // 100 ms. Faults never switch PZYL off (see ValveOut and parksafe.h).
  if (driversUp && !DigIo::tle_nfault.Get()) {
    uint16_t f = valves.PollFaults();
    if (f & ValveOut::CriticalMask())
      driverFault = true;
    Param::SetInt(Param::valveFault, f);
  }

  if (Param::GetBool(Param::CanTxEnable))
    SendStatus(o, atfTemp);
  // Optional status stream (statstream.h): on CAN1 (COMM_STREAM_ON_CAN2 is
  // false), and only with CanTxEnable as well.
  if (Param::GetBool(Param::StreamEnable)) {
    bool onCan2 = COMM_STREAM_ON_CAN2;
    CanHardware *bus = onCan2 ? can2 : (Param::GetBool(Param::CanTxEnable) ? can1 : 0);
    if (bus) {
      StreamIn si;
      si.tqTarget = in.m_t + o.tqDelta;
      si.tqReductPct = StatStream::ReductPct(in.m_t, o.tqDelta);
      si.shifting = o.mode == TCU_SHIFT;
      si.coast = o.coast;
      si.tqActive = o.tqDelta != 0;
      si.inertia = o.inertia;
      si.tccClosed = o.tccLocked;
      si.slip = Gears::SlipRpm(o.gear, in.n_t, in.n_ab);
      si.nT = in.n_t;
      si.nAb = in.n_ab;
      si.nMot = in.n_mot;
      si.nAux = spdAux.Rpm();
      si.gear = o.gear;
      si.target = o.target;
      si.selector = in.selector;
      si.mode = o.mode;
      si.leverState = lever.State();
      si.driveMode = (uint8_t)Param::GetInt(Param::DriveMode);
      si.zKzu = o.zKzu;
      si.zKab = o.zKab;
      si.fault = o.fault;
      si.diagBits = (uint16_t)Param::GetInt(Param::valveFault);
      si.atfC = atfTemp;
      si.boardC = Param::GetInt(Param::tBoard);
      si.vbatMv = vbatmV;
      si.adaStatus = (uint8_t)adapt.Status();
      si.canLoss = !in.canOk;
      si.leverTimeout = lever.Seen() && !lever.Valid();
      si.tqNotCan = tqSrc != TQSRC_CAN && tqSrc != TQSRC_PARAM;
      CanFrame sf[STREAM_MAX_TX];
      int ns = stream.Tick10ms(COMM_STREAM_BASE_ID, si, sf);
      for (int i = 0; i < ns; i++)
        bus->Send(sf[i].id, sf[i].data, sf[i].dlc);
    }
  }
  // Lever display / LEDs: only with COMM_LEVER_TX (off) and CanTxEnable. Only
  // one node may own these ids (see README "CAN gear levers").
  if (lever.IsCan() && Param::GetBool(Param::CanTxEnable) &&
      COMM_LEVER_TX) {
    LeverTxIn lt = {Param::GetBool(Param::t15), in.selector, o.gear};
    CanFrame lf[LEVER_MAX_TX];
    int nl = lever.Tx10ms(lt, lf);
    CanHardware *bus = Param::GetInt(Param::LeverBus) == 1 ? can2 : can1;
    for (int i = 0; i < nl && bus; i++)
      bus->Send(lf[i].id, lf[i].data, lf[i].dlc);
  }
  Param::SetInt(Param::lvMove, lever.RawMove());
  Param::SetInt(Param::lvPBtn, lever.PButton());
  Param::SetInt(Param::lvState, lever.State());
  Param::SetInt(Param::lvCrcErr, lever.CrcErrors());
  Param::SetInt(Param::lvCntErr, lever.CounterErrors());
  Param::SetInt(Param::lvTmo, lever.Seen() && !lever.Valid());
  if (COMM_LOG_ENABLE && can2)
    SendLog(o, in);

  Param::SetInt(Param::opmode, o.mode);
  Param::SetInt(Param::gear, o.gear);
  Param::SetInt(Param::target, o.target);
  Param::SetInt(Param::selector, in.selector);
  Param::SetInt(Param::tcufault, o.fault);
  Param::SetInt(Param::interlock, o.interlock);
  Param::SetInt(Param::applied, o.applied);
  Param::SetInt(Param::zkzu, o.zKzu);
  Param::SetInt(Param::zkab, o.zKab);
  Param::SetInt(Param::engphase, o.engPhase);
  Param::SetInt(Param::nIn, in.n_t);
  Param::SetInt(Param::nOut, in.n_ab);
  // n6. The OEM cross-check needs the per-gear
  // D_k_n6 factors, which are not all known yet (TBD), so it is shown only.
  Param::SetInt(Param::nAux, spdAux.Rpm());
  Param::SetInt(Param::nMotReq, o.nMotReq);
  Param::SetInt(Param::parkWait, o.parkWait);
  Param::SetInt(Param::gearInWait, o.gearInWait);
  Param::SetInt(Param::tipAuto, o.tipAuto);
  Param::SetInt(Param::pCh7, o.pCh7);
  Param::SetInt(Param::iPZYL, eds.Setpoint(EDS_PZYL) / 10);
  Param::SetInt(Param::parkMag, parkMagOn);
  Param::SetInt(Param::parkInh, o.parkInhibit);
  Param::SetInt(Param::brake, in.brake);
  Param::SetInt(Param::parkBrkW, o.parkBrakeWait);
  {
    const VariantCal &v = Cal::Active();
    Param::SetInt(Param::solSetAct, Cal::ActiveSolSet());
    Param::SetInt(Param::tqClass, v.size->torqueClassNm);
    Param::SetInt(Param::revLimNm, v.size->revLimitNm);
    Param::SetInt(Param::safeOfsNm, v.size->safetyOfsNm);
    Param::SetInt(Param::k0Present, v.k0Present);
  }
  Param::SetInt(Param::ch7Act, Cal::Ch7Effective(Param::GetInt(Param::Ch7Mode),
                                                 Param::GetInt(Param::GearboxType)));
  Param::SetInt(Param::t15, DigIo::t15_in.Get());
  Param::SetInt(Param::nMot, in.n_mot);
  Param::SetInt(Param::mTq, in.m_t);
  Param::SetInt(Param::tqDelta, o.tqDelta);
  Param::SetInt(Param::pA, o.p[CL_A]);
  Param::SetInt(Param::pB, o.p[CL_B]);
  Param::SetInt(Param::pC, o.p[CL_C]);
  Param::SetInt(Param::pD, o.p[CL_D]);
  Param::SetInt(Param::pE, o.p[CL_E]);
  Param::SetInt(Param::pSys, o.pSys);
  Param::SetInt(Param::iA, eds.Setpoint(EDS_A) / 10);
  Param::SetInt(Param::iB, eds.Setpoint(EDS_B) / 10);
  Param::SetInt(Param::iC, eds.Setpoint(EDS_C) / 10);
  Param::SetInt(Param::iD, eds.Setpoint(EDS_D) / 10);
  Param::SetInt(Param::iE, eds.Setpoint(EDS_E) / 10);
  Param::SetInt(Param::iSYS, eds.Setpoint(EDS_SYS) / 10);
  Param::SetInt(Param::iK0, eds.Setpoint(EDS_K0) / 10);
  Param::SetInt(Param::tSyn, o.tSyn);
  Param::SetInt(Param::gls, o.gls);
  Param::SetInt(Param::tqHold, o.tqHold);
  Param::SetInt(Param::canHold, o.canLossHold);
  Param::SetInt(Param::coast, o.coast);
  Param::SetInt(Param::canAct, canOk);
  Param::SetInt(Param::shFault, shifter.Fault());

  // Spot values (read-only, web UI).
  static const EdsValve sv[9] = {EDS_A, EDS_B, EDS_C, EDS_D, EDS_E,
                                 EDS_SYS, EDS_K0, EDS_POS, EDS_PZYL};
  for (int i = 0; i < 9; i++) {
    // Target = what the pressure or current command asks for (before the
    // slew); iX above is the slewed setpoint sent to the chip; imX is the
    // average current the chip measured (one dither period, read every
    // 100 ms).
    Param::SetInt((Param::PARAM_NUM)(Param::itA + i), eds.Target(sv[i]) / 10);
    Param::SetInt((Param::PARAM_NUM)(Param::fA + i), valves.ValveFlags(sv[i]));
    Param::SetInt((Param::PARAM_NUM)(Param::imA + i), valves.Measured(sv[i]) / 10);
    // Coil resistance estimate (0 = not valid now), read-only: this is
    // how the coil defaults get checked at first power-up (valveout.h).
    Param::SetInt((Param::PARAM_NUM)(Param::rA + i), valves.CoilMilliOhm(sv[i]));
  }
  Param::SetInt(Param::rPHOLD, valves.AuxCoilMilliOhm(VALVE_AUX_PHOLD));
  Param::SetInt(Param::imPHOLD, valves.AuxMeasured(VALVE_AUX_PHOLD) / 10);
  Param::SetInt(Param::iPHOLD, valves.Aux(VALVE_AUX_PHOLD) / 10);
  Param::SetInt(Param::parkHold, parkSafe.Holding());
  Param::SetInt(Param::iPOS, eds.Setpoint(EDS_POS) / 10);
  Param::SetInt(Param::ratio, Gears::MeasuredRatioX1000(in.n_t, in.n_ab));
  Param::SetInt(Param::slip, Gears::SlipRpm(o.gear, in.n_t, in.n_ab));
  // Selected gear: the ManualGear / VCU request, else the TCU's own target.
  int sel = o.target;
  if (in.manualGear)
    sel = in.manualGear;
  else if (Param::GetInt(Param::ShiftMode) == SHIFT_VCU && in.gearReq)
    sel = in.gearReq;
  if (in.selector == SEL_P || in.selector == SEL_N)
    sel = GEAR_N;
  else if (in.selector == SEL_R)
    sel = GEAR_R;
  Param::SetInt(Param::selGear, sel);
  Param::SetInt(Param::inertia, o.inertia);
  Param::SetInt(Param::drvMode, Param::GetInt(Param::DriveMode));
  Param::SetInt(Param::shiftMd, Param::GetInt(Param::ShiftMode));
  Param::SetInt(Param::pSysMeas, in.pMeas); // -1: no sensor on the board
  Param::SetInt(Param::lpBusFlt, linActive && (linPump.Classes() & LPC_COMM) != 0);
  Param::SetInt(Param::parkSt, o.parkState);
  Param::SetInt(Param::canLoss, !in.canOk);
  Param::SetInt(Param::tqSrc, tqSrc);
  Param::SetInt(Param::tqUsed, in.m_t);
  Param::SetInt(Param::adaStat, adapt.Status());
  Param::SetInt(Param::adaShifts, adapt.Shifts());
  for (int c = 0; c < ADA_CLUTCHES; c++)
    for (int k = 0; k < ADA_CLASSES; k++)
      Param::SetInt((Param::PARAM_NUM)(Param::adA_c + c * 2 + k), adapt.Raw(c, k));
  Param::SetInt(Param::tccSlip, o.tccLocked ? in.n_mot - in.n_t : 0);
  Param::SetInt(Param::tccSlipFlt, tccSlip.Fault());
  Param::SetInt(Param::canBus, (!in.canOk ? 1 : 0) | (lever.Seen() && !lever.Valid() ? 2 : 0));
  Param::SetInt(Param::calSig, CAL_SIG16);
}

// 100 ms task: battery, temperatures, faults, diagnosis, slow spot values.
static void Ms100Task() {
  if (BKP_DR1 != BOOT_RUNNING)
    BootStage(BOOT_RUNNING);
  DigIo::led_out.Toggle();
  // Feed the watchdog. The openinverter bootloader starts the hardware
  // watchdog before jumping to this firmware, and once started it cannot be
  // stopped. If it is not reset regularly the processor restarts (about once
  // a second), so this must run every 100 ms, as on the ZombieVerter.
  iwdg_reset();
  {
    const TcuOutputs &ao = tcu.Out();
    if (adapt.Dirty() && Param::GetBool(Param::AdaptEnable) && spdOut.Rpm() == 0 &&
        (ao.mode == TCU_PARK || ao.mode == TCU_NEUTRAL))
      AdaptSave();
  }
  Param::SetInt(Param::cpuload, scheduler->GetCpuLoad() / 10);
  {
    // Measured speed ISR load over the last 100 ms, 0.1 %: cycles /
    // 7.2 M cycles x 1000. Not in cpuload (that counts scheduler tasks).
    uint32_t cyc = spdIsrCycles;
    spdIsrCycles = 0;
    Param::SetInt(Param::spdLoad, (int32_t)(cyc / 7200));
  }

  // Battery voltage: VbatScale = volts at full ADC scale (18.33 V).
  vbatmV = AnaConv::VbatMv(SlowAdc::Get(SlowAdc::VBAT), Param::GetInt(Param::VbatScale));
  Param::SetInt(Param::vbat, vbatmV);
  if (Param::GetInt(Param::UvLimit) > 0 && vbatmV < Param::GetInt(Param::UvLimit))
    ErrorMessage::Post(ERR_UNDERVOLT);

  // ATF temperature: sensor resistance from the selected input, then the
  // fixed curve in AnaConv::AtfTempC (provisional, a generic NTC until the
  // 8HP sensor is measured). Fixed value by default (AtfSource = Fixed).
  bool in2 = Param::GetInt(Param::AtfInput) == 1;
  int32_t adc = SlowAdc::Get(in2 ? SlowAdc::ATF2 : SlowAdc::ATF1);
  int32_t ohm = AnaConv::TempInputOhms(adc, in2 ? AnaConv::TEMPIN2_PULLUP
                                                : AnaConv::TEMPIN1_PULLUP);
  Param::SetInt(Param::atfAdc, adc);
  Param::SetInt(Param::atfOhm, ohm);
  if (Param::GetInt(Param::AtfSource) == 1) {
    if (ohm < Param::GetInt(Param::AtfRMin) || ohm > Param::GetInt(Param::AtfRMax)) {
      atfTemp = ATF_SUBSTITUTE_C; // open (-1), short or out of range
      ErrorMessage::Post(ERR_ATFSENSOR);
    } else {
      atfTemp = AnaConv::AtfTempC(ohm);
    }
  } else {
    atfTemp = Param::GetInt(Param::AtfFixed);
  }
  Param::SetInt(Param::atfTemp, atfTemp);
  {
    int32_t tb;
    int src = AnaConv::BoardTempC(true, SlowAdc::Get(SlowAdc::BOARD_NTC),
                               SlowAdc::Get(SlowAdc::MCU_TEMP), &tb);
    Param::SetInt(Param::tBoard, tb);
    Param::SetInt(Param::tBoardSrc, src);
  }

  if (driversUp) {
    uint16_t f = valves.PollFaults();
    Param::SetInt(Param::valveFault, f);
    // FB_STAT bits 15:0 of IC10 / IC11, and the summary (TleSupFlag).
    Param::SetInt(Param::drvFlags0, valves.ChipFlags(0));
    Param::SetInt(Param::drvFlags1, valves.ChipFlags(1));
    Param::SetInt(Param::tleSup, valves.SupplyFlags());
    Param::SetInt(Param::tleVbat, valves.VbatmV());
    Param::SetInt(Param::tleTemp, valves.ChipTempC());
    Param::SetInt(Param::tleComErr, (int)valves.CommErrors());
    if (f & ValveOut::CriticalMask()) {
      driverFault = true;
      ErrorMessage::Post(ERR_DRVFAULT);
    } else if (f) {
      ErrorMessage::Post(ERR_AUXVALVE);
    }
    if (valves.ChipLost()) {
      driverFault = true;
      ErrorMessage::Post(ERR_DRVLOST);
    }
    // Per-channel diagnosis (diag.h) from the flags just read. The
    // reaction is the one above (pressure valve -> safe state). The V/I
    // resistance check is not fed yet (README).
    for (int v = 0; v < EDS_COUNT; v++) {
      uint8_t code = Diag::Channel(valves.ValveFlags((EdsValve)v), DIAG_OK);
      Param::SetInt((Param::PARAM_NUM)(Param::dgA + v), code);
    }
  }

  const TcuOutputs &o = tcu.Out();
  if (Param::GetInt(Param::CmdSource) == 0 &&
      msSinceRx >= (uint32_t)Param::GetInt(Param::CanTimeout))
    ErrorMessage::Post(ERR_CANTIMEOUT);

  if (o.mode == TCU_DRIVE && o.fault == TF_SPEED)
    ErrorMessage::Post(ERR_GEARRATIO);
  if (o.fault == TF_INTERLOCK)
    ErrorMessage::Post(ERR_INTERLOCK);
  if (shifter.Fault())
    ErrorMessage::Post(ERR_SHIFTER);

  // Spare inputs and speed sensor levels (for setting SpdAdcLo/Hi).
  Param::SetInt(Param::anaIn, 2 * AnaConv::ToMv(SlowAdc::Get(SlowAdc::ANA)));
  Param::SetInt(Param::dinBits, (DigIo::din1.Get() ? 1 : 0) |
                                    (DigIo::din2.Get() ? 2 : 0) |
                                    (DigIo::park_in.Get() ? 4 : 0) |
                                    (DigIo::notpark_in.Get() ? 8 : 0) |
                                    (DigIo::clutch_in.Get() ? 16 : 0)); // PA15 CL_IN
  Param::SetInt(Param::spdInLo, AnaConv::ToMv(SpdLo(0)));
  Param::SetInt(Param::spdInHi, AnaConv::ToMv(SpdHi(0)));
  Param::SetInt(Param::spdOutLo, AnaConv::ToMv(SpdLo(1)));
  Param::SetInt(Param::spdOutHi, AnaConv::ToMv(SpdHi(1)));
  // Speed sensor diagnosis (diag.h): stuck level, or no edges while
  // the other shaft shows the car moving in a steady gear.
  {
    const TcuOutputs &to = tcu.Out();
    bool inGear = to.mode == TCU_DRIVE && Gears::IsDriveGear(to.gear);
    int32_t noSig = Param::GetInt(Param::DiagSpdNoSig);
    int32_t openMv = Param::GetInt(Param::DiagSpdOpen);
    int32_t shortMv = Param::GetInt(Param::DiagSpdShort);
    bool nIn = Param::GetInt(Param::NtSource) == 0;
    uint8_t dIn = nIn ? Diag::Speed(AnaConv::ToMv(SpdLo(0)), AnaConv::ToMv(SpdHi(0)),
                                    spdIn.Rpm() > 0, inGear && spdOut.Rpm() > noSig,
                                    openMv, shortMv)
                      : (uint8_t)SPD_OK;
    uint8_t dOut = Diag::Speed(AnaConv::ToMv(SpdLo(1)), AnaConv::ToMv(SpdHi(1)),
                               spdOut.Rpm() > 0, inGear && nIn && spdIn.Rpm() > noSig,
                               openMv, shortMv);
    Param::SetInt(Param::spdInDiag, dIn);
    Param::SetInt(Param::spdOutDiag, dOut);
    if (dIn != SPD_OK || dOut != SPD_OK)
      ErrorMessage::Post(ERR_SPEEDSENS); // the gear check (TF_SPEED) reacts
  }
  SpdResetLevels();

  // LIN pump status. Reaction on a fault class: PumpFaultAct (default
  // report only).
  const LinPumpStatus &ls = linPump.Status();
  Param::SetInt(Param::lpOn, linWanted);
  Param::SetInt(Param::lpState, linPump.State());
  Param::SetInt(Param::lpOpPt, ls.opPoint);
  Param::SetInt(Param::lpRpm, ls.rpm);
  Param::SetInt(Param::lpCurr, ls.currentmA);
  Param::SetInt(Param::lpVolt, ls.supplymV);
  Param::SetInt(Param::lpEb0, ls.eb0);
  Param::SetInt(Param::lpEb1, ls.eb1);
  Param::SetInt(Param::lpOwn, linPump.StatusWord() & 0xFFFF);
  Param::SetInt(Param::lpClass, linPump.Classes());
  Param::SetInt(Param::lpTempRaw, ls.tempRaw);
  Param::SetInt(Param::lpHw, ls.hwVersion);
  Param::SetInt(Param::lpSw, ls.swVersion);
  Param::SetInt(Param::lpIdHi, ls.pumpId >> 16);
  Param::SetInt(Param::lpIdLo, ls.pumpId & 0xFFFF);
  Param::SetInt(Param::lpChkErr, linPump.LinChecksumErrors());
  Param::SetInt(Param::lpCrcErr, linPump.CrcErrors());
  Param::SetInt(Param::lpAlvErr, linPump.AliveErrors());
  // Pressure the OEM assumes from a running pump (KF_PIEP_SYS 4 bar [C]).
  // Shown only; the line pressure model does not use it yet.
  Param::SetInt(Param::lpPress,
                linWanted && ls.rpm > 0 ? Param::GetInt(Param::LinPumpP) : 0);
  if (linPump.Classes())
    ErrorMessage::Post(ERR_LINPUMP);
}

/// Called by libopeninv whenever a parameter changes.
void Param::Change(Param::PARAM_NUM paramNum) {
  switch (paramNum) {
  case Param::EcuType:
    if (can1)
      can1->ClearUserMessages(); // calls SetCanFilters() again
    break;
  default:
    break;
  }
  ConfigureEds();
  ConfigureTcu();
  ConfigureSpeed();
  ConfigureShifter();
  ConfigureLin();
}

// Start-up: hardware, parameters, CAN, drivers, then the scheduler.
extern "C" int main(void) {
  extern const TERM_CMD TermCmds[];

  // Every peripheral clock (CRC, PWR and BKP included) is on before
  // anything uses it.
  clock_setup();
  pwr_disable_backup_domain_write_protect(); // BootStage writes BKP_DR1
  uint32_t prevStage = BKP_DR1; // how far the last start got
  BootStage(BOOT_CLOCKS);
  rtc_setup();
  BootStage(BOOT_DIGIO);
  afio_setup(); // SWD only, CAN2 on PB5/PB6
  DIG_IO_CONFIGURE(DIG_IO_LIST);
  // Drivers off and in reset before anything else (at power-up nothing
  // is energised yet, so this cannot drop the park lock).
  DigIo::tle_en.Clear();
  DigIo::tle_resn.Clear();
  DigIo::tle_cs0.Set();
  DigIo::tle_cs1.Set();
  DigIo::out1.Clear();
  DigIo::out2.Clear();
  DigIo::lin_nslp.Clear(); // LIN transceiver asleep (330e: until LinWake)
  DigIo::led_out.Set();    // active low: off
  dwt_enable_cycle_counter(); // for the spdLoad measurement and WaitMs
  int lastStage = ShowLastBootStage(prevStage);
  BootStage(BOOT_SPEED_ADC);
  speed_adc_setup(spdBuf); // ADC1 first: SlowAdc uses its injected group
  BootStage(BOOT_SLOW_ADC);
  SlowAdc::Start();
  BootStage(BOOT_PARAMS);
  int paramSrc = LoadParams();
  BootStage(BOOT_ADAPT);
  AdaptLoad();
  BootStage(BOOT_SPI_LIN);
  tle_spi_setup();
  gp_pwm_setup((uint32_t)Param::GetInt(Param::GpPwmFreq)); // OUT3, duty 0 until the 10 ms task
  lin_usart_setup();
  if (Lin330e())
    LinWake(); // U7 awake and its wake flag cleared; the scheduler (and so
               // the first LIN break) only starts later
  BootStage(BOOT_PARAM_CHANGE);
  Param::Change(Param::PARAM_LAST);
  Param::SetInt(Param::paramSrc, paramSrc);
  Param::SetInt(Param::bootStage, lastStage);
  BootStage(BOOT_NVIC);
  nvic_setup();

  BootStage(BOOT_CAN1);
  Terminal t(USART3, TermCmds);
  const CanHardware::baudrates bauds[] = {
      CanHardware::Baud125, CanHardware::Baud250, CanHardware::Baud500,
      CanHardware::Baud800, CanHardware::Baud1000};
  Stm32Can c(CAN1, bauds[Param::GetInt(Param::Can1Baud)]);
  can1 = &c;
  // CAN2: ZombieVerter link on PB5 / PB6 (Can2Enable on by default).
  // Placement new into static storage: no heap, no guard variable.
  alignas(Stm32Can) static uint8_t can2mem[sizeof(Stm32Can)];
  can2 = 0;
  BootStage(BOOT_CAN2);
  if (Param::GetBool(Param::Can2Enable))
    can2 = new (can2mem) Stm32Can(CAN2, bauds[Param::GetInt(Param::Can2Baud)], true);
  FunctionPointerCallback cb(CanCallback, SetCanFilters);
  c.AddCallback(&cb);
  FunctionPointerCallback cb2(Can2Callback, SetCan2Filters);
  if (can2) {
    can2->AddCallback(&cb2);
    SetCan2Filters();
  }
  SetCanFilters();
  BootStage(BOOT_CANMAP);
  // The CAN map shares its flash page with the ZombieVerter's too. Only
  // load it when the parameter page was ours (or blank): a foreign map
  // names parameters by ids that are not ours.
  CanMap cm(&c, paramSrc != 2);
  CanSdo sdo(&c, &cm);
  sdo.SetNodeId(COMM_SDO_NODE_ID);
  TerminalCommands::SetCanMap(&cm);

  BootStage(BOOT_DRIVERS);
  StartDrivers(); // speed sensor supply is always on

  BootStage(BOOT_SCHEDULER);
  Stm32Scheduler s(TIM4);
  scheduler = &s;
  s.AddTask(Ms1Task, 1);
  s.AddTask(Ms10Task, 10);
  s.AddTask(Ms100Task, 100);

  Param::SetInt(Param::version, 4);

  while (1) {
    char ch = 0;
    t.Run();
    if (sdo.GetPrintRequest() == PRINT_JSON)
      TerminalCommands::PrintParamsJson(&sdo, &ch);
  }
  return 0;
}
