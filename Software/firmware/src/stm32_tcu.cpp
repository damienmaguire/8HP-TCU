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
 * Board: Bob's Stm32_TCU_V4 (pin map in hwdefs.h, digio_prj.h,
 * anain_prj.h and README "Pin map").
 *
 * Tasks (Stm32Scheduler on TIM4):
 *   1 ms    dither and current setpoints to the MAX22200, LIN schedule
 *   10 ms   inputs, shifter, TCU state machine, pressure -> current, CAN
 *   100 ms  driver faults, ATF temperature, battery voltage, CAN timeout
 * Interrupts: ADC2 JEOC at 40 kHz (speed sensor edges), UART4 (LIN bytes).
 *
 * CAN1 talks to the vehicle/VCU. CAN2 (logging) has no transceiver on V4
 * and is only started with Can2Enable = 1. Nothing is transmitted by
 * default (CanTxEnable = 0, LogEnable = 0). SDO replies on CAN1 are only
 * sent when a tool asks for them. The LIN pump master is off by default
 * (LinPumpEnable = 0); V4 has no LIN transceiver yet.
 */
#include "anain.h"
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
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <new>

#include "anaconv.h"
#include "eds.h"
#include "linpump.h"
#include "max22200.h"
#include "shifter.h"
#include "speedsens.h"
#include "tcu.h"
#include "valveout.h"

/// MAX22200 pins on the STM32: one chip on V4, CSB on PA4, CMD on PE8.
class StmMaxBus : public Max22200Bus
{
public:
   void Select(uint8_t dev, bool active) override
   {
      if (dev != 0)
         return; // V4 has one chip
      if (active)
         DigIo::max_cs0.Clear();
      else
         DigIo::max_cs0.Set();
   }
   void SetCmd(bool high) override
   {
      if (high)
         DigIo::max_cmd.Set();
      else
         DigIo::max_cmd.Clear();
   }
   uint8_t Transfer(uint8_t tx) override { return max_spi_transfer(tx); }
};

#define PRINT_JSON 0 // SDO print request sub-index for the JSON dump

static Stm32Scheduler *scheduler;
static CanHardware *can1;
static CanHardware *can2;
static StmMaxBus maxBus;
static Max22200 max0(maxBus, 0);
static ValveOut valves;
static Eds eds;
static Tcu tcu;
static Shifter shifter;
static LinPump linPump;
// Speed sensors: ISS (index 0, PA0) and OSS (index 1, PA1). No pin for an
// aux sensor on V4 (TeethAux is kept as a parameter only).
static SpeedSensor spdIn, spdOut;
static AdcEdge spdEdge[2];
static volatile uint32_t spdSamples = 0;
static uint32_t spdLastEdge[2];
static volatile uint16_t spdMin[2] = {4095, 4095}, spdMax[2] = {0, 0};
static bool driversUp = false;
static bool driverFault = false;
static bool configError = false;

// Latest command frame from the VCU (placeholder layout, see README).
static volatile uint8_t rxSelector = SEL_N;
static volatile uint8_t rxGearReq = 0;
static volatile int16_t rxTorque = 0;
static volatile int16_t rxMotorRpm = 0;
static volatile uint8_t rxTips = 0; // byte 6: bit 0 tip up, bit 1 tip down
static volatile uint32_t msSinceRx = 0xFFFF;

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
  c.ditherAmp = Param::GetInt(Param::DitherAmp) * 10;
  c.ditherPeriodMs = Param::GetInt(Param::DitherPeriod);
  c.ditherShape = Param::GetInt(Param::DitherShape);
  c.tempCorrection = Param::GetBool(Param::TempCorr);
  eds.Configure(c);
}

static void ConfigureTcu() {
  TcuConfig c;
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
  c.engage.variant = Param::GetInt(Param::GearboxType);
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
  c.canLossMode = Param::GetInt(Param::CanLossMode);
  c.ilOpenLow = Param::GetInt(Param::IlOpenLow) * 10;
  c.ilOpenHigh = Param::GetInt(Param::IlOpenHigh) * 10;
  // Must match ConfigureEds() so the interlock sees the real targets.
  c.iMax = Param::GetInt(Param::IMaxCoil) * 10;
  c.tempCorrection = Param::GetBool(Param::TempCorr);
  tcu.Configure(c);
}

static void ConfigureSpeed() {
  uint16_t to = Param::GetInt(Param::SpdTimeout);
  uint16_t mn = Param::GetInt(Param::SpdMinRpm);
  spdIn.Configure(SPEED_TIMER_HZ, Param::GetInt(Param::TeethIn), to, mn);
  spdOut.Configure(SPEED_TIMER_HZ, Param::GetInt(Param::TeethOut), to, mn);
  uint16_t lo = AnaConv::ToCounts(Param::GetInt(Param::SpdAdcLo));
  uint16_t hi = AnaConv::ToCounts(Param::GetInt(Param::SpdAdcHi));
  for (int i = 0; i < 2; i++)
    spdEdge[i].Configure(lo, hi);
}

static void ConfigureShifter() {
  ShifterConfig c;
  c.type = Param::GetInt(Param::ShifterType);
  c.anaRMax = Param::GetInt(Param::ShAnaR);
  c.anaNMax = Param::GetInt(Param::ShAnaN);
  c.anaMin = Param::GetInt(Param::ShAnaMin);
  c.anaMax = Param::GetInt(Param::ShAnaMax);
  shifter.Configure(c);
}

static void ConfigureLin() {
  LinPumpConfig c;
  c.enhanced = Param::GetInt(Param::LinChecksum) == 1;
  c.cycleMs = Param::GetInt(Param::LinCycleMs);
  c.rxDelayMs = Param::GetInt(Param::LinRxDelay);
  c.coldMs = Param::GetInt(Param::LinColdMs);
  c.phase2Ms = Param::GetInt(Param::LinPhase2Ms);
  c.timeoutMs = Param::GetInt(Param::LinTimeout);
  c.retryMs = 500;
  c.pollAux = Param::GetBool(Param::LinPollAux);
  linPump.Configure(c);
}

static void ConfigureValveMap() {
  for (int v = 0; v < EDS_COUNT; v++) {
    int ch = Param::GetInt(chanParams[v]);
    valves.SetMap((EdsValve)v, ch == CHAN_NOT_FITTED ? VALVE_NOT_FITTED : (uint8_t)ch);
  }
  valves.AttachChip(0, &max0); // V4: one MAX22200
  valves.AttachChip(1, 0);
}

/// Bring the MAX22200s up. Every channel stays off until the TCU asks.
static void StartDrivers() {
  DigIo::max_enable.Clear();
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
  DigIo::max_enable.Set();
  // Data sheet: wait for the chip to leave sleep (tWAKE) before SPI.
  for (volatile int i = 0; i < 72000; i++) {
  }
  Max22200Config mc;
  mc.ifsmA = Param::GetInt(Param::MaxIfs);
  mc.freq80k = Param::GetInt(Param::MaxFreq) == 1;
  mc.freqDiv = Param::GetInt(Param::MaxFreqDiv);
  mc.slewControl = false;
  mc.openLoadDetect = Param::GetBool(Param::MaxOpenLoad);
  mc.maxCurrentmA = Param::GetInt(Param::IMaxCoil);
  mc.halfScaleMask = 0; // set per chip by ValveOut from the mask below
  if (valves.Init(mc, (uint16_t)Param::GetInt(Param::MaxHfsMask))) {
    driversUp = true;
    eds.Release();
    tcu.Ready();
  } else {
    ErrorMessage::Post(ERR_DRVINIT);
    DigIo::max_enable.Clear();
    driverFault = true;
  }
}

// ---------------------------------------------------------------------
// Interrupts

/// Speed sensors: ADC2 injected pair (ISS, OSS) at SPEED_TIMER_HZ. The
/// sample count is the time base; AdcEdge finds the edges.
extern "C" void adc1_2_isr(void) {
  if (!(ADC_SR(SPEED_ADC) & ADC_SR_JEOC))
    return;
  ADC_SR(SPEED_ADC) &= ~ADC_SR_JEOC;
  uint16_t v[2] = {(uint16_t)ADC_JDR1(SPEED_ADC), (uint16_t)ADC_JDR2(SPEED_ADC)};
  uint32_t n = ++spdSamples;
  SpeedSensor *sens[2] = {&spdIn, &spdOut};
  for (int i = 0; i < 2; i++) {
    if (v[i] < spdMin[i])
      spdMin[i] = v[i];
    if (v[i] > spdMax[i])
      spdMax[i] = v[i];
    if (spdEdge[i].Sample(v[i])) {
      sens[i]->Edge(n - spdLastEdge[i]); // dropped inside after a timeout
      spdLastEdge[i] = n;
    }
  }
}

// ---------------------------------------------------------------------
// LIN master (aux oil pump) on UART4. The UART hears its own bytes, so
// the read-back buffer holds the header (and for 0x30 our data), then the
// pump's response. linpump.h has the protocol; this is just byte moving.

#define LIN_FRAME_MS 10 // break + header + 9 bytes at 19200 with margin

static uint8_t linTx[11];
static volatile uint8_t linTxN = 0, linTxPos = 0;
static volatile uint8_t linRx[24];
static volatile uint8_t linRxN = 0;
static uint8_t linId = 0;
static bool linPublish = false;
static uint8_t linBusyMs = 0;
static bool linWanted = false;

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

static void LinStart(const LinSlot &s) {
  uint8_t pid = Lin::Pid(s.id);
  uint8_t n = 0;
  linTx[n++] = 0x55;
  linTx[n++] = pid;
  if (s.publish) {
    for (int i = 0; i < 8; i++)
      linTx[n++] = s.data[i];
    linTx[n++] = Lin::Checksum(Param::GetInt(Param::LinChecksum) == 1, pid,
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

static void LinTick1ms() {
  if (linBusyMs) {
    if (++linBusyMs > LIN_FRAME_MS) {
      LinFinish();
      linBusyMs = 0;
    }
  }
  LinSlot slot;
  if (linPump.Tick1ms(linWanted, &slot) && !linBusyMs)
    LinStart(slot); // a slot that falls into a busy frame is skipped
  if (linPump.TransceiverAwake())
    DigIo::lin_nslp.Set();
  else
    DigIo::lin_nslp.Clear();
}

extern "C" void tim4_isr(void) { scheduler->Run(); }

// ---------------------------------------------------------------------
// CAN

static bool CanCallback(uint32_t id, uint32_t data[2], uint8_t dlc) {
  if (id == (uint32_t)Param::GetInt(Param::CanCmdId) && dlc >= 6) {
    /* Placeholder VCU command frame (little endian), see README:
     *  byte 0  selector 0=P 1=R 2=N 3=D
     *  byte 1  gear request 0 = hold, 1..8
     *  byte 2-3 actual motor torque, Nm, signed
     *  byte 4-5 motor speed, rpm, signed
     *  byte 6  (ShifterType = CAN) bit 0 tip up, bit 1 tip down, held
     *          while the lever is held. Optional: DLC 6 means no tips.
     */
    uint8_t *b = (uint8_t *)data;
    rxSelector = b[0] & 3;
    rxGearReq = b[1];
    rxTorque = (int16_t)(b[2] | (b[3] << 8));
    rxMotorRpm = (int16_t)(b[4] | (b[5] << 8));
    rxTips = dlc >= 7 ? (b[6] & 3) : 0;
    msSinceRx = 0;
    return true;
  }
  return false;
}

static void SetCanFilters() {
  can1->RegisterUserMessage(Param::GetInt(Param::CanCmdId));
}

static void SendStatus(const TcuOutputs &o, int32_t temp) {
  /* Placeholder TCU status frame (little endian), see README:
   *  byte 0  engaged gear (0 = N, 1..8, 9 = R)
   *  byte 1  mode (opmode)
   *  byte 2  target gear
   *  byte 3  fault
   *  byte 4-5 torque intervention request, Nm, signed (0 = none)
   *  byte 6  ATF temperature + 40 degC
   *  byte 7  bit 0 shifting, bit 1 park request, bit 2 safe state,
   *          bit 3 hold motor torque near 0 (gear-in), bit 4 CAN-loss
   *          gear hold, bit 5 coast shift
   */
  uint8_t b[8];
  b[0] = o.gear;
  b[1] = o.mode;
  b[2] = o.target;
  b[3] = o.fault;
  b[4] = o.tqDelta & 0xFF;
  b[5] = (o.tqDelta >> 8) & 0xFF;
  int32_t t = temp + 40;
  b[6] = t < 0 ? 0 : (t > 255 ? 255 : t);
  b[7] = (o.mode == TCU_SHIFT ? 1 : 0) | (o.parkRequest ? 2 : 0) |
         (o.safeOutputs ? 4 : 0) | (o.tqHold ? 8 : 0) |
         (o.canLossHold ? 16 : 0) | (o.coast ? 32 : 0);
  can1->Send(Param::GetInt(Param::CanStatId), b, 8);
}

static void Put16(uint8_t *b, int i, int32_t v) {
  b[2 * i] = v & 0xFF;
  b[2 * i + 1] = (v >> 8) & 0xFF;
}

/// CAN2 logging frames, every 10 ms. Layout in README.
static void SendLog(const TcuOutputs &o, const TcuInputs &in) {
  uint32_t base = Param::GetInt(Param::LogBaseId);
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
// Tasks

static void Ms1Task() {
  LinTick1ms();
  eds.Tick1ms();
  if (driversUp && !eds.InSafeState())
    valves.Update(eds);
  if (msSinceRx < 0xFFFF)
    msSinceRx++;
}

static int32_t atfTemp = 40;
static int32_t vbatmV = 12000;

static void Ms10Task() {
  spdIn.Tick10ms();
  spdOut.Tick10ms();

  bool fromParam = Param::GetInt(Param::CmdSource) == 1;
  bool canOk = msSinceRx < (uint32_t)Param::GetInt(Param::CanTimeout);

  // Shifter: CAN fields of the command frame, or the V4 inputs.
  ShifterRaw raw;
  raw.canSelector = canOk ? rxSelector : (uint8_t)SEL_N;
  raw.canTipUp = canOk && (rxTips & 1);
  raw.canTipDown = canOk && (rxTips & 2);
  raw.din1 = DigIo::din1.Get();
  raw.din2 = DigIo::din2.Get();
  raw.parkIn = DigIo::park_in.Get();
  raw.notParkIn = DigIo::notpark_in.Get();
  raw.anaMv = 2 * AnaConv::ToMv(AnaIn::ana_in.Get()); // /2 on the board
  shifter.Update(raw);
  bool canShifter = Param::GetInt(Param::ShifterType) == SHIFTER_CAN;

  TcuInputs in = {};
  if (fromParam)
    in.selector = Param::GetInt(Param::SelectorSet);
  else if (canShifter && !canOk)
    in.selector = rxSelector; // last frame; CAN-loss rules below
  else
    in.selector = shifter.Selector();
  in.tipUp = !fromParam && shifter.TipUp();
  in.tipDown = !fromParam && shifter.TipDown();
  in.gearReq = fromParam ? Param::GetInt(Param::GearSet) : rxGearReq;
  in.m_t = fromParam ? Param::GetInt(Param::TorqueSet) : rxTorque;
  in.n_mot = fromParam ? Param::GetInt(Param::MotorRpmSet) : rxMotorRpm;
  if (!fromParam && !canOk) {
    // No VCU. CanLossMode 1 (default, Damien 2026-11-23): keep the last
    // selector so the TCU holds the gear (no upshifts, LineMax, step down
    // as the car slows). CanLossMode 0: go to neutral. Either way assume
    // high torque for line pressure: CanLossTq defaults to the OEM
    // substitute D_m_IstVM_Default = 600 Nm (@0x218144) [C]. The OEM
    // fault-to-reaction table (KFC -> KMN) was not found, so what exactly
    // the OEM does on KFCx_ENGSIGTRQ is [H]; hold-gear is Ian's reading
    // of the limp states [I] (QA2 answer 8).
    if (Param::GetInt(Param::CanLossMode) != CANLOSS_HOLD)
      in.selector = SEL_N;
    in.gearReq = 0;
    in.tipUp = in.tipDown = false;
    in.m_t = Param::GetInt(Param::CanLossTq);
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
  if (Param::GetBool(Param::FaultClear)) {
    Param::SetInt(Param::FaultClear, 0);
    // Only at standstill in P or N: restart the chips, then let the TCU
    // leave the safe state if nothing is still wrong.
    if (tcu.Out().mode == TCU_SAFE && in.n_ab == 0 &&
        (in.selector == SEL_P || in.selector == SEL_N)) {
      driverFault = false;
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

  // Aux oil pump: line pressure needed at low input speed (Ian's spec).
  bool pressureNeeded = o.mode == TCU_NEUTRAL || o.mode == TCU_ENGAGE ||
                        o.mode == TCU_DRIVE || o.mode == TCU_SHIFT;
  linWanted = LinPumpDemand(Param::GetInt(Param::LinPumpReq),
                            Param::GetBool(Param::LinPumpEnable),
                            pressureNeeded, in.n_t,
                            Param::GetInt(Param::LinPumpRpm),
                            Param::GetInt(Param::LinPumpHyst), linWanted);

  // Discrete outputs. Park-hold magnet: its sequence is not known yet, so
  // it stays off except for the bench test at standstill in P or N.
  bool standstillPN = in.n_ab == 0 && (o.mode == TCU_PARK || o.mode == TCU_NEUTRAL);
  if (Param::GetBool(Param::ParkHoldTest) && standstillPN && !o.safeOutputs)
    DigIo::park_hold.Set();
  else
    DigIo::park_hold.Clear();
  // Spare outputs: StarterLock = on while the TCU is in P or N [H: the
  // polarity the starter interlock wants is not known].
  const Param::PARAM_NUM gpFunc[2] = {Param::GpOut1Func, Param::GpOut2Func};
  DigIo *gpPin[2] = {&DigIo::gp_out1, &DigIo::gp_out2};
  for (int i = 0; i < 2; i++) {
    int f = Param::GetInt(gpFunc[i]);
    bool on = f == 1 || (f == 2 && (o.mode == TCU_PARK || o.mode == TCU_NEUTRAL));
    if (on)
      gpPin[i]->Set();
    else
      gpPin[i]->Clear();
  }

  if (o.safeOutputs || !driversUp) {
    eds.SafeState();
    valves.AllOff();
  } else {
    eds.Release();
    for (int c = 0; c < CL_COUNT; c++)
      eds.SetPressure((EdsValve)c, o.p[c], atfTemp);
    eds.SetPressure(EDS_SYS, o.pSys, atfTemp);
    eds.SetPressure(EDS_K0, 0, atfTemp); // K0 open: no engine on an EV
    eds.SetCurrent(EDS_POS, Param::GetInt(Param::PosCurrent) * 10);
    // Park-lock cylinder valve: stub until its sequence is known (and not
    // on a V4 channel by default).
    eds.SetCurrent(EDS_PZYL, 0);

    // Bench valve test: only at standstill in P or N.
    if (Param::GetBool(Param::ValveTest) && in.n_ab == 0 &&
        (o.mode == TCU_NEUTRAL || o.mode == TCU_PARK)) {
      int v = Param::GetInt(Param::ValveTestSel);
      if (v >= 0 && v < EDS_COUNT)
        eds.SetCurrent((EdsValve)v, Param::GetInt(Param::ValveTestmA) * 10);
    }
  }
  eds.Tick10ms();

  // A fault pin pulled low gets read at once rather than at 100 ms.
  if (driversUp && !DigIo::max_nfault.Get()) {
    uint16_t f = valves.PollFaults();
    if (f & ValveOut::CriticalMask())
      driverFault = true;
    Param::SetInt(Param::valveFault, f);
  }

  if (Param::GetBool(Param::CanTxEnable))
    SendStatus(o, atfTemp);
  if (Param::GetBool(Param::LogEnable) && can2)
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
  Param::SetInt(Param::nAux, 0); // no aux speed sensor on V4
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
}

static void Ms100Task() {
  DigIo::led_out.Toggle();
  Param::SetInt(Param::cpuload, scheduler->GetCpuLoad() / 10);

  // Battery voltage: VbatScale = volts at full ADC scale (V4: 18.33 V).
  vbatmV = AnaConv::VbatMv(AnaIn::vbat.Get(), Param::GetInt(Param::VbatScale));
  Param::SetInt(Param::vbat, vbatmV);
  if (Param::GetInt(Param::UvLimit) > 0 && vbatmV < Param::GetInt(Param::UvLimit))
    ErrorMessage::Post(ERR_UNDERVOLT);

  // ATF temperature: sensor resistance from the selected input, then the
  // AtfR / AtfT table (generic NTC until the sensor is known). Fixed value
  // by default (AtfSource = Fixed).
  bool in2 = Param::GetInt(Param::AtfInput) == 1;
  int32_t adc = in2 ? AnaIn::atf_temp2.Get() : AnaIn::atf_temp1.Get();
  int32_t ohm = AnaConv::TempInputOhms(adc, in2 ? AnaConv::TEMPIN2_PULLUP
                                                : AnaConv::TEMPIN1_PULLUP);
  Param::SetInt(Param::atfAdc, adc);
  Param::SetInt(Param::atfOhm, ohm);
  if (Param::GetInt(Param::AtfSource) == 1) {
    if (ohm < Param::GetInt(Param::AtfRMin) || ohm > Param::GetInt(Param::AtfRMax)) {
      atfTemp = ATF_SUBSTITUTE_C; // open (-1), short or out of range
      ErrorMessage::Post(ERR_ATFSENSOR);
    } else {
      int32_t r[6], t[6];
      const Param::PARAM_NUM rp[6] = {Param::AtfR1, Param::AtfR2, Param::AtfR3,
                                      Param::AtfR4, Param::AtfR5, Param::AtfR6};
      for (int i = 0; i < 6; i++) {
        r[i] = Param::GetInt(rp[i]);
        t[i] = Param::GetInt((Param::PARAM_NUM)(Param::AtfT1 + i));
      }
      atfTemp = AnaConv::TempFromOhms(ohm, r, t, 6);
    }
  } else {
    atfTemp = Param::GetInt(Param::AtfFixed);
  }
  Param::SetInt(Param::atfTemp, atfTemp);

  if (driversUp) {
    uint16_t f = valves.PollFaults();
    Param::SetInt(Param::valveFault, f);
    Param::SetInt(Param::drvFlags0, valves.ChipFlags(0));
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
  Param::SetInt(Param::anaIn, 2 * AnaConv::ToMv(AnaIn::ana_in.Get()));
  Param::SetInt(Param::dinBits, (DigIo::din1.Get() ? 1 : 0) |
                                    (DigIo::din2.Get() ? 2 : 0) |
                                    (DigIo::park_in.Get() ? 4 : 0) |
                                    (DigIo::notpark_in.Get() ? 8 : 0));
  Param::SetInt(Param::spdInLo, AnaConv::ToMv(spdMin[0]));
  Param::SetInt(Param::spdInHi, AnaConv::ToMv(spdMax[0]));
  Param::SetInt(Param::spdOutLo, AnaConv::ToMv(spdMin[1]));
  Param::SetInt(Param::spdOutHi, AnaConv::ToMv(spdMax[1]));
  for (int i = 0; i < 2; i++) {
    spdMin[i] = 4095;
    spdMax[i] = 0;
  }

  // gp_pwm duty (frequency is applied on a parameter change)
  gp_pwm_set(Param::GetInt(Param::GpPwmDuty));

  // LIN pump status
  const LinPumpStatus &ls = linPump.Status();
  Param::SetInt(Param::lpOn, linWanted);
  Param::SetInt(Param::lpState, linPump.State());
  Param::SetInt(Param::lpStatus, ls.status);
  Param::SetInt(Param::lpRpm, ls.rpm);
  Param::SetInt(Param::lpVolt, ls.supplymV);
  Param::SetInt(Param::lpEcho, ls.echo);
  Param::SetInt(Param::lpFaults, linPump.CommFaults());
  Param::SetInt(Param::lpCsErr, linPump.ChecksumErrors());
  Param::SetInt(Param::lpTimeouts, linPump.Timeouts());
  if (linWanted && linPump.State() == LP_FAULT)
    ErrorMessage::Post(ERR_LINPUMP);
}

/// Called by libopeninv whenever a parameter changes.
void Param::Change(Param::PARAM_NUM paramNum) {
  switch (paramNum) {
  case Param::CanCmdId:
    if (can1)
      can1->ClearUserMessages(); // calls SetCanFilters() again
    break;
  case Param::GpPwmFreq:
    gp_pwm_setup(Param::GetInt(Param::GpPwmFreq));
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

extern "C" int main(void) {
  extern const TERM_CMD TermCmds[];

  clock_setup();
  rtc_setup();
  afio_setup(); // SWD only, TIM3 full remap, CAN2 on PB5/PB6
  ANA_IN_CONFIGURE(ANA_IN_LIST);
  DIG_IO_CONFIGURE(DIG_IO_LIST);
  DigIo::max_enable.Clear(); // drivers off before anything else
  DigIo::max_cs0.Set();
  DigIo::max_cmd.Clear();
  DigIo::max_triga.Clear(); // SPI-controlled channels: TRIGA/B held low
  DigIo::max_trigb.Clear();
  DigIo::park_hold.Clear();
  DigIo::gp_out1.Clear();
  DigIo::gp_out2.Clear();
  DigIo::lin_nslp.Clear(); // LIN transceiver asleep
  DigIo::led_out.Set();    // active low: off
  AnaIn::Start();
  parm_load();
  max_spi_setup();
  gp_pwm_setup(Param::GetInt(Param::GpPwmFreq));
  lin_usart_setup();
  Param::Change(Param::PARAM_LAST);
  speed_adc_setup();
  nvic_setup();

  Terminal t(USART3, TermCmds);
  const CanHardware::baudrates bauds[] = {
      CanHardware::Baud125, CanHardware::Baud250, CanHardware::Baud500,
      CanHardware::Baud800, CanHardware::Baud1000};
  Stm32Can c(CAN1, bauds[Param::GetInt(Param::Can1Baud)]);
  can1 = &c;
  // CAN2: no transceiver on V4 (REV: PB5/PB6). Only started on request.
  // Placement new into static storage: no heap, no guard variable.
  alignas(Stm32Can) static uint8_t can2mem[sizeof(Stm32Can)];
  can2 = 0;
  if (Param::GetBool(Param::Can2Enable))
    can2 = new (can2mem) Stm32Can(CAN2, bauds[Param::GetInt(Param::Can2Baud)], true);
  FunctionPointerCallback cb(CanCallback, SetCanFilters);
  c.AddCallback(&cb);
  SetCanFilters();
  CanMap cm(&c);
  CanSdo sdo(&c, &cm);
  sdo.SetNodeId(Param::GetInt(Param::SdoNodeId));
  TerminalCommands::SetCanMap(&cm);

  StartDrivers(); // speed sensor supply is always on (V4)

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
