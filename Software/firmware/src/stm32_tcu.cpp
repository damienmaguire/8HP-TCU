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
 * Tasks (Stm32Scheduler on TIM4):
 *   1 ms    dither and current setpoints to the MAX22200s
 *   10 ms   inputs, TCU state machine, pressure -> current, CAN frames
 *   100 ms  driver faults, ATF temperature, battery voltage, CAN timeout
 *
 * CAN1 talks to the vehicle/VCU, CAN2 is a logging bus. Both transmit
 * NOTHING by default (CanTxEnable = 0, LogEnable = 0). SDO replies on CAN1
 * are only sent when a tool asks for them.
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
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#include "eds.h"
#include "max22200.h"
#include "speedsens.h"
#include "tcu.h"
#include "valveout.h"

/// MAX22200 pins on the STM32 (CS per chip, shared CMD, SPI).
class StmMaxBus : public Max22200Bus
{
public:
   void Select(uint8_t dev, bool active) override
   {
      DigIo &cs = dev == 0 ? DigIo::max_cs0 : DigIo::max_cs1;
      if (active)
         cs.Clear();
      else
         cs.Set();
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
static Max22200 max1(maxBus, 1);
static ValveOut valves;
static Eds eds;
static Tcu tcu;
static SpeedSensor spdIn, spdOut, spdAux;
static volatile uint16_t lastStamp[3];
static bool driversUp = false;
static bool driverFault = false;
static bool configError = false;

// Latest command frame from the VCU (placeholder layout, see README).
static volatile uint8_t rxSelector = SEL_N;
static volatile uint8_t rxGearReq = 0;
static volatile int16_t rxTorque = 0;
static volatile int16_t rxMotorRpm = 0;
static volatile uint32_t msSinceRx = 0xFFFF;

static const Param::PARAM_NUM chanParams[EDS_COUNT] = {
    Param::ChanA,   Param::ChanB,    Param::ChanC,    Param::ChanD,
    Param::ChanE,   Param::ChanSYS,  Param::ChanK0,   Param::ChanPOS,
    Param::ChanPZYL, Param::ChanSOL1, Param::ChanPARK};

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
  c.engage.syncCloseGrad = Param::GetInt(Param::EngageCloseGrad);
  c.gearSelMode = Param::GetInt(Param::GearSelMode);
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
  tcu.Configure(c);
}

static void ConfigureSpeed() {
  uint16_t to = Param::GetInt(Param::SpdTimeout);
  uint16_t mn = Param::GetInt(Param::SpdMinRpm);
  spdIn.Configure(SPEED_TIMER_HZ, Param::GetInt(Param::TeethIn), to, mn);
  spdOut.Configure(SPEED_TIMER_HZ, Param::GetInt(Param::TeethOut), to, mn);
  spdAux.Configure(SPEED_TIMER_HZ, Param::GetInt(Param::TeethAux), to, mn);
}

static void ConfigureValveMap() {
  for (int v = 0; v < EDS_COUNT; v++) {
    int ch = Param::GetInt(chanParams[v]);
    valves.SetMap((EdsValve)v, ch == CHAN_NOT_FITTED ? VALVE_NOT_FITTED : (uint8_t)ch);
  }
  valves.AttachChip(0, &max0);
  valves.AttachChip(1, Param::GetInt(Param::MaxChips) > 1 ? &max1 : 0);
}

/// Bring the MAX22200s up. Every channel stays off until the TCU asks.
static void StartDrivers() {
  DigIo::max_enable.Clear();
  driversUp = false;
  ConfigureValveMap();
  configError = !valves.Validate();
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
  mc.halfScale = Param::GetBool(Param::MaxHfs);
  mc.freq80k = Param::GetInt(Param::MaxFreq) == 1;
  mc.freqDiv = Param::GetInt(Param::MaxFreqDiv);
  mc.slewControl = false;
  mc.openLoadDetect = Param::GetBool(Param::MaxOpenLoad);
  mc.maxCurrentmA = Param::GetInt(Param::IMaxCoil);
  Max22200Hit hit;
  hit.hitmA = Param::GetInt(Param::HitCurrent);
  hit.hitTime = Param::GetInt(Param::HitTime);
  if (valves.Init(mc, hit)) {
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

/// Speed sensor edges. Placeholder pins on EXTI 7, 8, 9 (TBD).
extern "C" void exti9_5_isr(void) {
  uint16_t now = timer_get_counter(SPEED_TIMER);
  SpeedSensor *s[3] = {&spdIn, &spdOut, &spdAux};
  const uint32_t lines[3] = {EXTI7, EXTI8, EXTI9};
  for (int i = 0; i < 3; i++) {
    if (exti_get_flag_status(lines[i])) {
      exti_reset_request(lines[i]);
      uint16_t period = now - lastStamp[i];
      lastStamp[i] = now;
      s[i]->Edge(period); // dropped inside if the sensor had timed out
    }
  }
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
     */
    uint8_t *b = (uint8_t *)data;
    rxSelector = b[0] & 3;
    rxGearReq = b[1];
    rxTorque = (int16_t)(b[2] | (b[3] << 8));
    rxMotorRpm = (int16_t)(b[4] | (b[5] << 8));
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
   *  byte 7  bit 0 shifting, bit 1 park request, bit 2 safe state
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
         (o.safeOutputs ? 4 : 0);
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
  spdAux.Tick10ms();

  bool fromParam = Param::GetInt(Param::CmdSource) == 1;
  bool canOk = msSinceRx < (uint32_t)Param::GetInt(Param::CanTimeout);

  TcuInputs in;
  in.selector = fromParam ? Param::GetInt(Param::SelectorSet) : rxSelector;
  in.gearReq = fromParam ? Param::GetInt(Param::GearSet) : rxGearReq;
  in.m_t = fromParam ? Param::GetInt(Param::TorqueSet) : rxTorque;
  in.n_mot = fromParam ? Param::GetInt(Param::MotorRpmSet) : rxMotorRpm;
  if (!fromParam && !canOk) {
    // No VCU: hold what we have, assume high torque for line pressure.
    in.selector = SEL_N;
    in.gearReq = 0;
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
    // Park lock and Sol1 are stubs until their sequence is known.
    eds.SetCurrent(EDS_PZYL, 0);
    eds.SetCurrent(EDS_SOL1, 0);
    eds.SetCurrent(EDS_PARK, 0);

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
  if (Param::GetBool(Param::LogEnable))
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
  Param::SetInt(Param::nAux, spdAux.Rpm());
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
  Param::SetInt(Param::canAct, canOk);
}

static void Ms100Task() {
  DigIo::led_out.Toggle();
  Param::SetInt(Param::cpuload, scheduler->GetCpuLoad() / 10);

  // Battery voltage: VbatScale is the voltage at full ADC scale (TBD).
  vbatmV = (int32_t)(((int64_t)AnaIn::vbat.Get() * Param::GetInt(Param::VbatScale)) / 4095);
  Param::SetInt(Param::vbat, vbatmV);
  if (Param::GetInt(Param::UvLimit) > 0 && vbatmV < Param::GetInt(Param::UvLimit))
    ErrorMessage::Post(ERR_UNDERVOLT);

  // ATF temperature: fixed value until the sensor curve is measured (Q4).
  int32_t adc = AnaIn::atf_temp.Get();
  Param::SetInt(Param::atfAdc, adc);
  if (Param::GetInt(Param::AtfSource) == 1) {
    if (adc < Param::GetInt(Param::AtfAdcMin) || adc > Param::GetInt(Param::AtfAdcMax)) {
      atfTemp = AtfTemp::SUBSTITUTE_C;
      ErrorMessage::Post(ERR_ATFSENSOR);
    } else {
      int32_t a[6], t[6];
      for (int i = 0; i < 6; i++) {
        a[i] = Param::GetInt((Param::PARAM_NUM)(Param::AtfAdc1 + i));
        t[i] = Param::GetInt((Param::PARAM_NUM)(Param::AtfT1 + i));
      }
      atfTemp = AtfTemp::FromAdc(adc, a, t, 6);
    }
  } else {
    atfTemp = Param::GetInt(Param::AtfFixed);
  }
  Param::SetInt(Param::atfTemp, atfTemp);

  if (driversUp) {
    uint16_t f = valves.PollFaults();
    Param::SetInt(Param::valveFault, f);
    Param::SetInt(Param::drvFlags0, valves.ChipFlags(0));
    Param::SetInt(Param::drvFlags1, valves.ChipFlags(1));
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
}

/// Called by libopeninv whenever a parameter changes.
void Param::Change(Param::PARAM_NUM paramNum) {
  switch (paramNum) {
  case Param::CanCmdId:
    if (can1)
      can1->ClearUserMessages(); // calls SetCanFilters() again
    break;
  default:
    break;
  }
  ConfigureEds();
  ConfigureTcu();
  ConfigureSpeed();
}

extern "C" int main(void) {
  extern const TERM_CMD TermCmds[];

  clock_setup();
  rtc_setup();
  // SWD only (frees the JTAG pins), CAN2 on PB5/PB6 as on the ZombieVerter.
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN2_REMAP);
  ANA_IN_CONFIGURE(ANA_IN_LIST);
  DIG_IO_CONFIGURE(DIG_IO_LIST);
  DigIo::max_enable.Clear(); // drivers off before anything else
  DigIo::max_cs0.Set();
  DigIo::max_cs1.Set();
  DigIo::max_cmd.Clear();
  AnaIn::Start();
  nvic_setup();
  parm_load();
  max_spi_setup();
  speed_timer_setup();
  speed_exti_setup();
  Param::Change(Param::PARAM_LAST);

  Terminal t(USART3, TermCmds);
  const CanHardware::baudrates bauds[] = {
      CanHardware::Baud125, CanHardware::Baud250, CanHardware::Baud500,
      CanHardware::Baud800, CanHardware::Baud1000};
  Stm32Can c(CAN1, bauds[Param::GetInt(Param::Can1Baud)]);
  Stm32Can c2(CAN2, bauds[Param::GetInt(Param::Can2Baud)], true);
  can1 = &c;
  can2 = &c2;
  FunctionPointerCallback cb(CanCallback, SetCanFilters);
  c.AddCallback(&cb);
  SetCanFilters();
  CanMap cm(&c);
  CanSdo sdo(&c, &cm);
  sdo.SetNodeId(Param::GetInt(Param::SdoNodeId));
  TerminalCommands::SetCanMap(&cm);

  DigIo::sens_supply.Set(); // speed sensor supply (TBD whether switched)
  StartDrivers();

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
