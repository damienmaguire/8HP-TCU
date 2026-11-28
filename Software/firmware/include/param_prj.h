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

#ifndef PARAM_PRJ_H_INCLUDED
#define PARAM_PRJ_H_INCLUDED

#define VER 0.1.0

/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
// Next param id (increase when adding a new parameter!): 273
// Next value id: 2190. Retired ids are never reused (test_params.cpp).
// Retired parameter ids: once used by this firmware, now removed. A saved
// parameter page may still hold them, so the start-up flash check
// (flashguard.h) treats them as ours and parm_load() skips them.
// 28-33 AtfT1..6 and 137-142 AtfR1..6: the ATF curve is now a fixed table
// (anaconv.h). 155, 156, 164-166, 168-170, 195-203: LIN pump settings now
// fixed in stm32_tcu.cpp (ConfigureLin). 230-242: adaptation entry
// conditions and rule, now fixed in stm32_tcu.cpp (ConfigureTcu). 108,
// 110, 111, 115, 117, 118, 215, 216, 218-228, 249, 250, 262, 263:
// communication settings now fixed in stm32_tcu.cpp (COMM_*); 181 F11TqTx
// was removed with its 0x0B0 torque frame. 2037, 2110 (tq0B0) and 2111
// (tqQual) are retired spot value ids. V5 board (TLE92466ED drivers): 41
// MaxIfs, 43 MaxFreq, 44 MaxFreqDiv, 146 MaxHfsMask (old driver chip
// settings), 49 DitherAmp, 50 DitherPeriod, 51 DitherShape (the dither is
// now a fixed table made by the driver chip, valveout.h) and 272
// GpOutCurrent (OUT1..3 are plain switches now); spot values 2148
// diagPair and 2159 gpFault.
#define PARAM_RETIRED_IDS                                                      \
  1, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 40, 42, 53, 54,   \
      69, 70, 102, 106, 107, 137, 138, 139, 140, 141, 142, 155, 156, 157, 158, \
      159, 160, 161, 162, 164, 165, 166, 168, 169, 170, 195, 196, 197, 198,    \
      199, 200, 201, 202, 203, 230, 231, 232, 233, 234, 235, 236, 237, 238,    \
      239, 240, 241, 242, 108, 110, 111, 115, 117, 118, 181, 215, 216, 218,    \
      219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 249, 250, 262, 263,    \
      2037, 2110, 2111, 41, 43, 44, 146, 49, 50, 51, 272, 2148, 2159
// (ids 9, 101, 103-105 and 108 were free slots and are used below; 102,
// 106 and 107 were used by an unreleased draft and are left free. The V4
// update dropped 1, 22-27, 34, 35, 40, 42, 53, 54, 69 and 70; they are
// not reused so an old saved parameter set cannot land on a new meaning.
// The first LIN draft's 157-162 are dropped as well.)
//
// Anything marked TBD below has not been measured yet. The defaults are
// placeholders chosen so the firmware runs on a bench, not values to drive
// a car on. See README.md "Parameters for unmeasured items".
//
/*              category     name         unit       min     max     default id
 */
#define PARAM_LIST                                                             \
  PARAM_ENTRY(CAT_SETUP, TopGear, "", 1, 8, 8, 2)                              \
  PARAM_ENTRY(CAT_SETUP, CmdSource, CMDSRC, 0, 1, 0, 3)                        \
  PARAM_ENTRY(CAT_SETUP, NtSource, NTSRC, 0, 1, 0, 7)                          \
  PARAM_ENTRY(CAT_SETUP, FaultClear, ONOFF, 0, 1, 0, 8)                        \
  PARAM_ENTRY(CAT_SETUP, GearboxType, GBOXTYPES, 0, 6, 0, 9)                   \
  PARAM_ENTRY(CAT_SETUP, SolCurveSet, SOLSETS, 0, 4, 0, 214)                   \
  PARAM_ENTRY(CAT_SETUP, ShiftMode, SHIFTMODES, 0, 2, 0, 127)                  \
  PARAM_ENTRY(CAT_SETUP, ShifterType, SHIFTERS, 0, 6, 0, 128)                  \
  PARAM_ENTRY(CAT_SETUP, ShAnaR, "mV", 0, 6600, 1500, 129)                     \
  PARAM_ENTRY(CAT_SETUP, ShAnaN, "mV", 0, 6600, 3000, 130)                     \
  PARAM_ENTRY(CAT_SETUP, ShAnaMin, "mV", 0, 6600, 200, 131)                    \
  PARAM_ENTRY(CAT_SETUP, ShAnaMax, "mV", 0, 6600, 6000, 132)                   \
  PARAM_ENTRY(CAT_SETUP, ManualGear, MANGEARS, 0, 11, 0, 171)                  \
  PARAM_ENTRY(CAT_SETUP, TipOverride, "ms", 0, 60000, 0, 172)                  \
  PARAM_ENTRY(CAT_SETUP, Tip12Deb, "ms", 10, 200, 30, 173)                     \
  PARAM_ENTRY(CAT_SETUP, EcuType, ECUTYPES, 0, 0, 0, 182)                      \
  PARAM_ENTRY(CAT_SPEED, TeethIn, "", 1, 200, 60, 10)                          \
  PARAM_ENTRY(CAT_SPEED, TeethOut, "", 1, 200, 60, 11)                         \
  PARAM_ENTRY(CAT_SPEED, TeethAux, "", 1, 200, 60, 12)                         \
  PARAM_ENTRY(CAT_SPEED, SpdTimeout, "ms", 10, 130, 100, 13)                   \
  PARAM_ENTRY(CAT_SPEED, SpdMinRpm, "rpm", 0, 500, 30, 14)                     \
  PARAM_ENTRY(CAT_SPEED, SpdAdcLo, "mV", 0, 3300, 900, 135)                    \
  PARAM_ENTRY(CAT_SPEED, SpdAdcHi, "mV", 0, 3300, 1200, 136)                   \
  PARAM_ENTRY(CAT_ATF, AtfSource, ATFSRC, 0, 1, 0, 20)                         \
  PARAM_ENTRY(CAT_ATF, AtfFixed, "C", -40, 150, 40, 21)                        \
  PARAM_ENTRY(CAT_ATF, AtfInput, ATFINPUTS, 0, 1, 0, 143)                      \
  PARAM_ENTRY(CAT_ATF, AtfRMin, "ohm", 0, 200000, 30, 144)                     \
  PARAM_ENTRY(CAT_ATF, AtfRMax, "ohm", 0, 200000, 100000, 145)                 \
  PARAM_ENTRY(CAT_VALVE, MaxOpenLoad, ONOFF, 0, 1, 1, 45)                      \
  PARAM_ENTRY(CAT_VALVE, IMaxCoil, "mA", 100, 1000, 850, 46)                   \
  PARAM_ENTRY(CAT_VALVE, IMinCoil, "mA", 0, 200, 50, 47)                       \
  PARAM_ENTRY(CAT_VALVE, SlewRate, "mA/10ms", 0, 500, 60, 48)                  \
  PARAM_ENTRY(CAT_VALVE, TempCorr, ONOFF, 0, 1, 1, 52)                         \
  PARAM_ENTRY(CAT_VALVE, PosCurrent, "mA", 0, 1000, 0, 55)                     \
  PARAM_ENTRY(CAT_VALVE, ChanA, "ch", 0, 255, 0, 60)                           \
  PARAM_ENTRY(CAT_VALVE, ChanB, "ch", 0, 255, 1, 61)                           \
  PARAM_ENTRY(CAT_VALVE, ChanC, "ch", 0, 255, 2, 62)                           \
  PARAM_ENTRY(CAT_VALVE, ChanD, "ch", 0, 255, 3, 63)                           \
  PARAM_ENTRY(CAT_VALVE, ChanE, "ch", 0, 255, 8, 64)                           \
  PARAM_ENTRY(CAT_VALVE, ChanSYS, "ch", 0, 255, 9, 65)                         \
  PARAM_ENTRY(CAT_VALVE, ChanK0, "ch", 0, 255, 11, 66)                         \
  PARAM_ENTRY(CAT_VALVE, ChanPOS, "ch", 0, 255, 10, 67)                        \
  PARAM_ENTRY(CAT_VALVE, ChanPZYL, "ch", 0, 255, 5, 68)                        \
  PARAM_ENTRY(CAT_VALVE, Ch7Mode, CH7MODES, 0, 3, 0, 174)                      \
  PARAM_ENTRY(CAT_VALVE, K0Press, "mbar", 0, 20000, 0, 175)                    \
  PARAM_ENTRY(CAT_VALVE, TccPress, "mbar", 0, 12000, 6000, 176)                \
  PARAM_ENTRY(CAT_VALVE, TccOnRpm, "rpm", 0, 8000, 1500, 177)                  \
  PARAM_ENTRY(CAT_VALVE, TccOffRpm, "rpm", 0, 8000, 1200, 178)                 \
  PARAM_ENTRY(CAT_VALVE, TccMinGear, "", 1, 8, 2, 179)                         \
  PARAM_ENTRY(CAT_VALVE, TccRamp, "mbar/10ms", 0, 12000, 200, 180)             \
  PARAM_ENTRY(CAT_VALVE, TccMode, TCCMODES, 0, 1, 0, 252)                      \
  PARAM_ENTRY(CAT_VALVE, TccMinRpm, "rpm", 0, 5000, 1100, 253)                 \
  PARAM_ENTRY(CAT_VALVE, TccHyst, "rpm", 0, 2000, 200, 254)                    \
  PARAM_ENTRY(CAT_VALVE, TccKp, "0.1mbar/rpm", 0, 1000, 10, 255)               \
  PARAM_ENTRY(CAT_VALVE, TccKi, "0.1mbar/rpms", 0, 1000, 20, 256)              \
  PARAM_ENTRY(CAT_VALVE, TccStartP, "mbar", 0, 12000, 1500, 257)               \
  PARAM_ENTRY(CAT_VALVE, TccSlipTgt, "rpm", 0, 500, 0, 258)                    \
  PARAM_ENTRY(CAT_VALVE, TccSlipGear, "", 1, 8, 3, 259)                        \
  PARAM_ENTRY(CAT_VALVE, TccSlipRpm, "rpm", 0, 6000, 1800, 260)                \
  PARAM_ENTRY(CAT_VALVE, PzylCurrent, "mA", 0, 1000, 400, 188)                 \
  PARAM_ENTRY(CAT_SHIFT, ClutchApplyP, "mbar", 5000, 25000, 16500, 80)         \
  PARAM_ENTRY(CAT_SHIFT, LineOffset, "mbar", -5000, 10000, 0, 81)              \
  PARAM_ENTRY(CAT_SHIFT, LineMax, "mbar", 5000, 30000, 25000, 82)              \
  PARAM_ENTRY(CAT_SHIFT, ShiftPMax, "mbar", 5000, 20000, 12750, 83)            \
  PARAM_ENTRY(CAT_SHIFT, SyncBand, "rpm", 5, 500, 30, 84)                      \
  PARAM_ENTRY(CAT_SHIFT, GlsGain, "%", 0, 200, 100, 85)                        \
  PARAM_ENTRY(CAT_SHIFT, RampMaxDown, "ms", 100, 5000, 1000, 86)               \
  PARAM_ENTRY(CAT_SHIFT, EngageMaxRpm, "rpm", 0, 2000, 300, 87)                \
  PARAM_ENTRY(CAT_SHIFT, EngageFillEq, "ms", 0, 1000, 50, 88)                  \
  PARAM_ENTRY(CAT_SHIFT, EngageRamp, "ms", 0, 3000, 300, 89)                   \
  PARAM_ENTRY(CAT_SHIFT, EngageCloseGrad, "mbar/s", 20000, 50000, 30000, 90)   \
  PARAM_ENTRY(CAT_SHIFT, AutoUpRpm, "rpm", 500, 10000, 4000, 91)               \
  PARAM_ENTRY(CAT_SHIFT, AutoDownRpm, "rpm", 200, 8000, 1500, 92)              \
  PARAM_ENTRY(CAT_SHIFT, ManMaxRpm, "rpm", 1000, 10000, 6000, 133)             \
  PARAM_ENTRY(CAT_DRIVE, DriveMode, DRIVEMODES, 0, 2, 0, 204)                 \
  PARAM_ENTRY(CAT_DRIVE, SportUpOfs, "rpm", -2000, 3000, 300, 205)             \
  PARAM_ENTRY(CAT_DRIVE, SportDnOfs, "rpm", -2000, 3000, 200, 206)             \
  PARAM_ENTRY(CAT_DRIVE, SportTime, "%", 50, 100, 100, 207)                    \
  PARAM_ENTRY(CAT_DRIVE, SportTccOfs, "rpm", 0, 3000, 200, 208)                \
  PARAM_ENTRY(CAT_DRIVE, TrackRedline, "rpm", 1000, 10000, 5500, 209)          \
  PARAM_ENTRY(CAT_DRIVE, TrackDnOfs, "rpm", -2000, 3000, 400, 210)             \
  PARAM_ENTRY(CAT_DRIVE, TrackTime, "%", 50, 100, 100, 211)                    \
  PARAM_ENTRY(CAT_DRIVE, TrackTccOfs, "rpm", 0, 3000, 300, 212)                \
  PARAM_ENTRY(CAT_DRIVE, TrackLiftNm, "Nm", -200, 500, 10, 213)                \
  PARAM_ENTRY(CAT_SHIFT, ManMinRpm, "rpm", 0, 5000, 1000, 134)                 \
  PARAM_ENTRY(CAT_SHIFT, TqInterv, TQINTERV, 0, 2, 1, 93)                      \
  PARAM_ENTRY(CAT_SHIFT, TqRamp, "ms", 0, 1000, 100, 94)                       \
  PARAM_ENTRY(CAT_SHIFT, DntTau, "ms", 0, 500, 70, 95)                         \
  PARAM_ENTRY(CAT_SHIFT, GearChkPct, "%", 1, 50, 10, 96)                       \
  PARAM_ENTRY(CAT_SHIFT, GearChkMs, "ms", 0, 5000, 500, 97)                    \
  PARAM_ENTRY(CAT_SHIFT, GearChkMinNab, "rpm", 50, 2000, 200, 98)              \
  PARAM_ENTRY(CAT_SHIFT, UvLimit, "mV", 0, 12000, 7400, 99)                    \
  PARAM_ENTRY(CAT_SHIFT, VbatScale, "mV", 1000, 60000, 18330, 100)             \
  PARAM_ENTRY(CAT_SHIFT, EngageSyncMs, "ms", 50, 100, 80, 101)                 \
  PARAM_ENTRY(CAT_SHIFT, EngageTqZero, "Nm", 0, 100, 10, 103)                  \
  PARAM_ENTRY(CAT_SHIFT, IlOpenLow, "mA", 0, 200, 60, 104)                     \
  PARAM_ENTRY(CAT_SHIFT, IlOpenHigh, "mA", 500, 1000, 790, 105)                \
  PARAM_ENTRY(CAT_SHIFT, ParkRelRpm, "rpm", 0, 3000, 700, 183)                 \
  PARAM_ENTRY(CAT_SHIFT, ParkRelP, "mbar", 0, 30000, 17000, 184)               \
  PARAM_ENTRY(CAT_SHIFT, ParkRelHold, "ms", 0, 10000, 2000, 185)               \
  PARAM_ENTRY(CAT_SHIFT, ParkPNDelay, "ms", 0, 2000, 150, 186)                 \
  PARAM_ENTRY(CAT_SHIFT, ParkCtrl, ONOFF, 0, 1, 0, 187)                        \
  PARAM_ENTRY(CAT_SHIFT, ParkMagMin, "ms", 0, 1000, 100, 189)                  \
  PARAM_ENTRY(CAT_SHIFT, ParkMagMax, "ms", 0, 1000, 1000, 190)                 \
  PARAM_ENTRY(CAT_SHIFT, ParkEngKmh, "km/h", 0, 30, 5, 267)                    \
  PARAM_ENTRY(CAT_SHIFT, NabPerKmh, "rpm", 0, 200, 25, 268)                    \
  PARAM_ENTRY(CAT_SHIFT, ParkEngNab, "rpm", 0, 100, 5, 269)                    \
  PARAM_ENTRY(CAT_SHIFT, ParkBrake, PARKBRAKES, 0, 3, 0, 270)                  \
  PARAM_ENTRY(CAT_SHIFT, BrakeIn, ONOFF, 0, 1, 0, 271)                         \
  PARAM_ENTRY(CAT_SHIFT, GearInMinP, "mbar", 0, 20000, 8000, 191)              \
  PARAM_ENTRY(CAT_SHIFT, GearInPumpMs, "ms", 0, 10000, 1000, 192)              \
  PARAM_ENTRY(CAT_SHIFT, GearInMotRpm, "rpm", 0, 3000, 100, 193)               \
  PARAM_ENTRY(CAT_COMM, CanTimeout, "ms", 50, 5000, 500, 112)                  \
  PARAM_ENTRY(CAT_COMM, CanTxEnable, ONOFF, 0, 1, 0, 113)                      \
  PARAM_ENTRY(CAT_COMM, CanLossTq, "Nm", 0, 1000, 600, 114)                    \
  PARAM_ENTRY(CAT_COMM, Can1Baud, CANBAUDS, 0, 4, 2, 116)                      \
  PARAM_ENTRY(CAT_COMM, Can2Baud, CANBAUDS, 0, 4, 2, 119)                      \
  PARAM_ENTRY(CAT_COMM, Can2Enable, ONOFF, 0, 1, 1, 163)                       \
  PARAM_ENTRY(CAT_COMM, LeverBus, LEVERBUSES, 0, 1, 0, 217)                    \
  PARAM_ENTRY(CAT_COMM, StreamEnable, ONOFF, 0, 1, 0, 261)                     \
  PARAM_ENTRY(CAT_IO, GpOut1Func, GPOUTFUNCS, 0, 2, 0, 147)                    \
  PARAM_ENTRY(CAT_IO, GpOut2Func, GPOUTFUNCS, 0, 2, 0, 148)                    \
  PARAM_ENTRY(CAT_IO, GpPwmFreq, "Hz", 10, 20000, 200, 150)                    \
  PARAM_ENTRY(CAT_IO, GpPwmDuty, "%", 0, 100, 0, 151)                          \
  PARAM_ENTRY(CAT_LIN, LinPumpEnable, ONOFF, 0, 1, 0, 152)                     \
  PARAM_ENTRY(CAT_LIN, LinPumpReq, LINREQS, 0, 2, 1, 153)                      \
  PARAM_ENTRY(CAT_LIN, LinPumpRpm, "rpm", 0, 3000, 500, 154)                   \
  PARAM_ENTRY(CAT_LIN, LinPumpP, "mbar", 0, 10000, 4000, 167)                  \
  PARAM_ENTRY(CAT_LIN, PumpFaultAct, PUMPFAULTS, 0, 2, 0, 194)                 \
  PARAM_ENTRY(CAT_ADAPT, AdaptEnable, ONOFF, 0, 1, 0, 229)                     \
  PARAM_ENTRY(CAT_ADAPT, AdaStep, "mbar", 0, 50, 5, 243)                       \
  PARAM_ENTRY(CAT_ADAPT, AdaMax, "mbar", 0, 1000, 300, 244)                    \
  PARAM_ENTRY(CAT_DIAG, DiagRBand, "%", 5, 100, 25, 245)                       \
  PARAM_ENTRY(CAT_DIAG, DiagRWire, "mOhm", 0, 2000, 150, 246)                  \
  PARAM_ENTRY(CAT_DIAG, TccSlipMax, "rpm", 0, 2000, 0, 247)                    \
  PARAM_ENTRY(CAT_DIAG, TccSlipMs, "ms", 0, 10000, 1000, 248)                  \
  PARAM_ENTRY(CAT_DIAG, CalSigExp, "", 0, 65535, 0, 251)                       \
  PARAM_ENTRY(CAT_DIAG, DiagSpdOpen, "mV", 0, 3300, 0, 264)                    \
  PARAM_ENTRY(CAT_DIAG, DiagSpdShort, "mV", 0, 3300, 0, 265)                   \
  PARAM_ENTRY(CAT_DIAG, DiagSpdNoSig, "rpm", 0, 3000, 300, 266)                \
  PARAM_ENTRY(CAT_TEST, SelectorSet, SELECTORS, 0, 3, 2, 120)                  \
  PARAM_ENTRY(CAT_TEST, GearSet, "", 0, 8, 0, 121)                             \
  PARAM_ENTRY(CAT_TEST, TorqueSet, "Nm", -500, 1000, 0, 122)                   \
  PARAM_ENTRY(CAT_TEST, MotorRpmSet, "rpm", 0, 15000, 0, 123)                  \
  PARAM_ENTRY(CAT_TEST, ValveTest, ONOFF, 0, 1, 0, 124)                        \
  PARAM_ENTRY(CAT_TEST, ValveTestSel, VALVES, 0, 8, 0, 125)                    \
  PARAM_ENTRY(CAT_TEST, ValveTestmA, "mA", 0, 1000, 0, 126)                    \
  PARAM_ENTRY(CAT_TEST, ParkHoldTest, ONOFF, 0, 1, 0, 149)                     \
  VALUE_ENTRY(version, VERSTR, 2000)                                           \
  VALUE_ENTRY(opmode, OPMODES, 2001)                                           \
  VALUE_ENTRY(lasterr, errorListString, 2002)                                  \
  VALUE_ENTRY(gear, GEARS, 2003)                                               \
  VALUE_ENTRY(target, GEARS, 2004)                                             \
  VALUE_ENTRY(selector, SELECTORS, 2005)                                       \
  VALUE_ENTRY(tcufault, TCUFAULTS, 2006)                                       \
  VALUE_ENTRY(interlock, ILRESULTS, 2007)                                      \
  VALUE_ENTRY(applied, "bits", 2008)                                           \
  VALUE_ENTRY(zkzu, "", 2009)                                                  \
  VALUE_ENTRY(zkab, "", 2010)                                                  \
  VALUE_ENTRY(engphase, ENGPHASES, 2011)                                       \
  VALUE_ENTRY(nIn, "rpm", 2012)                                                \
  VALUE_ENTRY(nOut, "rpm", 2013)                                               \
  VALUE_ENTRY(nAux, "rpm", 2014)                                               \
  VALUE_ENTRY(nMot, "rpm", 2015)                                               \
  VALUE_ENTRY(mTq, "Nm", 2016)                                                 \
  VALUE_ENTRY(tqDelta, "Nm", 2017)                                             \
  VALUE_ENTRY(atfTemp, "C", 2018)                                              \
  VALUE_ENTRY(atfAdc, "dig", 2019)                                             \
  VALUE_ENTRY(vbat, "mV", 2020)                                                \
  VALUE_ENTRY(pA, "mbar", 2021)                                                \
  VALUE_ENTRY(pB, "mbar", 2022)                                                \
  VALUE_ENTRY(pC, "mbar", 2023)                                                \
  VALUE_ENTRY(pD, "mbar", 2024)                                                \
  VALUE_ENTRY(pE, "mbar", 2025)                                                \
  VALUE_ENTRY(pSys, "mbar", 2026)                                              \
  VALUE_ENTRY(iA, "mA", 2027)                                                  \
  VALUE_ENTRY(iB, "mA", 2028)                                                  \
  VALUE_ENTRY(iC, "mA", 2029)                                                  \
  VALUE_ENTRY(iD, "mA", 2030)                                                  \
  VALUE_ENTRY(iE, "mA", 2031)                                                  \
  VALUE_ENTRY(iSYS, "mA", 2032)                                                \
  VALUE_ENTRY(iK0, "mA", 2033)                                                 \
  VALUE_ENTRY(tSyn, "ms", 2034)                                                \
  VALUE_ENTRY(gls, "mbar", 2035)                                               \
  VALUE_ENTRY(drvFlags0, "bits", 2036)                                         \
  VALUE_ENTRY(valveFault, "bits", 2038)                                        \
  VALUE_ENTRY(canAct, ONOFF, 2039)                                             \
  VALUE_ENTRY(cpuload, "%", 2040)                                              \
  VALUE_ENTRY(tqHold, ONOFF, 2041)                                             \
  VALUE_ENTRY(canHold, ONOFF, 2042)                                            \
  VALUE_ENTRY(coast, ONOFF, 2043)                                              \
  VALUE_ENTRY(atfOhm, "ohm", 2044)                                             \
  VALUE_ENTRY(anaIn, "mV", 2045)                                               \
  VALUE_ENTRY(dinBits, "bits", 2046)                                           \
  VALUE_ENTRY(shFault, ONOFF, 2047)                                            \
  VALUE_ENTRY(spdInLo, "mV", 2048)                                             \
  VALUE_ENTRY(spdInHi, "mV", 2049)                                             \
  VALUE_ENTRY(spdOutLo, "mV", 2050)                                            \
  VALUE_ENTRY(spdOutHi, "mV", 2051)                                            \
  VALUE_ENTRY(lpState, LPSTATES, 2052)                                         \
  VALUE_ENTRY(lpRpm, "rpm", 2054)                                              \
  VALUE_ENTRY(lpVolt, "mV", 2055)                                              \
  VALUE_ENTRY(lpOn, ONOFF, 2060)                                               \
  VALUE_ENTRY(lpOpPt, "hex", 2053)                                             \
  VALUE_ENTRY(lpCurr, "mA", 2056)                                              \
  VALUE_ENTRY(lpClass, "bits", 2057)                                           \
  VALUE_ENTRY(lpEb0, "hex", 2058)                                              \
  VALUE_ENTRY(lpEb1, "hex", 2059)                                              \
  VALUE_ENTRY(lpOwn, "bits", 2061)                                             \
  VALUE_ENTRY(lpTempRaw, "dig", 2062)                                          \
  VALUE_ENTRY(lpHw, "hex", 2063)                                               \
  VALUE_ENTRY(lpSw, "hex", 2064)                                               \
  VALUE_ENTRY(lpIdHi, "hex", 2065)                                             \
  VALUE_ENTRY(lpIdLo, "hex", 2066)                                             \
  VALUE_ENTRY(lpChkErr, "", 2067)                                              \
  VALUE_ENTRY(lpCrcErr, "", 2068)                                              \
  VALUE_ENTRY(lpAlvErr, "", 2069)                                              \
  VALUE_ENTRY(lpPress, "mbar", 2070)                                           \
  VALUE_ENTRY(nMotReq, "rpm", 2071)                                            \
  VALUE_ENTRY(parkWait, ONOFF, 2072)                                           \
  VALUE_ENTRY(gearInWait, ONOFF, 2073)                                         \
  VALUE_ENTRY(tipAuto, ONOFF, 2074)                                            \
  VALUE_ENTRY(pCh7, "mbar", 2075)                                              \
  VALUE_ENTRY(iPZYL, "mA", 2076)                                               \
  VALUE_ENTRY(parkMag, ONOFF, 2077)                                            \
  VALUE_ENTRY(ch7Act, CH7MODES, 2078)                                          \
  VALUE_ENTRY(t15, ONOFF, 2079)                                                \
  VALUE_ENTRY(iPOS, "mA", 2080)                                                \
  VALUE_ENTRY(itA, "mA", 2081)                                                 \
  VALUE_ENTRY(itB, "mA", 2082)                                                 \
  VALUE_ENTRY(itC, "mA", 2083)                                                 \
  VALUE_ENTRY(itD, "mA", 2084)                                                 \
  VALUE_ENTRY(itE, "mA", 2085)                                                 \
  VALUE_ENTRY(itSYS, "mA", 2086)                                               \
  VALUE_ENTRY(itCh7, "mA", 2087)                                               \
  VALUE_ENTRY(itPOS, "mA", 2088)                                               \
  VALUE_ENTRY(itPZYL, "mA", 2089)                                              \
  VALUE_ENTRY(fA, "bits", 2090)                                                \
  VALUE_ENTRY(fB, "bits", 2091)                                                \
  VALUE_ENTRY(fC, "bits", 2092)                                                \
  VALUE_ENTRY(fD, "bits", 2093)                                                \
  VALUE_ENTRY(fE, "bits", 2094)                                                \
  VALUE_ENTRY(fSYS, "bits", 2095)                                              \
  VALUE_ENTRY(fCh7, "bits", 2096)                                              \
  VALUE_ENTRY(fPOS, "bits", 2097)                                              \
  VALUE_ENTRY(fPZYL, "bits", 2098)                                             \
  VALUE_ENTRY(tBoard, "C", 2099)                                               \
  VALUE_ENTRY(ratio, "x1000", 2100)                                            \
  VALUE_ENTRY(slip, "rpm", 2101)                                               \
  VALUE_ENTRY(selGear, GEARS, 2102)                                            \
  VALUE_ENTRY(inertia, ONOFF, 2103)                                            \
  VALUE_ENTRY(drvMode, DRIVEMODES, 2104)                                       \
  VALUE_ENTRY(shiftMd, SHIFTMODES, 2105)                                       \
  VALUE_ENTRY(pSysMeas, "mbar", 2106)                                          \
  VALUE_ENTRY(lpBusFlt, ONOFF, 2107)                                           \
  VALUE_ENTRY(parkSt, PARKSTATES, 2108)                                        \
  VALUE_ENTRY(canLoss, ONOFF, 2109)                                            \
  VALUE_ENTRY(solSetAct, SOLSETS, 2112)                                        \
  VALUE_ENTRY(tqClass, "Nm", 2113)                                             \
  VALUE_ENTRY(revLimNm, "Nm", 2114)                                            \
  VALUE_ENTRY(safeOfsNm, "Nm", 2115)                                           \
  VALUE_ENTRY(k0Present, ONOFF, 2116)                                          \
  VALUE_ENTRY(tBoardSrc, TBOARDSRCS, 2117)                                     \
  VALUE_ENTRY(lvMove, "", 2118)                                                \
  VALUE_ENTRY(lvPBtn, ONOFF, 2119)                                             \
  VALUE_ENTRY(lvState, LEVERSTATES, 2120)                                      \
  VALUE_ENTRY(lvCrcErr, "", 2121)                                              \
  VALUE_ENTRY(lvCntErr, "", 2122)                                              \
  VALUE_ENTRY(lvTmo, ONOFF, 2123)                                              \
  VALUE_ENTRY(tqSrc, TQSRCS, 2124)                                             \
  VALUE_ENTRY(tqUsed, "Nm", 2125)                                              \
  VALUE_ENTRY(tqFrozen, ONOFF, 2126)                                           \
  VALUE_ENTRY(adaStat, "", 2127)                                               \
  VALUE_ENTRY(adaShifts, "", 2128)                                             \
  VALUE_ENTRY(adA_c, "mbar", 2129)                                             \
  VALUE_ENTRY(adA_w, "mbar", 2130)                                             \
  VALUE_ENTRY(adB_c, "mbar", 2131)                                             \
  VALUE_ENTRY(adB_w, "mbar", 2132)                                             \
  VALUE_ENTRY(adC_c, "mbar", 2133)                                             \
  VALUE_ENTRY(adC_w, "mbar", 2134)                                             \
  VALUE_ENTRY(adD_c, "mbar", 2135)                                             \
  VALUE_ENTRY(adD_w, "mbar", 2136)                                             \
  VALUE_ENTRY(adE_c, "mbar", 2137)                                             \
  VALUE_ENTRY(adE_w, "mbar", 2138)                                             \
  VALUE_ENTRY(dgA, DIAGCODES, 2139)                                            \
  VALUE_ENTRY(dgB, DIAGCODES, 2140)                                            \
  VALUE_ENTRY(dgC, DIAGCODES, 2141)                                            \
  VALUE_ENTRY(dgD, DIAGCODES, 2142)                                            \
  VALUE_ENTRY(dgE, DIAGCODES, 2143)                                            \
  VALUE_ENTRY(dgSys, DIAGCODES, 2144)                                          \
  VALUE_ENTRY(dgCh7, DIAGCODES, 2145)                                          \
  VALUE_ENTRY(dgPos, DIAGCODES, 2146)                                          \
  VALUE_ENTRY(dgPzyl, DIAGCODES, 2147)                                         \
  VALUE_ENTRY(spdInDiag, SPDDIAGS, 2149)                                       \
  VALUE_ENTRY(spdOutDiag, SPDDIAGS, 2150)                                      \
  VALUE_ENTRY(canBus, "", 2151)                                                \
  VALUE_ENTRY(calSig, "", 2152)                                                \
  VALUE_ENTRY(tccSlip, "rpm", 2153)                                            \
  VALUE_ENTRY(tccSlipFlt, ONOFF, 2154)                                         \
  VALUE_ENTRY(parkInh, ONOFF, 2155)                                            \
  VALUE_ENTRY(brake, ONOFF, 2156)                                              \
  VALUE_ENTRY(parkBrkW, ONOFF, 2157)                                           \
  VALUE_ENTRY(drvFlags1, "bits", 2158)                                         \
  VALUE_ENTRY(spdLoad, "0.1%", 2160)                                           \
  VALUE_ENTRY(brakePin, ONOFF, 2161)                                          \
  VALUE_ENTRY(paramSrc, PARAMSRCS, 2162)                                       \
  VALUE_ENTRY(bootStage, "stage", 2163)                                        \
  VALUE_ENTRY(imA, "mA", 2164)                                                 \
  VALUE_ENTRY(imB, "mA", 2165)                                                 \
  VALUE_ENTRY(imC, "mA", 2166)                                                 \
  VALUE_ENTRY(imD, "mA", 2167)                                                 \
  VALUE_ENTRY(imE, "mA", 2168)                                                 \
  VALUE_ENTRY(imSYS, "mA", 2169)                                               \
  VALUE_ENTRY(imCh7, "mA", 2170)                                               \
  VALUE_ENTRY(imPOS, "mA", 2171)                                               \
  VALUE_ENTRY(imPZYL, "mA", 2172)                                              \
  VALUE_ENTRY(imPHOLD, "mA", 2173)                                             \
  VALUE_ENTRY(iPHOLD, "mA", 2174)                                              \
  VALUE_ENTRY(tleSup, "bits", 2175)                                            \
  VALUE_ENTRY(tleVbat, "mV", 2176)                                             \
  VALUE_ENTRY(tleTemp, "C", 2177)                                              \
  VALUE_ENTRY(tleComErr, "", 2178)                                             \
  VALUE_ENTRY(parkHold, ONOFF, 2179)                                           \
  VALUE_ENTRY(rA, "mOhm", 2180)                                                \
  VALUE_ENTRY(rB, "mOhm", 2181)                                                \
  VALUE_ENTRY(rC, "mOhm", 2182)                                                \
  VALUE_ENTRY(rD, "mOhm", 2183)                                                \
  VALUE_ENTRY(rE, "mOhm", 2184)                                                \
  VALUE_ENTRY(rSYS, "mOhm", 2185)                                              \
  VALUE_ENTRY(rCh7, "mOhm", 2186)                                              \
  VALUE_ENTRY(rPOS, "mOhm", 2187)                                              \
  VALUE_ENTRY(rPZYL, "mOhm", 2188)                                             \
  VALUE_ENTRY(rPHOLD, "mOhm", 2189)

// Generate the version string. This must be formatted without spaces to meet
// the format requirements of the "units".
// clang-format off
#define VERSTR STRINGIFY(4=VER)
// clang-format on

#define ONOFF "0=Off, 1=On"
// GearboxType: one row per gearbox and calibration (calset.cpp). Ids 0 and
// 1 are kept; new rows are appended, never renumbered.
#define GBOXTYPES                                                              \
  "0=8HP75_330e, 1=8HP45_F11, 2=8HP45_F11N47_v095, 3=8HP70_N57_v095, "         \
  "4=8HP70_N63_v095, 5=8HP90_N74_v095, 6=8HP90_RR4N74_v095"
// SolCurveSet: solenoid p->I curves, per unit (valve hardware revision).
#define SOLSETS "0=Auto, 1=330e_Q030, 2=F11_v058, 3=v095_8HP90, 4=v095_8HP45_70"
#define TBOARDSRCS "0=Die, 1=NTC"
// EcuType: who the TCU talks to on CAN1 (canif.cpp handler table). Only
// the placeholder VCU protocol for now; OEM engine ECUs will be added later.
#define ECUTYPES "0=None/VCU"
#define MANGEARS                                                               \
  "0=Selector, 1=P, 2=R, 3=N, 4=1st, 5=2nd, 6=3rd, 7=4th, 8=5th, 9=6th, "     \
  "10=7th, 11=8th"
#define CH7MODES "0=Variant, 1=TCC, 2=K0, 3=ForceOpen"
#define PUMPFAULTS "0=Report, 1=MotorSpeed, 2=Stop"
#define SHIFTMODES "0=Auto, 1=Manual, 2=VCU"
#define DRIVEMODES "0=Normal, 1=Sport, 2=Track"
#define PARKSTATES "0=Engaged, 1=Releasing, 2=Released, 3=Engaging, 4=Unheld"
// Where the footbrake for the park release comes from (ParkBrakeSrc in
// tcu.h). Input = the 12 V brake switch input on PD1.
#define PARKBRAKES "0=VcuCan, 1=Param, 2=Bypass, 3=Input"
#define PARAMSRCS "0=Defaults, 1=Flash, 2=Foreign"
// ShifterType: 0 and 1 keep their ids (0 = CAN is now the BMW GWS lever,
// with the VCU-frame selector as fallback until the lever is seen).
#define SHIFTERS                                                               \
  "0=BMW_GWS, 1=Analog, 2=BMW_E65, 3=JLR_G1, 4=JLR_G2, 5=PKP2300, "            \
  "6=VCU_Frame"
#define LEVERBUSES "0=CAN1, 1=CAN2"
#define TCCMODES "0=Simple, 1=Schedule"
#define TQSRCS "0=CAN, 1=Model, 2=Substitute, 3=Param"
#define DIAGCODES "0=OK, 1=Open, 2=ShortBat, 3=ShortGnd, 4=RHigh, 5=RLow"
#define SPDDIAGS "0=OK, 1=Open, 2=Short, 3=NoSignal"
#define LEVERSTATES "0=P, 1=R, 2=N, 3=D, 4=M"
#define ATFINPUTS "0=TempIn1, 1=TempIn2"
#define GPOUTFUNCS "0=Off, 1=On, 2=StarterLock"
#define LINREQS "0=Off, 1=Auto, 2=Force"
#define LPSTATES "0=Off, 1=Idle, 2=Run"
#define CMDSRC "0=CAN, 1=Param"
#define NTSRC "0=Sensor, 1=CanMotorRpm"
#define ATFSRC "0=Fixed, 1=Sensor"
#define TQINTERV "0=Off, 1=Upshift, 2=UpAndDown"
#define CANBAUDS "0=125k, 1=250k, 2=500k, 3=800k, 4=1M"
#define SELECTORS "0=P, 1=R, 2=N, 3=D"
#define VALVES                                                                 \
  "0=A, 1=B, 2=C, 3=D, 4=E, 5=SYS, 6=K0/TCC, 7=POS, 8=PZYL"
#define OPMODES                                                                \
  "0=Init, 1=Park, 2=Neutral, 3=Engage, 4=Drive, 5=Shift, 6=Safe"
#define GEARS "0=N, 1=1st, 2=2nd, 3=3rd, 4=4th, 5=5th, 6=6th, 7=7th, 8=8th, 9=R"
#define TCUFAULTS                                                              \
  "0=None, 1=Interlock, 2=Driver, 3=Undervolt, 4=Speed, 5=Config"
#define ILRESULTS "0=OK, 1=NotAllowed, 2=TooMany, 3=NotAGear"
#define ENGPHASES "0=Idle, 1=Fill, 2=Power, 3=Close, 4=Done"

#define CAT_SETUP "General Setup"
#define CAT_SPEED "Speed Sensors"
#define CAT_ATF "ATF Temperature"
#define CAT_VALVE "Valve Driver"
#define CAT_SHIFT "Shift Control"
#define CAT_DRIVE "Drive Modes"
#define CAT_COMM "Communication"
#define CAT_IO "Spare Outputs"
#define CAT_LIN "LIN Oil Pump"
#define CAT_TEST "Testing"
#define CAT_ADAPT "Adaptation"
#define CAT_DIAG "Diagnostics"

/// Value of a ChanX parameter for a valve that is not on any driver channel.
#define CHAN_NOT_FITTED 255

extern const char *errorListString;
#endif // PARAM_PRJ_H_INCLUDED
