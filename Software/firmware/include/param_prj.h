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
// Next param id (increase when adding a new parameter!): 127
//
// Anything marked TBD below has not been measured yet. The defaults are
// placeholders chosen so the firmware runs on a bench, not values to drive
// a car on. See README.md "Parameters for unmeasured items".
//
/*              category     name         unit       min     max     default id
 */
#define PARAM_LIST                                                             \
  PARAM_ENTRY(CAT_SETUP, GearSelMode, GSELMODES, 0, 1, 0, 1)                   \
  PARAM_ENTRY(CAT_SETUP, TopGear, "", 1, 8, 8, 2)                              \
  PARAM_ENTRY(CAT_SETUP, CmdSource, CMDSRC, 0, 1, 0, 3)                        \
  PARAM_ENTRY(CAT_SETUP, NtSource, NTSRC, 0, 1, 0, 7)                          \
  PARAM_ENTRY(CAT_SETUP, FaultClear, ONOFF, 0, 1, 0, 8)                        \
  PARAM_ENTRY(CAT_SPEED, TeethIn, "", 1, 200, 60, 10)                          \
  PARAM_ENTRY(CAT_SPEED, TeethOut, "", 1, 200, 60, 11)                         \
  PARAM_ENTRY(CAT_SPEED, TeethAux, "", 1, 200, 60, 12)                         \
  PARAM_ENTRY(CAT_SPEED, SpdTimeout, "ms", 10, 130, 100, 13)                  \
  PARAM_ENTRY(CAT_SPEED, SpdMinRpm, "rpm", 0, 500, 30, 14)                     \
  PARAM_ENTRY(CAT_ATF, AtfSource, ATFSRC, 0, 1, 0, 20)                         \
  PARAM_ENTRY(CAT_ATF, AtfFixed, "C", -40, 150, 40, 21)                        \
  PARAM_ENTRY(CAT_ATF, AtfAdc1, "dig", 0, 4095, 400, 22)                       \
  PARAM_ENTRY(CAT_ATF, AtfAdc2, "dig", 0, 4095, 1000, 23)                      \
  PARAM_ENTRY(CAT_ATF, AtfAdc3, "dig", 0, 4095, 1800, 24)                      \
  PARAM_ENTRY(CAT_ATF, AtfAdc4, "dig", 0, 4095, 2600, 25)                      \
  PARAM_ENTRY(CAT_ATF, AtfAdc5, "dig", 0, 4095, 3300, 26)                      \
  PARAM_ENTRY(CAT_ATF, AtfAdc6, "dig", 0, 4095, 3900, 27)                      \
  PARAM_ENTRY(CAT_ATF, AtfT1, "C", -40, 150, 140, 28)                          \
  PARAM_ENTRY(CAT_ATF, AtfT2, "C", -40, 150, 100, 29)                          \
  PARAM_ENTRY(CAT_ATF, AtfT3, "C", -40, 150, 70, 30)                           \
  PARAM_ENTRY(CAT_ATF, AtfT4, "C", -40, 150, 40, 31)                           \
  PARAM_ENTRY(CAT_ATF, AtfT5, "C", -40, 150, 10, 32)                           \
  PARAM_ENTRY(CAT_ATF, AtfT6, "C", -40, 150, -30, 33)                          \
  PARAM_ENTRY(CAT_ATF, AtfAdcMin, "dig", 0, 4095, 50, 34)                      \
  PARAM_ENTRY(CAT_ATF, AtfAdcMax, "dig", 0, 4095, 4050, 35)                    \
  PARAM_ENTRY(CAT_VALVE, MaxChips, "", 1, 2, 2, 40)                            \
  PARAM_ENTRY(CAT_VALVE, MaxIfs, "mA", 150, 1000, 1000, 41)                    \
  PARAM_ENTRY(CAT_VALVE, MaxHfs, ONOFF, 0, 1, 0, 42)                           \
  PARAM_ENTRY(CAT_VALVE, MaxFreq, MAXFREQ, 0, 1, 0, 43)                        \
  PARAM_ENTRY(CAT_VALVE, MaxFreqDiv, MAXDIV, 0, 3, 0, 44)                      \
  PARAM_ENTRY(CAT_VALVE, MaxOpenLoad, ONOFF, 0, 1, 1, 45)                      \
  PARAM_ENTRY(CAT_VALVE, IMaxCoil, "mA", 100, 1000, 850, 46)                   \
  PARAM_ENTRY(CAT_VALVE, IMinCoil, "mA", 0, 200, 50, 47)                       \
  PARAM_ENTRY(CAT_VALVE, SlewRate, "mA/10ms", 0, 500, 60, 48)                  \
  PARAM_ENTRY(CAT_VALVE, DitherAmp, "mA", 0, 100, 30, 49)                      \
  PARAM_ENTRY(CAT_VALVE, DitherPeriod, "ms", 2, 50, 8, 50)                     \
  PARAM_ENTRY(CAT_VALVE, DitherShape, DSHAPE, 0, 1, 1, 51)                     \
  PARAM_ENTRY(CAT_VALVE, TempCorr, ONOFF, 0, 1, 1, 52)                         \
  PARAM_ENTRY(CAT_VALVE, HitCurrent, "mA", 0, 1000, 0, 53)                     \
  PARAM_ENTRY(CAT_VALVE, HitTime, "", 0, 255, 0, 54)                           \
  PARAM_ENTRY(CAT_VALVE, PosCurrent, "mA", 0, 1000, 0, 55)                     \
  PARAM_ENTRY(CAT_VALVE, ChanA, "ch", 0, 255, 0, 60)                           \
  PARAM_ENTRY(CAT_VALVE, ChanB, "ch", 0, 255, 1, 61)                           \
  PARAM_ENTRY(CAT_VALVE, ChanC, "ch", 0, 255, 2, 62)                           \
  PARAM_ENTRY(CAT_VALVE, ChanD, "ch", 0, 255, 3, 63)                           \
  PARAM_ENTRY(CAT_VALVE, ChanE, "ch", 0, 255, 4, 64)                           \
  PARAM_ENTRY(CAT_VALVE, ChanSYS, "ch", 0, 255, 5, 65)                         \
  PARAM_ENTRY(CAT_VALVE, ChanK0, "ch", 0, 255, 6, 66)                          \
  PARAM_ENTRY(CAT_VALVE, ChanPOS, "ch", 0, 255, 7, 67)                         \
  PARAM_ENTRY(CAT_VALVE, ChanPZYL, "ch", 0, 255, 8, 68)                        \
  PARAM_ENTRY(CAT_VALVE, ChanSOL1, "ch", 0, 255, 9, 69)                        \
  PARAM_ENTRY(CAT_VALVE, ChanPARK, "ch", 0, 255, 10, 70)                       \
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
  PARAM_ENTRY(CAT_SHIFT, EngageCloseGrad, "mbar/s", 1000, 100000, 20000, 90)   \
  PARAM_ENTRY(CAT_SHIFT, AutoUpRpm, "rpm", 500, 10000, 4000, 91)               \
  PARAM_ENTRY(CAT_SHIFT, AutoDownRpm, "rpm", 200, 8000, 1500, 92)              \
  PARAM_ENTRY(CAT_SHIFT, TqInterv, TQINTERV, 0, 2, 1, 93)                      \
  PARAM_ENTRY(CAT_SHIFT, TqRamp, "ms", 0, 1000, 100, 94)                       \
  PARAM_ENTRY(CAT_SHIFT, DntTau, "ms", 0, 500, 70, 95)                         \
  PARAM_ENTRY(CAT_SHIFT, GearChkPct, "%", 1, 50, 10, 96)                       \
  PARAM_ENTRY(CAT_SHIFT, GearChkMs, "ms", 0, 5000, 500, 97)                    \
  PARAM_ENTRY(CAT_SHIFT, GearChkMinNab, "rpm", 50, 2000, 200, 98)              \
  PARAM_ENTRY(CAT_SHIFT, UvLimit, "mV", 0, 12000, 7400, 99)                    \
  PARAM_ENTRY(CAT_SHIFT, VbatScale, "mV", 1000, 60000, 23100, 100)             \
  PARAM_ENTRY(CAT_COMM, CanCmdId, "", 1, 2047, 768, 110)                       \
  PARAM_ENTRY(CAT_COMM, CanStatId, "", 1, 2047, 769, 111)                      \
  PARAM_ENTRY(CAT_COMM, CanTimeout, "ms", 50, 5000, 500, 112)                  \
  PARAM_ENTRY(CAT_COMM, CanTxEnable, ONOFF, 0, 1, 0, 113)                      \
  PARAM_ENTRY(CAT_COMM, CanLossTq, "Nm", 0, 1000, 400, 114)                    \
  PARAM_ENTRY(CAT_COMM, SdoNodeId, "", 1, 127, 22, 115)                        \
  PARAM_ENTRY(CAT_COMM, Can1Baud, CANBAUDS, 0, 4, 2, 116)                      \
  PARAM_ENTRY(CAT_COMM, LogEnable, ONOFF, 0, 1, 0, 117)                        \
  PARAM_ENTRY(CAT_COMM, LogBaseId, "", 1, 2040, 1520, 118)                     \
  PARAM_ENTRY(CAT_COMM, Can2Baud, CANBAUDS, 0, 4, 2, 119)                      \
  PARAM_ENTRY(CAT_TEST, SelectorSet, SELECTORS, 0, 3, 2, 120)                  \
  PARAM_ENTRY(CAT_TEST, GearSet, "", 0, 8, 0, 121)                             \
  PARAM_ENTRY(CAT_TEST, TorqueSet, "Nm", -500, 1000, 0, 122)                   \
  PARAM_ENTRY(CAT_TEST, MotorRpmSet, "rpm", 0, 15000, 0, 123)                  \
  PARAM_ENTRY(CAT_TEST, ValveTest, ONOFF, 0, 1, 0, 124)                        \
  PARAM_ENTRY(CAT_TEST, ValveTestSel, VALVES, 0, 10, 0, 125)                   \
  PARAM_ENTRY(CAT_TEST, ValveTestmA, "mA", 0, 1000, 0, 126)                    \
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
  VALUE_ENTRY(drvFlags1, "bits", 2037)                                         \
  VALUE_ENTRY(valveFault, "bits", 2038)                                        \
  VALUE_ENTRY(canAct, ONOFF, 2039)                                             \
  VALUE_ENTRY(cpuload, "%", 2040)

// Generate the version string. This must be formatted without spaces to meet
// the format requirements of the "units".
// clang-format off
#define VERSTR STRINGIFY(4=VER)
// clang-format on

#define ONOFF "0=Off, 1=On"
#define GSELMODES "0=VCU, 1=Auto"
#define CMDSRC "0=CAN, 1=Param"
#define NTSRC "0=Sensor, 1=CanMotorRpm"
#define ATFSRC "0=Fixed, 1=Sensor"
#define MAXFREQ "0=100kHz, 1=80kHz"
#define MAXDIV "0=Div4, 1=Div3, 2=Div2, 3=Div1"
#define DSHAPE "0=Square, 1=Triangle"
#define TQINTERV "0=Off, 1=Upshift, 2=UpAndDown"
#define CANBAUDS "0=125k, 1=250k, 2=500k, 3=800k, 4=1M"
#define SELECTORS "0=P, 1=R, 2=N, 3=D"
#define VALVES                                                                 \
  "0=A, 1=B, 2=C, 3=D, 4=E, 5=SYS, 6=K0, 7=POS, 8=PZYL, 9=SOL1, 10=PARK"
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
#define CAT_COMM "Communication"
#define CAT_TEST "Testing"

/// Value of a ChanX parameter for a valve that is not on any MAX22200.
#define CHAN_NOT_FITTED 255

extern const char *errorListString;
#endif // PARAM_PRJ_H_INCLUDED
