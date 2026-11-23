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
#include "test.h"
#include "param_prj.h"
#include <string.h>

/* Parameter and spot value ids. openinverter stores parameters by id and
 * the web UI / logging tools address values by name and id, so:
 *  - every id and every name is unique;
 *  - parameters are below 2000, spot values from 2000 up;
 *  - retired ids are never used again;
 *  - an id, once given, keeps its name (the frozen table below: add new
 *    entries to it, never change or remove one);
 *  - "Next param id" / "Next value id" in param_prj.h stay above every id.
 */

struct Entry {
  const char *name;
  int id;
  bool value;
};

#define PARAM_ENTRY(cat, name, unit, min, max, def, id) {#name, id, false},
#define VALUE_ENTRY(name, unit, id) {#name, id, true},
static const Entry entries[] = {PARAM_LIST};
#undef PARAM_ENTRY
#undef VALUE_ENTRY

static const int kNextParamId = 217; // keep equal to param_prj.h
static const int kNextValueId = 2118;

static const int retired[] = {1, 22, 23, 24, 25, 26, 27, 34, 35, 40, 42, 53, 54, 69, 70, 102, 106, 107, 157, 158, 159, 160, 161, 162, 2037};

struct Frozen {
  const char *name;
  int id;
};
static const Frozen frozen[] = {
    {"TopGear", 2}, {"CmdSource", 3}, {"NtSource", 7},
    {"FaultClear", 8}, {"GearboxType", 9}, {"ShiftMode", 127},
    {"ShifterType", 128}, {"ShAnaR", 129}, {"ShAnaN", 130},
    {"ShAnaMin", 131}, {"ShAnaMax", 132}, {"ManualGear", 171},
    {"TipOverride", 172}, {"Tip12Deb", 173}, {"EcuType", 182},
    {"TeethIn", 10}, {"TeethOut", 11}, {"TeethAux", 12},
    {"SpdTimeout", 13}, {"SpdMinRpm", 14}, {"SpdAdcLo", 135},
    {"SpdAdcHi", 136}, {"AtfSource", 20}, {"AtfFixed", 21},
    {"AtfInput", 143}, {"AtfR1", 137}, {"AtfR2", 138},
    {"AtfR3", 139}, {"AtfR4", 140}, {"AtfR5", 141},
    {"AtfR6", 142}, {"AtfRMin", 144}, {"AtfRMax", 145},
    {"AtfT1", 28}, {"AtfT2", 29}, {"AtfT3", 30},
    {"AtfT4", 31}, {"AtfT5", 32}, {"AtfT6", 33},
    {"MaxIfs", 41}, {"MaxHfsMask", 146}, {"MaxFreq", 43},
    {"MaxFreqDiv", 44}, {"MaxOpenLoad", 45}, {"IMaxCoil", 46},
    {"IMinCoil", 47}, {"SlewRate", 48}, {"DitherAmp", 49},
    {"DitherPeriod", 50}, {"DitherShape", 51}, {"TempCorr", 52},
    {"PosCurrent", 55}, {"ChanA", 60}, {"ChanB", 61},
    {"ChanC", 62}, {"ChanD", 63}, {"ChanE", 64},
    {"ChanSYS", 65}, {"ChanK0", 66}, {"ChanPOS", 67},
    {"ChanPZYL", 68}, {"Ch7Mode", 174}, {"K0Press", 175},
    {"TccPress", 176}, {"TccOnRpm", 177}, {"TccOffRpm", 178},
    {"TccMinGear", 179}, {"TccRamp", 180}, {"PzylCurrent", 188},
    {"ClutchApplyP", 80}, {"LineOffset", 81}, {"LineMax", 82},
    {"ShiftPMax", 83}, {"SyncBand", 84}, {"GlsGain", 85},
    {"RampMaxDown", 86}, {"EngageMaxRpm", 87}, {"EngageFillEq", 88},
    {"EngageRamp", 89}, {"EngageCloseGrad", 90}, {"AutoUpRpm", 91},
    {"AutoDownRpm", 92}, {"ManMaxRpm", 133}, {"DriveMode", 204},
    {"SportUpOfs", 205}, {"SportDnOfs", 206}, {"SportTime", 207},
    {"SportTccOfs", 208}, {"TrackRedline", 209}, {"TrackDnOfs", 210},
    {"TrackTime", 211}, {"TrackTccOfs", 212}, {"TrackLiftNm", 213},
    {"ManMinRpm", 134}, {"TqInterv", 93}, {"TqRamp", 94},
    {"DntTau", 95}, {"GearChkPct", 96}, {"GearChkMs", 97},
    {"GearChkMinNab", 98}, {"UvLimit", 99}, {"VbatScale", 100},
    {"EngageSyncMs", 101}, {"EngageTqZero", 103}, {"IlOpenLow", 104},
    {"IlOpenHigh", 105}, {"ParkRelRpm", 183}, {"ParkRelP", 184},
    {"ParkRelHold", 185}, {"ParkPNDelay", 186}, {"ParkCtrl", 187},
    {"ParkMagMin", 189}, {"ParkMagMax", 190}, {"GearInMinP", 191},
    {"GearInPumpMs", 192}, {"GearInMotRpm", 193}, {"CanCmdId", 110},
    {"CanStatId", 111}, {"CanTimeout", 112}, {"CanTxEnable", 113},
    {"CanLossTq", 114}, {"CanLossMode", 108}, {"SdoNodeId", 115},
    {"Can1Baud", 116}, {"LogEnable", 117}, {"LogBaseId", 118},
    {"Can2Baud", 119}, {"Can2Enable", 163}, {"F11TqTx", 181},
    {"TqCeilTx", 215}, {"TqCeilNm", 216}, {"SolCurveSet", 214},
    {"GpOut1Func", 147}, {"GpOut2Func", 148}, {"GpPwmFreq", 150},
    {"GpPwmDuty", 151}, {"LinPumpEnable", 152}, {"LinPumpReq", 153},
    {"LinPumpRpm", 154}, {"LinPumpHyst", 155}, {"LinChecksum", 156},
    {"LinTempMin", 164}, {"LinTempMax", 165}, {"LinPMax", 166},
    {"LinPumpP", 167}, {"LinOffDelay", 168}, {"LinSlotMs", 169},
    {"LinBusFault", 170}, {"LinColdMin", 196}, {"LinColdMs", 197},
    {"LinArmMs", 198}, {"LinSymMax", 199}, {"LinDelivRpm", 200},
    {"LinRxCheck", 201}, {"ChgPlugSrc", 202}, {"ChgPlugSet", 203},
    {"PumpFaultAct", 194}, {"PumpFaultRpm", 195}, {"SelectorSet", 120},
    {"GearSet", 121}, {"TorqueSet", 122}, {"MotorRpmSet", 123},
    {"ValveTest", 124}, {"ValveTestSel", 125}, {"ValveTestmA", 126},
    {"ParkHoldTest", 149}, {"version", 2000}, {"opmode", 2001},
    {"lasterr", 2002}, {"gear", 2003}, {"target", 2004},
    {"selector", 2005}, {"tcufault", 2006}, {"interlock", 2007},
    {"applied", 2008}, {"zkzu", 2009}, {"zkab", 2010},
    {"engphase", 2011}, {"nIn", 2012}, {"nOut", 2013},
    {"nAux", 2014}, {"nMot", 2015}, {"mTq", 2016},
    {"tqDelta", 2017}, {"atfTemp", 2018}, {"atfAdc", 2019},
    {"vbat", 2020}, {"pA", 2021}, {"pB", 2022},
    {"pC", 2023}, {"pD", 2024}, {"pE", 2025},
    {"pSys", 2026}, {"iA", 2027}, {"iB", 2028},
    {"iC", 2029}, {"iD", 2030}, {"iE", 2031},
    {"iSYS", 2032}, {"iK0", 2033}, {"tSyn", 2034},
    {"gls", 2035}, {"drvFlags0", 2036}, {"valveFault", 2038},
    {"canAct", 2039}, {"cpuload", 2040}, {"tqHold", 2041},
    {"canHold", 2042}, {"coast", 2043}, {"atfOhm", 2044},
    {"anaIn", 2045}, {"dinBits", 2046}, {"shFault", 2047},
    {"spdInLo", 2048}, {"spdInHi", 2049}, {"spdOutLo", 2050},
    {"spdOutHi", 2051}, {"lpState", 2052}, {"lpRpm", 2054},
    {"lpVolt", 2055}, {"lpOn", 2060}, {"lpOpPt", 2053},
    {"lpCurr", 2056}, {"lpClass", 2057}, {"lpEb0", 2058},
    {"lpEb1", 2059}, {"lpOwn", 2061}, {"lpTempRaw", 2062},
    {"lpHw", 2063}, {"lpSw", 2064}, {"lpIdHi", 2065},
    {"lpIdLo", 2066}, {"lpChkErr", 2067}, {"lpCrcErr", 2068},
    {"lpAlvErr", 2069}, {"lpPress", 2070}, {"nMotReq", 2071},
    {"parkWait", 2072}, {"gearInWait", 2073}, {"tipAuto", 2074},
    {"pCh7", 2075}, {"iPZYL", 2076}, {"parkMag", 2077},
    {"ch7Act", 2078}, {"t15", 2079}, {"iPOS", 2080},
    {"itA", 2081}, {"itB", 2082}, {"itC", 2083},
    {"itD", 2084}, {"itE", 2085}, {"itSYS", 2086},
    {"itCh7", 2087}, {"itPOS", 2088}, {"itPZYL", 2089},
    {"fA", 2090}, {"fB", 2091}, {"fC", 2092},
    {"fD", 2093}, {"fE", 2094}, {"fSYS", 2095},
    {"fCh7", 2096}, {"fPOS", 2097}, {"fPZYL", 2098},
    {"tBoard", 2099}, {"ratio", 2100}, {"slip", 2101},
    {"selGear", 2102}, {"inertia", 2103}, {"drvMode", 2104},
    {"shiftMd", 2105}, {"pSysMeas", 2106}, {"lpBusFlt", 2107},
    {"parkSt", 2108}, {"canLoss", 2109}, {"tq0B0", 2110},
    {"tqQual", 2111}, {"solSetAct", 2112}, {"tqClass", 2113},
    {"revLimNm", 2114}, {"safeOfsNm", 2115}, {"k0Present", 2116},
    {"tBoardSrc", 2117},
};

static const Entry *Find(const char *name) {
  for (const Entry &e : entries)
    if (strcmp(e.name, name) == 0)
      return &e;
  return 0;
}

void TestParams() {
  const int n = sizeof(entries) / sizeof(entries[0]);
  CHECK(n > 100);
  int dupId = 0, dupName = 0;
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++) {
      if (entries[i].id == entries[j].id)
        dupId++;
      if (strcmp(entries[i].name, entries[j].name) == 0)
        dupName++;
    }
  CHECK_EQ(dupId, 0);
  CHECK_EQ(dupName, 0);

  int badRange = 0, reused = 0;
  for (const Entry &e : entries) {
    if (e.value ? (e.id < 2000 || e.id >= kNextValueId)
                : (e.id <= 0 || e.id >= kNextParamId))
      badRange++;
    for (int r : retired)
      if (e.id == r)
        reused++;
  }
  CHECK_EQ(badRange, 0);
  CHECK_EQ(reused, 0);

  int moved = 0;
  for (const Frozen &f : frozen) {
    const Entry *e = Find(f.name);
    if (!e || e->id != f.id)
      moved++;
  }
  CHECK_EQ(moved, 0);
  CHECK_EQ((int)(sizeof(frozen) / sizeof(frozen[0])), n); // add new ones

  // The spot values the DriveMode / diagnostics change added.
  static const char *const spot[] = {
      "iPOS", "itA", "itB", "itC", "itD", "itE", "itSYS", "itCh7", "itPOS",
      "itPZYL", "fA", "fB", "fC", "fD", "fE", "fSYS", "fCh7", "fPOS", "fPZYL",
      "tBoard", "ratio", "slip", "selGear", "inertia", "drvMode", "shiftMd",
      "pSysMeas", "lpBusFlt", "parkSt", "canLoss", "tq0B0", "tqQual",
      "solSetAct", "tqClass", "revLimNm", "safeOfsNm", "k0Present", "tBoardSrc"};
  int missing = 0;
  for (const char *s : spot) {
    const Entry *e = Find(s);
    if (!e || !e->value || e->id < 2080)
      missing++;
  }
  CHECK_EQ(missing, 0);
  // itA..itPZYL and fA..fPZYL are filled in a loop by offset from itA / fA.
  for (int i = 0; i < 9; i++) {
    CHECK_EQ(Find(spot[1 + i])->id, 2081 + i);
    CHECK_EQ(Find(spot[10 + i])->id, 2090 + i);
  }
  // Variant restructure: GearboxType keeps id 9, SolCurveSet is new.
  const Entry *gt = Find("GearboxType");
  CHECK(gt && !gt->value && gt->id == 9);
  const Entry *ss = Find("SolCurveSet");
  CHECK(ss && !ss->value && ss->id == 214);
  const Entry *dm = Find("DriveMode");
  CHECK(dm && !dm->value && dm->id == 204);
}
