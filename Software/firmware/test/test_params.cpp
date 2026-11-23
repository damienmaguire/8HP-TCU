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
#include "flashguard.h"
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

static const int kNextParamId = 273; // keep equal to param_prj.h
static const int kNextValueId = 2164;

static const int retired[] = {PARAM_RETIRED_IDS};

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
    {"AtfInput", 143}, {"AtfRMin", 144}, {"AtfRMax", 145},
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
    {"LeverBus", 217}, {"LeverTxEnable", 218}, {"LeverTimeout", 219},
    {"LeverDeb", 220}, {"LeverMSub", 221}, {"LeverB34", 222},
    {"GpOut1Func", 147}, {"GpOut2Func", 148}, {"GpPwmFreq", 150},
    {"GpPwmDuty", 151}, {"LinPumpEnable", 152}, {"LinPumpReq", 153},
    {"LinPumpRpm", 154}, {"LinPumpP", 167},
    {"PumpFaultAct", 194}, {"SelectorSet", 120},
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
    {"tBoardSrc", 2117}, {"lvMove", 2118}, {"lvPBtn", 2119},
    {"lvState", 2120}, {"lvCrcErr", 2121}, {"lvCntErr", 2122}, {"lvTmo", 2123},
    {"TqModel", 223}, {"TqModelScale", 224}, {"TqBoostPct", 225},
    {"MapKpa", 226}, {"TqNoRedVal", 227}, {"PedalPct", 228},
    {"AdaptEnable", 229}, {"AdaTqMin", 230}, {"AdaTqMax", 231},
    {"AdaRpmMin", 232}, {"AdaRpmMax", 233}, {"AdaOilMin", 234},
    {"AdaOilMax", 235}, {"AdaGearMin", 236}, {"AdaGearMax", 237},
    {"AdaPedalMin", 238}, {"AdaDriveMin", 239}, {"AdaShiftMs", 240},
    {"AdaBandMs", 241}, {"AdaFlareRpm", 242}, {"AdaStep", 243},
    {"AdaMax", 244}, {"DiagRBand", 245}, {"DiagRWire", 246},
    {"TccSlipMax", 247}, {"TccSlipMs", 248}, {"FrzTime", 249},
    {"FrzRpm", 250}, {"CalSigExp", 251}, {"TccMode", 252},
    {"TccMinRpm", 253}, {"TccHyst", 254}, {"TccKp", 255},
    {"TccKi", 256}, {"TccStartP", 257}, {"TccSlipTgt", 258},
    {"TccSlipGear", 259}, {"TccSlipRpm", 260}, {"StreamEnable", 261},
    {"StreamBaseId", 262}, {"StreamBus", 263}, {"DiagSpdOpen", 264},
    {"DiagSpdShort", 265}, {"DiagSpdNoSig", 266},
    {"ParkEngKmh", 267}, {"NabPerKmh", 268}, {"ParkEngNab", 269},
    {"ParkBrake", 270}, {"BrakeIn", 271}, {"GpOutCurrent", 272},
    {"tqSrc", 2124}, {"tqUsed", 2125}, {"tqFrozen", 2126},
    {"adaStat", 2127}, {"adaShifts", 2128}, {"adA_c", 2129},
    {"adA_w", 2130}, {"adB_c", 2131}, {"adB_w", 2132},
    {"adC_c", 2133}, {"adC_w", 2134}, {"adD_c", 2135},
    {"adD_w", 2136}, {"adE_c", 2137}, {"adE_w", 2138},
    {"dgA", 2139}, {"dgB", 2140}, {"dgC", 2141},
    {"dgD", 2142}, {"dgE", 2143}, {"dgSys", 2144},
    {"dgCh7", 2145}, {"dgPos", 2146}, {"dgPzyl", 2147},
    {"diagPair", 2148}, {"spdInDiag", 2149}, {"spdOutDiag", 2150},
    {"canBus", 2151}, {"calSig", 2152}, {"tccSlip", 2153},
    {"tccSlipFlt", 2154}, {"parkInh", 2155}, {"brake", 2156},
    {"parkBrkW", 2157}, {"drvFlags1", 2158}, {"gpFault", 2159}, {"spdLoad", 2160}, {"brakePin", 2161},
    {"paramSrc", 2162}, {"bootStage", 2163},
};

static const Entry *Find(const char *name) {
  for (const Entry &e : entries)
    if (strcmp(e.name, name) == 0)
      return &e;
  return 0;
}

// Our parameter ids or retired ones, as the firmware's check sees them.
static bool OurParamId(uint16_t id) {
  for (const Entry &e : entries)
    if (!e.value && e.id == id)
      return true;
  return FlashGuard::Retired(id);
}

/* A ZombieVerter parameter page must not be taken as ours (it shares the
 * flash page and its CRC is valid), while our own and a blank page are.
 */
static void TestFlashGuard() {
  using namespace FlashGuard;
  StoredParam page[255];
  for (StoredParam &p : page)
    p = {KEY_BLANK, 0xFF, 0xFF, 0xFFFFFFFFu};
  CHECK_EQ(UnknownKeys(page, 255, OurParamId), 0); // blank page

  int k = 0;
  for (const Entry &e : entries)
    if (!e.value && k < 255)
      page[k++] = {(uint16_t)e.id, 0, 0, 0};
  CHECK_EQ(UnknownKeys(page, 255, OurParamId), 0); // a page we saved

  // Some ZombieVerter ids (Stm32-vcu param_prj.h): 5 Inverter, 6 Vehicle,
  // 39 interface, 116 BrkVacHyst (= our Can1Baud). The ZV page holds all
  // of its ids, so the ones we do not have show it is foreign.
  StoredParam zv[255];
  for (StoredParam &p : zv)
    p = {KEY_BLANK, 0xFF, 0xFF, 0xFFFFFFFFu};
  zv[0] = {5, 0, 0, 0};
  zv[1] = {6, 0, 0, 0};
  zv[2] = {39, 0, 0, 0};
  zv[3] = {116, 0, 0, 2500 << 5};
  CHECK_EQ(UnknownKeys(zv, 255, OurParamId), 3);
  CHECK(!OurParamId(5) && !OurParamId(6) && !OurParamId(39));
  CHECK(OurParamId(116)); // the shared id that would break Can1Baud

  // A page saved before the ATF curve was hard-coded: our current ids plus
  // AtfT1..6 (28-33) and AtfR1..6 (137-142). It must still load.
  const int oldAtf[] = {28, 29, 30, 31, 32, 33, 137, 138, 139, 140, 141, 142};
  for (int id : oldAtf)
    if (k < 255)
      page[k++] = {(uint16_t)id, 0, 0, 0};
  CHECK_EQ(UnknownKeys(page, 255, OurParamId), 0);
  CHECK(Retired(28) && Retired(33) && Retired(137) && Retired(142));
  CHECK(!Retired(20) && !Retired(143)); // AtfSource, AtfInput stay
  // The LIN settings that became fixed values, the same way.
  const int oldLin[] = {155, 156, 164, 165, 166, 168, 169, 170, 195,
                        196, 197, 198, 199, 200, 201, 202, 203};
  for (int id : oldLin) {
    CHECK(Retired((uint16_t)id));
    if (k < 255)
      page[k++] = {(uint16_t)id, 0, 0, 0};
  }
  CHECK_EQ(UnknownKeys(page, 255, OurParamId), 0);
  const int keptLin[] = {152, 153, 154, 167, 194};
  for (int id : keptLin)
    CHECK(!Retired((uint16_t)id) && OurParamId((uint16_t)id));
  CHECK(!Retired(5) && !Retired(6));

  // Every parameter id of the ZombieVerter (Stm32-vcu param_prj.h): even
  // with the retired ids allowed, the page is still seen as foreign.
  static const uint16_t zvIds[] = {
      5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 17, 18, 19, 20, 21, 22, 23, 24,
      25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 37, 38, 39, 40, 41, 42, 43, 44,
      45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 68,
      70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87,
      88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104,
      105, 106, 107, 108, 110, 111, 114, 115, 116, 117, 118, 120, 121, 122,
      123, 124, 126, 127, 128, 129, 131, 132, 133, 134, 135, 136, 137, 138,
      139, 140, 141, 142, 143, 144, 145, 146, 147, 149, 150, 151, 152, 155,
      156};
  int nz = 0;
  for (uint16_t id : zvIds)
    zv[nz++] = {id, 0, 0, 0};
  CHECK(UnknownKeys(zv, 255, OurParamId) >= 10);

  CHECK_EQ(InRange(2, 0, 4, 2), 2);
  CHECK_EQ(InRange(2500, 0, 4, 2), 2);
  CHECK_EQ(InRange(-1, 0, 4, 2), 2);
  CHECK_EQ(InRange(4, 0, 4, 2), 4);
}

void TestParams() {
  TestFlashGuard();
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
