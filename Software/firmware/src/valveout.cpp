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
/*
 * Valve outputs: maps the logical valves (A..E, SYS, ch7, PZYL ...) onto
 * TLE92466ED chip channels, sends the currents and collects the faults
 * and measured currents.
 */
#include "valveout.h"

static const uint8_t CH_PER_CHIP = 6; // TLE92466ED channels 0..5

// No chips attached, every valve unmapped.
ValveOut::ValveOut() : sup(0), ok(false), lost(false) {
  for (int i = 0; i < VALVE_MAX_CHIPS; i++) {
    chips[i] = 0;
    flags[i] = 0;
    misses[i] = 0;
    cfg[i] = TleConfig();
  }
  for (int v = 0; v < EDS_COUNT; v++) {
    map[v] = VALVE_NOT_FITTED;
    vflags[v] = 0;
  }
  for (int k = 0; k < EDS_COUNT + VALVE_AUX_MAX; k++) {
    lastSet[k] = -1;
    rFilt[k] = rShow[k] = 0;
  }
  for (int i = 0; i < VALVE_AUX_MAX; i++) {
    auxMap[i] = VALVE_NOT_FITTED;
    aux[i] = 0;
    aflags[i] = 0;
  }
}

// Attach a driver chip (0 = not fitted).
void ValveOut::AttachChip(uint8_t idx, Tle92466 *chip) {
  if (idx < VALVE_MAX_CHIPS)
    chips[idx] = chip;
}

// Valves whose fault means loss of pressure control: A..E and SYS.
uint16_t ValveOut::CriticalMask() {
  return (1u << EDS_A) | (1u << EDS_B) | (1u << EDS_C) | (1u << EDS_D) |
         (1u << EDS_E) | (1u << EDS_SYS);
}

// True if g is a real channel on an attached chip.
static bool ChOk(uint8_t g, Tle92466 *const *chips) {
  return g < VALVE_MAX_CHIPS * 8 && (g & 7) < CH_PER_CHIP && chips[g >> 3] != 0;
}

// Check the valve map before anything is driven.
bool ValveOut::Validate() const {
  uint16_t used = 0;
  for (int v = 0; v < EDS_COUNT; v++) {
    uint8_t g = map[v];
    if (g == VALVE_NOT_FITTED) {
      if (CriticalMask() & (1u << v))
        return false; // a pressure valve must be driven
      continue;
    }
    if (!ChOk(g, chips) || (used & (1u << g)))
      return false;
    used |= 1u << g;
  }
  for (int i = 0; i < VALVE_AUX_MAX; i++) {
    uint8_t g = auxMap[i];
    if (g == VALVE_NOT_FITTED)
      continue;
    if (!ChOk(g, chips) || (used & (1u << g)))
      return false; // an aux output must not sit on a valve channel
    used |= 1u << g;
  }
  return true;
}

// Bring the chips up with every channel at 0 mA.
bool ValveOut::Init(const TleConfig &base, bool openLoad) {
  ok = false;
  lost = false;
  sup = 0;
  if (!Validate())
    return false;
  // Channels that must never be switched off by the chip's own open-load /
  // short check: PZYL and the aux outputs (PHOLD), see parksafe.h.
  uint8_t parkMask[VALVE_MAX_CHIPS] = {0, 0};
  uint8_t usedMask[VALVE_MAX_CHIPS] = {0, 0};
  for (int v = 0; v < EDS_COUNT; v++) {
    uint8_t g = map[v];
    if (g == VALVE_NOT_FITTED)
      continue;
    usedMask[g >> 3] |= 1u << (g & 7);
    if (v == EDS_PZYL)
      parkMask[g >> 3] |= 1u << (g & 7);
  }
  for (int a = 0; a < VALVE_AUX_MAX; a++) {
    uint8_t g = auxMap[a];
    if (g == VALVE_NOT_FITTED)
      continue;
    usedMask[g >> 3] |= 1u << (g & 7);
    parkMask[g >> 3] |= 1u << (g & 7);
  }
  bool good = true;
  for (int c = 0; c < VALVE_MAX_CHIPS; c++) {
    cfg[c] = base;
    cfg[c].usedMask = usedMask[c];
    // The chip's own open-load / short check (which switches the channel
    // off) only on the proportional valves, never on the park parts.
    cfg[c].olMask = openLoad ? (uint8_t)(usedMask[c] & ~parkMask[c]) : 0;
    if (chips[c] && !chips[c]->Init(cfg[c]))
      good = false;
  }
  ok = good;
  return good;
}

// Send the present currents to the chips, every 1 ms. Only setpoints that
// changed go out on SPI.
void ValveOut::Update(const Eds &eds) {
  if (!ok)
    return;
  for (int v = 0; v < EDS_COUNT; v++) {
    uint8_t g = map[v];
    if (g != VALVE_NOT_FITTED) {
      int32_t i = eds.Output((EdsValve)v);
      chips[g >> 3]->SetCurrent(g & 7, i, DitherFor((EdsValve)v, i));
    }
  }
  for (int a = 0; a < VALVE_AUX_MAX; a++) {
    uint8_t g = auxMap[a];
    if (g != VALVE_NOT_FITTED)
      chips[g >> 3]->SetCurrent(g & 7, aux[a]);
  }
  for (int c = 0; c < VALVE_MAX_CHIPS; c++)
    if (chips[c])
      chips[c]->Flush();
}

// Every channel to 0 mA (the chips stay in Mission Mode, EN stays high).
void ValveOut::AllOff() {
  for (int c = 0; c < VALVE_MAX_CHIPS; c++) {
    if (!chips[c])
      continue;
    for (uint8_t ch = 0; ch < CH_PER_CHIP; ch++)
      chips[c]->SetCurrent(ch, 0);
    chips[c]->Flush();
  }
}

// Ian's 330e dither map at 25 degC for A..E and TCC / K0 (see valveout.h).
// Current and amplitude in 0.1 mA. PROVISIONAL.
static const int32_t ditI[] = {1200, 1500, 3100, 3900, 4300, 5500, 5740, 6200, 7800};
static const int32_t ditA[] = {0, 300, 700, 900, 1000, 1000, 900, 700, 0};
static const int DIT_N = sizeof(ditI) / sizeof(ditI[0]);

int32_t ValveOut::DitherFor(EdsValve v, int32_t i) {
  if (v == EDS_SYS)
    return i >= 1000 && i <= 8000 ? 300 : 0; // LINE: flat 0.03 A
  if (v > EDS_K0)
    return 0; // POS / PREL and PZYL: no dither
  if (i <= ditI[0] || i >= ditI[DIT_N - 1])
    return 0;
  for (int k = 1; k < DIT_N; k++)
    if (i <= ditI[k])
      return ditA[k - 1] + (ditA[k] - ditA[k - 1]) * (i - ditI[k - 1]) / (ditI[k] - ditI[k - 1]);
  return 0;
}

int32_t ValveOut::CurrentErrLimit(int32_t set) { return set >= 4000 ? 1200 : 1700; }

// Measured-vs-setpoint check for one output, and the coil resistance
// estimate. Both only when the setpoint has not changed since the last
// poll (100 ms), so slews and steps do not count. underOk: a current
// below the setpoint is fine for the error flag (PZYL may run at full
// duty without reaching its pull-in current on a low supply).
uint8_t ValveOut::CurrentCheck(int idx, uint8_t g, bool underOk) {
  int32_t set = chips[g >> 3]->Wanted(g & 7);
  int32_t meas = chips[g >> 3]->Current(g & 7);
  bool steady = set == lastSet[idx];
  lastSet[idx] = set;
  int32_t dev = meas - set;
  if (dev < 0)
    dev = -dev;
  int32_t err = (underOk && meas < set) ? 0 : dev;

  // Resistance estimate (read-only, see valveout.h): steady, on, enough
  // current, and the current where it should be.
  int32_t raw = chips[g >> 3]->CoilMilliOhm(g & 7);
  bool rValid = steady && set >= RCOIL_MIN_I && meas >= RCOIL_MIN_I &&
                dev <= CurrentErrLimit(set) && raw > 0;
  if (rValid) {
    rFilt[idx] = rFilt[idx] == 0 ? raw : rFilt[idx] + (raw - rFilt[idx]) / 8;
    rShow[idx] = rFilt[idx];
  } else {
    rShow[idx] = 0;
  }
  return steady && err > CurrentErrLimit(set) ? VALVE_F_IERR : 0;
}

int32_t ValveOut::ChCurrent(uint8_t g) const {
  if (g == VALVE_NOT_FITTED || !ChOk(g, chips))
    return 0;
  return chips[g >> 3]->Current(g & 7);
}

int32_t ValveOut::Measured(EdsValve v) const { return ChCurrent(map[v]); }

int32_t ValveOut::AuxMeasured(uint8_t i) const {
  return i < VALVE_AUX_MAX ? ChCurrent(auxMap[i]) : 0;
}

int32_t ValveOut::ChipTempC() const {
  int32_t t = -100;
  for (int c = 0; c < VALVE_MAX_CHIPS; c++)
    if (chips[c] && chips[c]->TempC() > t)
      t = chips[c]->TempC();
  return t;
}

uint32_t ValveOut::CommErrors() const {
  uint32_t n = 0;
  for (int c = 0; c < VALVE_MAX_CHIPS; c++)
    if (chips[c])
      n += chips[c]->CommErrors();
  return n;
}

// Read the chip faults and map them to valves (bit per valve).
uint16_t ValveOut::PollFaults() {
  uint16_t valves = 0;
  sup = 0;
  for (int v = 0; v < EDS_COUNT; v++)
    vflags[v] = 0;
  for (int a = 0; a < VALVE_AUX_MAX; a++)
    aflags[a] = 0;
  for (int c = 0; c < VALVE_MAX_CHIPS; c++) {
    if (!chips[c])
      continue;
    TleDiag d;
    bool answered = chips[c]->ReadDiag(&d);
    if (!answered) {
      sup |= TLE_SUP_COMM;
      if (++misses[c] < VALVE_CHIP_MISSES)
        continue; // try again next poll before calling the chip lost
      misses[c] = VALVE_CHIP_MISSES;
    } else {
      misses[c] = 0;
    }
    flags[c] = (uint16_t)d.fbStat;
    if (d.diag0 & 0x03) sup |= TLE_SUP_VBAT;
    if (d.diag0 & 0x0C) sup |= TLE_SUP_VIO;
    if (d.diag0 & 0x30) sup |= TLE_SUP_VDD;
    if (d.fbStat & (TLE_FS_COTERR | TLE_FS_COTWARN)) sup |= TLE_SUP_TEMP;
    if (d.fbStat & TLE_FS_CLK_NOK) sup |= TLE_SUP_CLOCK;
    if (d.fbStat & (TLE_FS_POR_EVENT | TLE_FS_RES_EVENT)) sup |= TLE_SUP_RESET;
    if (!d.commOk || d.critical) sup |= TLE_SUP_COMM;
    bool wholeChip = !answered || !d.mission || (d.fbStat & TLE_FS_COTERR);
    if (!d.mission && ok) {
      sup |= c == 0 ? TLE_SUP_NOT_MISSION0 : TLE_SUP_NOT_MISSION1;
      // The chip fell back to Config Mode by itself (supply or clock
      // fault, or a reset), which switched all its channels off. Put it
      // straight back: this only ever turns outputs on again, it never
      // resets the chip or touches EN. Setpoints follow on the next Update().
      if (answered) {
        sup |= TLE_SUP_RECOVER;
        if (!chips[c]->Init(cfg[c]))
          lost = true;
      } else {
        lost = true;
      }
    }
    chips[c]->ReadCurrents();
    for (int v = 0; v < EDS_COUNT; v++) {
      uint8_t g = map[v];
      if (g == VALVE_NOT_FITTED || (g >> 3) != c)
        continue;
      uint8_t bit = 1u << (g & 7);
      vflags[v] = (d.oc & bit ? VALVE_F_OCP : 0) | ((d.ol | d.olsg) & bit ? VALVE_F_OLF : 0) |
                  (d.sg & bit ? VALVE_F_SG : 0) |
                  ((wholeChip || (d.ote & bit)) ? VALVE_F_CHIP : 0);
      if (vflags[v])
        valves |= 1u << v;
      // Report only (not in the returned mask), see VALVE_F_IERR.
      if (!wholeChip)
        vflags[v] |= CurrentCheck(v, g, v == EDS_PZYL);
      else
        rShow[v] = 0;
    }
    for (int a = 0; a < VALVE_AUX_MAX; a++) {
      uint8_t g = auxMap[a];
      if (g == VALVE_NOT_FITTED || (g >> 3) != c)
        continue;
      uint8_t bit = 1u << (g & 7);
      aflags[a] = (d.oc & bit ? VALVE_F_OCP : 0) | ((d.ol | d.olsg) & bit ? VALVE_F_OLF : 0) |
                  (d.sg & bit ? VALVE_F_SG : 0) |
                  ((wholeChip || (d.ote & bit)) ? VALVE_F_CHIP : 0);
      if (!wholeChip)
        aflags[a] |= CurrentCheck(EDS_COUNT + a, g, false);
      else
        rShow[EDS_COUNT + a] = 0;
    }
  }
  return valves;
}
