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
#include "linpump.h"

namespace Lin {

uint8_t Pid(uint8_t id) {
  id &= 0x3F;
  uint8_t b = id;
  uint8_t p0 = ((b >> 0) ^ (b >> 1) ^ (b >> 2) ^ (b >> 4)) & 1;
  uint8_t p1 = (~((b >> 1) ^ (b >> 3) ^ (b >> 4) ^ (b >> 5))) & 1;
  return (uint8_t)(id | (p0 << 6) | (p1 << 7));
}

static uint8_t Sum(uint16_t s, const uint8_t *d, int n) {
  for (int i = 0; i < n; i++) {
    s += d[i];
    if (s > 0xFF)
      s -= 0xFF; // add the carry back in
  }
  return (uint8_t)~s;
}

uint8_t ChecksumClassic(const uint8_t *d, int n) { return Sum(0, d, n); }

uint8_t ChecksumEnhanced(uint8_t pid, const uint8_t *d, int n) {
  return Sum(pid, d, n);
}

uint8_t Checksum(bool enhanced, uint8_t pid, const uint8_t *d, int n) {
  return enhanced ? ChecksumEnhanced(pid, d, n) : ChecksumClassic(d, n);
}

uint8_t Crc8(const uint8_t *d, int n) {
  uint8_t crc = 0x00;
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
  }
  return crc;
}

int FindData(const uint8_t *buf, int n, uint8_t pid) {
  for (int i = 0; i + 1 < n; i++) {
    if (buf[i] == 0x55 && buf[i + 1] == pid)
      return i + 2;
  }
  return -1;
}

} // namespace Lin

// Schedule table [C]: 0x30 in the even slots, the three status frames in
// the odd ones.
static const uint8_t schedule[6] = {LINPUMP_ID_CMD, LINPUMP_ID_STATUS,
                                    LINPUMP_ID_CMD, LINPUMP_ID_AUX1,
                                    LINPUMP_ID_CMD, LINPUMP_ID_AUX2};

LinPump::LinPump()
    : cfg(), state(LP_OFF), st(), tSlot(0), slotIdx(0), txAlive(0),
      oilRaw(0), extraFlags(0), clearPending(false), tDiagBlock(0),
      tNoValid(0), tStartBlank(0), tRun(0), tDeliv(0), tLow(0), ownBits(0),
      badCrc(0), linChkErrors(0), crcErrors(0),
      aliveErrors(0), txErrors(0) {
  cfg.enhanced = true;
  cfg.slotMs = 10;
  cfg.busFaultMs = 1800;
  cfg.diagDelayMs = 400;
  cfg.startBlankMs = 1000;
  cfg.armMs = 1000;
  cfg.symMaxMs = 1500;
  cfg.delivRpm = 100;
  cfg.processRx = true;
  st.supplymV = 14000;
  for (int i = 0; i < 3; i++) {
    lastAlive[i] = -1;
    stuck[i] = 0;
  }
}

void LinPump::ClearErrors() {
  clearPending = true;
  ownBits = 0;
  badCrc = 0;
  tNoValid = 0;
  for (int i = 0; i < 3; i++) {
    lastAlive[i] = -1;
    stuck[i] = 0;
  }
}

void LinPump::Command(uint8_t d[8]) const {
  d[0] = state == LP_RUN ? LINPUMP_OP_RUN : LINPUMP_OP_OFF;
  d[1] = (uint8_t)(extraFlags | (clearPending ? LINPUMP_FLAG_CLEAR : 0));
  d[2] = oilRaw;
  d[3] = 0; // after-run time: 0 as in the 330e calibration
  d[4] = 0;
  d[5] = 0;
  d[6] = Lin::Crc8(d, 1);
  d[7] = txAlive & 0x0F;
}

bool LinPump::Tick1ms(bool active, bool run, int32_t oilTempC, LinSlot *slot) {
  if (!active) {
    if (state != LP_OFF) {
      state = LP_OFF;
      tSlot = 0;
      slotIdx = 0;
      st.seen32 = false; // old replies do not count after a restart
    }
    tRun = tDeliv = tLow = 0;
    return false;
  }
  if (state == LP_OFF) {
    // Bus comes up: start the schedule and clear the pump errors once,
    // as the OEM does at ignition on. The alive counter is not reset: it
    // starts at 0 after power-on only (Ian, Q11 [C]).
    ClearErrors();
    tSlot = 0;
    slotIdx = 0;
    tStartBlank = cfg.startBlankMs;
  }
  // AA -> 55 directly, no settle phase (Ian, Q5 [C]).
  state = run ? LP_RUN : LP_IDLE;
  if (tStartBlank)
    tStartBlank--;

  // "Pump not delivering": the check is armed armMs after the run
  // command; the symptom is set when the pump has then reported less than
  // delivRpm for (symMaxMs - armMs) without a break.
  if (run) {
    if (tRun < 0xFFFFFFu)
      tRun++;
    bool turning = st.seen32 && st.rpm >= cfg.delivRpm && tNoValid < cfg.busFaultMs;
    tDeliv = turning ? (tDeliv < 0xFFFFFFu ? tDeliv + 1 : tDeliv) : 0;
    if (tRun >= cfg.armMs) {
      if (turning) {
        tLow = 0;
        ownBits &= ~LINPUMP_ST_NODELIV;
      } else if (++tLow >= (uint32_t)(cfg.symMaxMs > cfg.armMs ? cfg.symMaxMs - cfg.armMs : 1)) {
        ownBits |= LINPUMP_ST_NODELIV;
      }
    }
  } else {
    tRun = tDeliv = tLow = 0;
  }
  int32_t t = oilTempC + 50;
  oilRaw = (uint8_t)(t < 0 ? 0 : (t > 255 ? 255 : t));

  if (tDiagBlock)
    tDiagBlock--;
  if (tNoValid < 0xFFFFFFu)
    tNoValid++;
  if (tNoValid >= cfg.busFaultMs)
    ownBits |= LINPUMP_ST_NORX;

  bool due = false;
  if (tSlot == 0) {
    slot->id = schedule[slotIdx];
    slot->publish = slot->id == LINPUMP_ID_CMD;
    if (slot->publish) {
      Command(slot->data);
      txAlive = (uint8_t)((txAlive + 1) & 0x0F);
      if (clearPending) {
        clearPending = false; // on the bus for exactly one 0x30
        tDiagBlock = cfg.diagDelayMs;
      }
    }
    due = true;
  }
  if (++tSlot >= cfg.slotMs) {
    tSlot = 0;
    slotIdx = (uint8_t)((slotIdx + 1) % 6);
  }
  return due;
}

bool LinPump::CheckAlive(int idx, uint8_t alive) {
  alive &= 0x0F;
  int8_t last = lastAlive[idx];
  lastAlive[idx] = (int8_t)alive;
  if (last < 0)
    return true;
  uint8_t d = (uint8_t)((alive - last) & 0x0F);
  if (d == 0) {
    // One repeat is tolerated; the second in a row is a fault [C].
    if (++stuck[idx] >= 2) {
      aliveErrors++;
      ownBits |= LINPUMP_ST_ALIVE;
      return false;
    }
    return true;
  }
  stuck[idx] = 0;
  if (d > 2) {
    aliveErrors++;
    ownBits |= LINPUMP_ST_ALIVE;
    return false;
  }
  return true;
}

bool LinPump::OnResponse(uint8_t id, const uint8_t *b, int n) {
  if (n < 9 || id < LINPUMP_ID_STATUS || id > LINPUMP_ID_AUX2)
    return false;
  if (Lin::Checksum(cfg.enhanced, Lin::Pid(id), b, 8) != b[8]) {
    linChkErrors++;
    ownBits |= LINPUMP_ST_LINCHK;
    return false;
  }
  int idx = id - LINPUMP_ID_STATUS;
  if (id == LINPUMP_ID_STATUS) {
    if (Lin::Crc8(&b[2], 1) != b[6]) {
      crcErrors++;
      if (++badCrc >= 2)
        ownBits |= LINPUMP_ST_CRC8;
      return false;
    }
    badCrc = 0;
  }
  if (!CheckAlive(idx, b[7]))
    return false;

  uint8_t *dst = idx == 0 ? st.raw32 : (idx == 1 ? st.raw33 : st.raw34);
  for (int i = 0; i < 8; i++)
    dst[i] = b[i];
  st.eb0 = b[0];
  st.eb1 = b[1];
  if (id == LINPUMP_ID_STATUS) {
    st.opPoint = b[2];
    st.rpm = (int32_t)b[3] * 32;
    st.currentmA = (int32_t)b[4] * 200;
    st.supplymV = (int32_t)b[5] * 100;
    st.seen32 = true;
    tNoValid = 0;
  } else if (id == LINPUMP_ID_AUX1) {
    st.tempRaw = b[2];
    st.pumpId = ((uint32_t)b[3] << 24) | ((uint32_t)b[4] << 16) |
                ((uint32_t)b[5] << 8) | b[6];
  } else {
    st.hwVersion = b[2];
    st.swVersion = (uint16_t)((b[3] << 8) | b[4]);
    st.seen34 = true;
    // The 330e expects SW variant bits 001 = IEP85 (D_SWVersion_LINIEP
    // 0x2000, mask 0xE000) [C]; HW check off.
    if ((st.swVersion & 0xE000) != 0x2000)
      ownBits |= LINPUMP_ST_SWVER;
    else
      ownBits &= ~LINPUMP_ST_SWVER;
  }
  return true;
}

uint32_t LinPump::StatusWord() const {
  return ((uint32_t)st.eb0 << 16) | ((uint32_t)st.eb1 << 24) | ownBits;
}

uint8_t LinPump::Classes() const {
  if (state == LP_OFF || tDiagBlock || tStartBlank || !cfg.processRx)
    return 0;
  uint32_t w = StatusWord();
  uint8_t c = 0;
  if (w & LINPUMP_MASK_TEMP)
    c |= LPC_TEMPORARY;
  if (w & LINPUMP_MASK_PERM)
    c |= LPC_PERMANENT;
  if (w & LINPUMP_MASK_COMM)
    c |= LPC_COMM;
  if (w & LINPUMP_ST_NORX)
    c |= LPC_BUS;
  if (w & LINPUMP_ST_NODELIV)
    c |= LPC_DELIVERY;
  return c;
}

bool LinPumpRule::Update(uint8_t req, int32_t n_t, int32_t sumpC,
                         int32_t pSys) {
  if (req == LINREQ_OFF) {
    on = speedOn = false;
    tOff = 0;
    return false;
  }
  if (req == LINREQ_FORCE) {
    on = true;
    return true;
  }
  if (n_t < 0)
    n_t = -n_t;
  speedOn = speedOn ? n_t < cfg.onRpm + cfg.hystRpm : n_t < cfg.onRpm;
  bool tempOk = sumpC >= cfg.tempMin && sumpC <= cfg.tempMax;
  if (sumpC >= cfg.tempMin) {
    tCold = 0; // warm again: the cold allowance is back
  } else if (sumpC >= cfg.coldMin && tCold < cfg.coldMaxMs && speedOn &&
             pSys < cfg.pMaxMbar) {
    tempOk = true; // cold run, counted below
    tCold += 10;
  }
  bool want = speedOn && tempOk && pSys < cfg.pMaxMbar;
  if (want) {
    on = true;
    tOff = 0;
  } else if (on) {
    tOff += 10;
    if (tOff >= cfg.offDelayMs) {
      on = false;
      tOff = 0;
    }
  }
  return on;
}
