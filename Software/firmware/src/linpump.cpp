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

int FindData(const uint8_t *buf, int n, uint8_t pid) {
  for (int i = 0; i + 1 < n; i++) {
    if (buf[i] == 0x55 && buf[i + 1] == pid)
      return i + 2;
  }
  return -1;
}

} // namespace Lin

bool LinPumpDemand(uint8_t req, bool enabled, bool pressureNeeded,
                   int32_t n_t, int32_t onRpm, int32_t hyst, bool wasOn) {
  if (!enabled || req == LINREQ_OFF)
    return false;
  if (req == LINREQ_FORCE)
    return true;
  if (!pressureNeeded)
    return false;
  if (n_t < 0)
    n_t = -n_t;
  return wasOn ? n_t < onRpm + hyst : n_t < onRpm;
}

LinPump::LinPump()
    : cfg(), state(LP_OFF), st(), tState(0), tCycle(0), tNoGood(0), alive(0),
      auxToggle(0), commFaults(0), csErrors(0), timeouts(0), txErrors(0) {
  cfg.enhanced = true;
  cfg.cycleMs = 25;
  cfg.rxDelayMs = 12;
  cfg.coldMs = 900;
  cfg.phase2Ms = 25;
  cfg.timeoutMs = 200;
  cfg.retryMs = 500;
  cfg.pollAux = false;
}

void LinPump::Enter(LinPumpState s) {
  state = s;
  tState = 0;
  if (s == LP_COLD1 || s == LP_OFF) {
    tCycle = 0;
    tNoGood = 0;
    st.fresh = false;
  }
}

void LinPump::Command(uint8_t d[8]) const {
  for (int i = 0; i < 8; i++)
    d[i] = 0;
  bool run = state == LP_START || state == LP_RUN;
  d[0] = run ? 0x55 : 0xAA;
  d[2] = state == LP_COLD1 ? 0x0A : 0x42;
  // Byte 6: 0x73 until the pump has answered with status 00, then the
  // live echo of byte 6 of the last good 0x32.
  d[6] = state == LP_RUN ? st.echo : 0x73;
  d[7] = alive & 0x0F;
}

bool LinPump::Tick1ms(bool request, LinSlot *slot) {
  if (!request) {
    if (state != LP_OFF)
      Enter(LP_OFF);
    return false;
  }
  switch (state) {
  case LP_OFF:
    Enter(LP_COLD1);
    break;
  case LP_COLD1:
    if (tState >= cfg.coldMs)
      Enter(LP_COLD2);
    break;
  case LP_COLD2:
    if (tState >= cfg.phase2Ms)
      Enter(LP_START);
    break;
  case LP_FAULT:
    tState++;
    if (tState >= cfg.retryMs)
      Enter(LP_COLD1);
    return false; // bus silent while pausing
  default:
    break;
  }

  // Responses are only expected once the run sequence has started.
  if (state == LP_START || state == LP_RUN) {
    if (tNoGood >= cfg.timeoutMs) {
      timeouts++;
      Enter(LP_FAULT);
      return false;
    }
    tNoGood++;
  }

  bool due = false;
  uint32_t c = tCycle;
  if (c == 0) {
    slot->id = LINPUMP_ID_CMD;
    slot->publish = true;
    Command(slot->data);
    alive = (uint8_t)((alive + 1) & 0x0F);
    due = true;
  } else if (c == cfg.rxDelayMs && state != LP_COLD1 && state != LP_COLD2) {
    slot->id = LINPUMP_ID_STATUS;
    slot->publish = false;
    due = true;
  } else if (cfg.pollAux && c == (uint32_t)cfg.rxDelayMs + 8u &&
             (state == LP_RUN)) {
    slot->id = auxToggle ? LINPUMP_ID_AUX2 : LINPUMP_ID_AUX1;
    slot->publish = false;
    auxToggle ^= 1;
    due = true;
  }
  tState++;
  tCycle++;
  if (tCycle >= cfg.cycleMs)
    tCycle = 0;
  return due;
}

bool LinPump::OnResponse(uint8_t id, const uint8_t *b, int n) {
  if (n < 9)
    return false;
  if (Lin::Checksum(cfg.enhanced, Lin::Pid(id), b, 8) != b[8]) {
    csErrors++;
    return false;
  }
  uint8_t *dst = id == LINPUMP_ID_STATUS ? st.raw32
                 : id == LINPUMP_ID_AUX1 ? st.raw33
                 : id == LINPUMP_ID_AUX2 ? st.raw34
                                         : 0;
  if (!dst)
    return false;
  for (int i = 0; i < 8; i++)
    dst[i] = b[i];
  if (id != LINPUMP_ID_STATUS)
    return true;
  st.status = b[0];
  st.rpm = (int32_t)b[3] * 32;
  st.supplymV = (int32_t)b[4] * 100;
  st.echo = b[6];
  if (b[0] == LINPUMP_ST_COMMFAULT)
    commFaults++;
  if (b[0] == LINPUMP_ST_OK) {
    st.fresh = true;
    tNoGood = 0;
    if (state == LP_START)
      Enter(LP_RUN);
  }
  return true;
}
