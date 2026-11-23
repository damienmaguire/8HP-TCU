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
#include "gwslever.h"
#include "linpump.h" // Lin::Crc8: poly 0x1D, init 0
#include "tcu.h"     // Selector

GwsLever::GwsLever()
    : cfg{90, 2}, seen(false), msSinceRx(0xFFFF), lastAlive(-1), stuck(0),
      rawMove(0), pendMove(0), pendCount(0), takenMove(0), pRaw(false),
      pStable(false), pEdge(false), pCount(0), state(GWS_P), tipUp(false), tipDown(false),
      flashMs(0), crcErr(0), cntErr(0), dlcErr(0) {}

uint8_t GwsLever::RxCrc(const uint8_t *d) {
  return (uint8_t)(Lin::Crc8(d + 1, 3) ^ GWS_RX_XOR);
}

uint8_t GwsLever::TxCrc(const uint8_t *d) {
  return (uint8_t)(Lin::Crc8(d + 1, 4) ^ GWS_TX_XOR);
}

bool GwsLever::OnFrame(const uint8_t *d, uint8_t dlc) {
  if (dlc != 4) {
    dlcErr++;
    return false;
  }
  if (RxCrc(d) != d[0]) {
    crcErr++;
    return false;
  }
  uint8_t alive = d[1] & 0x0F;
  if (alive > 14) {
    cntErr++;
    return false;
  }
  if (lastAlive >= 0) {
    uint8_t diff = (uint8_t)((alive + 15 - lastAlive) % 15);
    lastAlive = (int8_t)alive;
    if (diff == 0) {
      if (++stuck >= 2) { // one repeat tolerated, the second is not
        cntErr++;
        return false;
      }
    } else {
      stuck = 0;
      if (diff > 2) {
        cntErr++;
        return false;
      }
    }
  } else {
    lastAlive = (int8_t)alive;
  }
  rawMove = d[2] >> 4; // mask the S button / drive logic low nibble
  uint8_t p0 = d[3] & 3, p1 = (d[3] >> 2) & 3, p2 = (d[3] >> 4) & 3;
  pRaw = p0 == 1 && p1 == 1 && p2 == 1;
  // Debounce on frames, not time.
  if (rawMove == pendMove) {
    if (pendCount < 255)
      pendCount++;
  } else {
    pendMove = rawMove;
    pendCount = 1;
  }
  pCount = pRaw == pStable ? 0 : (uint8_t)(pCount + 1);
  if (pCount >= cfg.debounce) {
    pStable = pRaw;
    pCount = 0;
    pEdge = pStable;
  }
  seen = true;
  msSinceRx = 0;
  return true;
}

void GwsLever::Take(uint8_t move, bool engageOk) {
  uint8_t want = state;
  bool gated = false;
  switch (move) {
  case GWS_UP1:
    if (state == GWS_P || state == GWS_D || state == GWS_M)
      want = GWS_N;
    break;
  case GWS_UP2:
    if (state != GWS_R) {
      want = GWS_R;
      gated = true;
    }
    break;
  case GWS_DOWN1:
    if (state == GWS_P || state == GWS_R)
      want = GWS_N;
    break;
  case GWS_DOWN2:
    if (state == GWS_P || state == GWS_N || state == GWS_R) {
      want = GWS_D;
      gated = true;
    }
    break;
  case GWS_GATE:
    if (state == GWS_D)
      want = GWS_M;
    break;
  case GWS_MPLUS:
    if (state == GWS_M)
      tipUp = true;
    else if (state == GWS_D)
      want = GWS_M;
    break;
  case GWS_MMINUS:
    if (state == GWS_M)
      tipDown = true;
    else if (state == GWS_D)
      want = GWS_M;
    break;
  default:
    break;
  }
  if (want == state)
    return;
  if (gated && !engageOk) {
    flashMs = 1000; // blocked request [I]
    return;
  }
  state = want;
}

void GwsLever::Tick10ms(bool engageOk) {
  tipUp = tipDown = false;
  if (flashMs >= 10)
    flashMs -= 10;
  else
    flashMs = 0;
  if (msSinceRx < 0xFFFF - 10)
    msSinceRx += 10;
  if (!Valid()) {
    pEdge = false;
    return;
  }
  if (pEdge) {
    pEdge = false;
    if (state != GWS_P) {
      if (engageOk)
        state = GWS_P;
      else
        flashMs = 1000;
    }
  }
  if (pendCount >= cfg.debounce && pendMove != takenMove) {
    takenMove = pendMove;
    if (pendMove == GWS_CENTRE) {
      if (state == GWS_M)
        state = GWS_D; // out of the side gate
    } else {
      Take(pendMove, engageOk);
    }
  }
}

uint8_t GwsLever::Selector() const {
  switch (state) {
  case GWS_P: return SEL_P;
  case GWS_R: return SEL_R;
  case GWS_N: return SEL_N;
  default: return SEL_D;
  }
}

void GwsDisplay::Pack(uint8_t pos, uint8_t counter, uint8_t b3, uint8_t b4,
                      uint8_t d[5]) {
  d[1] = counter % 15;
  d[2] = pos;
  d[3] = b3;
  d[4] = b4;
  d[0] = GwsLever::TxCrc(d);
}

void GwsDisplay::Tick100ms(bool t15, uint8_t st, bool flash, uint8_t mSub,
                           uint8_t b34, uint8_t gear, uint8_t d[5]) {
  static const uint8_t posCode[5] = {0x20, 0x40, 0x60, 0x80, 0x80};
  if (t15) {
    t15Seen = true;
    offMs = 0;
  } else if (t15Seen && offMs < 0xFFFFFF) {
    offMs += 100;
  }
  bool off = !t15Seen || (!t15 && offMs >= 30000);
  uint8_t pos = 0;
  if (!off && st <= GWS_M) {
    pos = posCode[st];
    if (st == GWS_M)
      pos |= mSub & 0x07;
    if (flash)
      pos |= 0x08;
  }
  uint8_t b3 = 0xFC, b4 = 0xFF;
  if (b34 == GWS_B34_ZV) {
    b3 = 0x00;
    b4 = gear;
  }
  Pack(pos, counter, b3, b4, d);
  counter = (uint8_t)((counter + 1) % 15);
}
