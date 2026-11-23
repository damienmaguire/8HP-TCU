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
#include "canlever.h"
#include "tcu.h" // SEL_*

// ---- JLR tables (ZV src/shifters/JLR_G2.cpp, as there) ----
static const uint8_t g2Byte4[16] = {0,    0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70,
                                    0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0};
static const uint8_t g2Byte5P[16] = {0x21, 0xBC, 0x06, 0x9B, 0x6F, 0xF2, 0x48, 0xD5,
                                     0xBD, 0x20, 0x9A, 0x07, 0xF3, 0x6E, 0xD4, 0x49};
static const uint8_t g2Byte5R[16] = {0x67, 0xFA, 0x40, 0xDD, 0x29, 0xB4, 0x0E, 0x93,
                                     0xFB, 0x66, 0xDC, 0x41, 0xB5, 0x28, 0x92, 0x0F};
static const uint8_t g2Byte5N[16] = {0xFB, 0x76, 0xCC, 0x51, 0xA5, 0x38, 0x82, 0x1F,
                                     0x77, 0xEA, 0x52, 0xCD, 0x39, 0xA4, 0x1E, 0x83};
static const uint8_t g2Byte5D[16] = {0xE8, 0x75, 0xCF, 0x52, 0xA6, 0x3B, 0x81, 0x1C,
                                     0x74, 0xE9, 0x53, 0xCE, 0x3A, 0xA7, 0x1D, 0x80};
static const uint8_t g2Byte5S[16] = {0xE2, 0x7F, 0xC5, 0x58, 0xAC, 0x31, 0x8B, 0x16,
                                     0x7E, 0xE3, 0x59, 0xC4, 0x30, 0xAD, 0x17, 0x8A};

// JLR knob position codes (ZV JLR_Park .. JLR_Sport) -> GwsState.
static int JlrToState(uint8_t code) {
  switch (code) {
  case 0: return GWS_P;
  case 1: return GWS_R;
  case 2: return GWS_N;
  case 3: return GWS_D;
  case 7: return GWS_M; // "Sport"
  default: return -1;
  }
}

// ---- PKP-2300 keys (ZV src/shifters/PKP2300_Lever.cpp) ----
#define PKP_TPDO1 0x195
#define PKP_RPDO1 0x215
#define PKP_NODE_ID 0x15
#define PKP_KEY_D (1 << 0)
#define PKP_KEY_N (1 << 1)
#define PKP_KEY_R (1 << 2)
#define PKP_KEY_P (1 << 3)

// PKP2300 LED bits for a lever state.
static uint8_t PkpMask(uint8_t st) {
  switch (st) {
  case GWS_P: return PKP_KEY_P;
  case GWS_R: return PKP_KEY_R;
  case GWS_N: return PKP_KEY_N;
  default: return PKP_KEY_D;
  }
}

// RX ids and default timeouts per type (ms). GWS 90 ms is the EGS record
// [C]; the others are not documented: 500 ms [I], PKP 300 ms as ZV.
static const uint16_t rxId[LEVER_COUNT] = {GWS_RX_ID, 0, 0x192, 0x312, 0x0E0, PKP_TPDO1, 0};
static const uint16_t defTimeout[LEVER_COUNT] = {90, 0, 500, 500, 500, 300, 0};

// Start with no lever seen and the selector in P.
CanLever::CanLever()
    : cfg{LEVER_BMW_GWS, 0, 2, 1, GWS_B34_OEM}, seen(false), msSinceRx(0xFFFF),
      state(GWS_P), rawCode(0), lastCode(0xFF), pendCode(0xFF), pendCount(0),
      flashMs(0), flashOnce(false),
      rejected(0), prevKeys(0), cntErr(0), txDiv(0), txCnt(0), offCount(0),
      knobLock(0) {}

// Pick the lever type, bus and timing settings.
void CanLever::Configure(const LeverConfig &c) {
  cfg = c;
  if (cfg.type >= LEVER_COUNT)
    cfg.type = LEVER_BMW_GWS;
  GwsConfig g = {TimeoutMs(), cfg.debounce};
  gws.Configure(g);
}

// Lever timeout in use (the type's default when LeverTimeout = 0).
uint16_t CanLever::TimeoutMs() const {
  return cfg.timeoutMs ? cfg.timeoutMs : defTimeout[cfg.type];
}

// CAN ids this lever type sends.
int CanLever::RxIds(uint32_t ids[2]) const {
  if (!rxId[cfg.type])
    return 0;
  ids[0] = rxId[cfg.type];
  return 1;
}

// Lever frames seen and still arriving (no timeout).
bool CanLever::Valid() const {
  if (cfg.type == LEVER_BMW_GWS)
    return gws.Valid();
  return seen && msSinceRx <= TimeoutMs();
}

// Common lever state P / R / N / D / M.
uint8_t CanLever::State() const { return cfg.type == LEVER_BMW_GWS ? gws.State() : state; }

// Lever state as a TCU selector (M counts as D).
uint8_t CanLever::Selector() const {
  switch (State()) {
  case GWS_P: return SEL_P;
  case GWS_R: return SEL_R;
  case GWS_N: return SEL_N;
  default: return SEL_D;
  }
}

// One-shot tip up (GWS M+).
bool CanLever::TipUp() const { return cfg.type == LEVER_BMW_GWS && gws.TipUp(); }
// One-shot tip down (GWS M-).
bool CanLever::TipDown() const { return cfg.type == LEVER_BMW_GWS && gws.TipDown(); }
// Display should flash (a request was refused).
bool CanLever::Flash() const { return cfg.type == LEVER_BMW_GWS ? gws.Flash() : flashMs > 0; }
// Raw lever code of the last frame, for diagnosis.
uint8_t CanLever::RawMove() const { return cfg.type == LEVER_BMW_GWS ? gws.RawMove() : rawCode; }
// P button pressed (where the lever has one).
bool CanLever::PButton() const {
  if (cfg.type == LEVER_BMW_GWS)
    return gws.PButton();
  if (cfg.type == LEVER_PKP2300)
    return (prevKeys & PKP_KEY_P) != 0;
  return false;
}
// Frames dropped for a bad CRC.
uint16_t CanLever::CrcErrors() const { return cfg.type == LEVER_BMW_GWS ? gws.CrcErrors() : 0; }
// Frames dropped for a bad or repeated counter.
uint16_t CanLever::CounterErrors() const {
  return cfg.type == LEVER_BMW_GWS ? gws.CounterErrors() : cntErr;
}

/// Take a requested position through the engage gate. N is always allowed.
void CanLever::Request(uint8_t want, bool engageOk) {
  if (want == state)
    return;
  if (want != GWS_N && !engageOk) {
    flashMs = 1000;
    flashOnce = true;
    rejected = want;
    return;
  }
  state = want;
}

// Decode one received lever frame; false if it is not this lever's.
bool CanLever::OnFrame(uint32_t id, const uint8_t *d, uint8_t dlc) {
  if (cfg.type == LEVER_BMW_GWS)
    return id == GWS_RX_ID && gws.OnFrame(d, dlc);
  if (!rxId[cfg.type] || id != rxId[cfg.type])
    return false;
  switch (cfg.type) {
  case LEVER_BMW_E65: {
    if (dlc < 3)
      return false;
    // ZV: data[0] & 0x00FFFFFF, little-endian bytes 0..2.
    uint32_t v = d[0] | (d[1] << 8) | ((uint32_t)d[2] << 16);
    uint8_t code;
    switch (v) {
    case 0x80506A: code = 1; break; // P button
    case 0x80042D: code = 2; break; // R+ (pushed fully forward)
    case 0x800374: code = 3; break; // D+ (pulled fully back)
    case 0x80006A: case 0x800147: case 0x800259: case 0x81006A:
    case 0x82006A: case 0x84006A: case 0x88006A: case 0xA0006A:
      code = 0; break;              // rest / buttons: no change (as ZV)
    default:
      return false;                 // unknown pattern
    }
    rawCode = code;
    break;
  }
  case LEVER_JLR_G1:
    if (dlc < 8)
      return false;
    rawCode = d[3] >> 4; // knob position (ZV DirJLRG1)
    break;
  case LEVER_JLR_G2:
    if (dlc < 5)
      return false;
    rawCode = d[4] >> 4; // knob position (ZV DirJLRG2)
    break;
  case LEVER_PKP2300:
    if (dlc < 1)
      return false;
    rawCode = d[0]; // key bits
    break;
  default:
    return false;
  }
  if (rawCode == pendCode) {
    if (pendCount < 255)
      pendCount++;
  } else {
    pendCode = rawCode;
    pendCount = 1;
  }
  seen = true;
  msSinceRx = 0;
  return true;
}

// Timeout, debounce and the engage gate, every 10 ms.
void CanLever::Tick10ms(bool engageOk) {
  if (cfg.type == LEVER_BMW_GWS) {
    gws.Tick10ms(engageOk);
    return;
  }
  if (flashMs >= 10)
    flashMs -= 10;
  else
    flashMs = 0;
  if (msSinceRx < 0xFFFF - 10)
    msSinceRx += 10;
  if (!Valid())
    return;
  switch (cfg.type) {
  case LEVER_BMW_E65:
    // Momentary codes: act once per new code.
    if (pendCount >= cfg.debounce && pendCode != lastCode) {
      lastCode = pendCode;
      static const uint8_t e65[4] = {0xFF, GWS_P, GWS_R, GWS_D};
      if (pendCode >= 1 && pendCode <= 3)
        Request(e65[pendCode], engageOk);
    }
    break;
  case LEVER_JLR_G1:
  case LEVER_JLR_G2: {
    // Absolute knob: follow it whenever the gate allows; flash while it
    // points somewhere the TCU will not go yet.
    int want = pendCount >= cfg.debounce ? JlrToState(pendCode) : -1;
    if (want >= 0 && want != state) {
      if (want == GWS_N || engageOk)
        state = (uint8_t)want;
      else
        flashMs = 100;
    }
    break;
  }
  case LEVER_PKP2300: {
    // Keys: rising edges (ZV), one key per frame.
    uint8_t keys = pendCode;
    uint8_t pressed = keys & ~prevKeys;
    prevKeys = keys;
    if (pressed & PKP_KEY_D) Request(GWS_D, engageOk);
    if (pressed & PKP_KEY_R) Request(GWS_R, engageOk);
    if (pressed & PKP_KEY_N) Request(GWS_N, engageOk);
    if (pressed & PKP_KEY_P) Request(GWS_P, engageOk);
    break;
  }
  default:
    break;
  }
}

// Display / LED frames due this cycle (only sent with LeverTxEnable).
int CanLever::Tx10ms(const LeverTxIn &in, CanFrame out[LEVER_MAX_TX]) {
  int n = 0;
  uint8_t st = State();
  switch (cfg.type) {
  case LEVER_BMW_GWS:
    if (++txDiv >= 10) { // 100 ms
      txDiv = 0;
      out[n].id = GWS_TX_ID;
      out[n].dlc = 5;
      gwsDisp.Tick100ms(in.run, st, gws.Flash(), cfg.mSub, cfg.b34, in.gear, out[n].data);
      out[n].data[5] = out[n].data[6] = out[n].data[7] = 0;
      n++;
    }
    break;
  case LEVER_JLR_G1: {
    // ZV JLR_G1::sendcan every 20 ms; stops 2 s after leaving run.
    offCount = in.run ? 0 : (offCount < 200 ? offCount + 1 : 200);
    if (offCount >= 200 || ++txDiv < 2)
      break;
    txDiv = 0;
    if (!in.run)
      st = GWS_P; // ZV: park when not running
    uint8_t *b = out[n].data;
    out[n].id = 0x3F3;
    out[n].dlc = 8;
    b[0] = st == GWS_P ? 0x5C : 0x7C;
    b[2] = txCnt;
    switch (st) {
    case GWS_P: b[1] = 0x66; b[3] = txCnt + 130; b[4] = 0xFF; b[5] = 0x7F; b[6] = 0x00; b[7] = 0x80; break;
    case GWS_R: b[1] = 0x24; b[3] = txCnt + 3; b[4] = 0xFE; b[5] = 0xFF; b[6] = 0x01; b[7] = 0x00; break;
    case GWS_N: b[1] = 0x25; b[3] = txCnt + 4; b[4] = 0xFD; b[5] = 0xFF; b[6] = 0x02; b[7] = 0x00; break;
    case GWS_D: b[1] = 0x24; b[3] = txCnt + 6; b[4] = 0xFB; b[5] = 0xFF; b[6] = 0x04; b[7] = 0x00; break;
    default:    b[1] = 0x24; b[3] = txCnt + 10; b[4] = 0xF7; b[5] = 0xFF; b[6] = 0x08; b[7] = 0x00; break;
    }
    n++;
    txCnt++;
    if (st == GWS_P && txCnt == 0x02)
      txCnt = 0x04; // ZV: skips 2 and 3 in park
    if (txCnt == 0x0F)
      txCnt = 0;
    break;
  }
  case LEVER_JLR_G2: {
    // ZV JLR_G2::sendcan every 10 ms; stops 2 s after leaving run.
    offCount = in.run ? 0 : (offCount < 200 ? offCount + 1 : 200);
    if (offCount >= 200)
      break;
    if (!in.run)
      st = GWS_P;
    else
      knobLock = 0x40; // raise / unlock the knob (sticky, as ZV)
    uint8_t *b = out[n].data;
    out[n].id = 0x02C;
    out[n].dlc = 8;
    uint8_t c = txCnt & 0x0F;
    b[0] = 0x00;
    b[1] = 0x00;
    b[2] = (in.run ? 0x01 : 0x00) | (in.run ? 0x08 : 0x10); // on, raise / lower
    b[6] = 0x00;
    b[7] = 0x00;
    switch (st) {
    case GWS_P:
      b[3] = knobLock | 0x12; b[4] = g2Byte4[c]; b[5] = g2Byte5P[c]; b[7] = 0x80;
      break;
    case GWS_R:
      b[3] = knobLock | 0x00; b[4] = g2Byte4[c]; b[5] = g2Byte5R[c];
      if (in.selector == SEL_R) b[6] = 0x01;
      break;
    case GWS_N:
      b[3] = knobLock | 0x01; b[4] = g2Byte4[c]; b[5] = g2Byte5N[c];
      if (in.selector == SEL_N) b[6] = 0x02;
      break;
    case GWS_D:
      b[3] = knobLock | 0x04; b[4] = g2Byte4[c] | 0x0B; b[5] = g2Byte5D[c];
      if (in.selector == SEL_D) b[6] = 0x04;
      break;
    default: // Sport / M
      b[3] = knobLock | 0x04; b[4] = g2Byte4[c] | 0x0B; b[5] = g2Byte5S[c]; b[6] = 0x08;
      break;
    }
    n++;
    txCnt = (uint8_t)((txCnt + 1) & 0x0F);
    break;
  }
  case LEVER_PKP2300:
    if (++txDiv < 10) // 100 ms
      break;
    txDiv = 0;
    if (!Valid()) {
      // ZV: CANopen NMT "start remote node" while the keypad is silent.
      out[n].id = 0x000;
      out[n].dlc = 8;
      for (int i = 0; i < 8; i++)
        out[n].data[i] = 0;
      out[n].data[0] = 0x01;
      out[n].data[1] = PKP_NODE_ID;
      n++;
    }
    {
      // LEDs: byte 0 red, 1 green, 2 blue, one bit per key. The engaged
      // position blue; one red cycle for a rejected key (as ZV). The ZV
      // state-of-charge bar is VCU-only and not ported.
      uint8_t *b = out[n].data;
      out[n].id = PKP_RPDO1;
      out[n].dlc = 8;
      for (int i = 0; i < 8; i++)
        b[i] = 0;
      if (in.run)
        b[2] = PkpMask(st);
      if (flashOnce) {
        uint8_t m = PkpMask(rejected);
        b[0] |= m;
        b[2] &= ~m;
        flashOnce = false;
      }
      n++;
    }
    break;
  default:
    break; // E65: ZV sends nothing
  }
  return n;
}
