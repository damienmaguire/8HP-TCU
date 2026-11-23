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
#include "max22200.h"

Max22200::Max22200(Max22200Bus &b, uint8_t d)
    : bus(b), dev(d), cfg(), maxCode(), onMask(0), lastFlags(0),
      echoError(false) {
  for (int i = 0; i < MAX22200_CHANNELS; i++) {
    holdCode[i] = 0;
    hitCode[i] = 0;
    hitTime[i] = 0;
  }
}

uint8_t Max22200::CommandByte(uint8_t reg, bool write, bool eightBit) {
  // Table 9: [7] RB/W, [6:5] reserved (0), [4:1] A_BNK, [0] 8bit/n32bits
  return (uint8_t)((write ? MAX22200_CMD_WRITE : 0) | ((reg & 0x0F) << 1) |
                   (eightBit ? MAX22200_CMD_8BIT : 0));
}

uint8_t Max22200::CurrentToCode(int32_t current_0p1mA, uint16_t ifsmA,
                                bool halfScale, uint8_t maxCode) {
  // Table 4: I_HOLD = HOLD / 127 x IFS, and HFS = 1 halves IFS.
  int32_t fs = (int32_t)ifsmA * 10; // 0.1 mA
  if (halfScale)
    fs /= 2;
  if (current_0p1mA <= 0 || fs <= 0)
    return 0;
  int32_t code = (current_0p1mA * 127 + fs / 2) / fs;
  if (code < 1)
    code = 1; // any non-zero request gets the smallest regulated step
  if (maxCode > MAX22200_CODE_MAX)
    maxCode = MAX22200_CODE_MAX;
  if (code > maxCode)
    code = maxCode;
  return (uint8_t)code;
}

int32_t Max22200::CodeToCurrent(uint8_t code, uint16_t ifsmA, bool halfScale) {
  int32_t fs = (int32_t)ifsmA * 10;
  if (halfScale)
    fs /= 2;
  if (code > MAX22200_CODE_MAX)
    code = MAX22200_CODE_MAX;
  return (code * fs + 63) / 127;
}

uint32_t Max22200::CfgWord(const Max22200Config &c, bool hfs, uint8_t hold, uint8_t hit,
                           uint8_t hitT) {
  if (hold > MAX22200_CODE_MAX)
    hold = MAX22200_CODE_MAX;
  if (hit > MAX22200_CODE_MAX)
    hit = MAX22200_CODE_MAX;
  uint32_t w = 0;
  if (hfs)
    w |= MAX22200_CFG_HFS;
  w |= (uint32_t)(hold & 0x7F) << MAX22200_CFG_HOLD_SHIFT;
  // TRGnSPI = 0: channel switched by its ONCH bit over SPI, not TRIGA/B.
  w |= (uint32_t)(hit & 0x7F) << MAX22200_CFG_HIT_SHIFT;
  w |= (uint32_t)hitT << MAX22200_CFG_HIT_T_SHIFT;
  // VDRnCDR = 0 (current drive), HSnLS = 0 (low side).
  w |= (uint32_t)(c.freqDiv & 3) << MAX22200_CFG_FREQ_SHIFT;
  if (c.slewControl)
    w |= MAX22200_CFG_SRC;
  if (c.openLoadDetect)
    w |= MAX22200_CFG_OL_EN;
  // HIT-not-reached check only makes sense with a HIT phase.
  if (hit != 0 && hitT != 0)
    w |= MAX22200_CFG_HHF_EN;
  return w;
}

/* Command register write: CMD high for the whole transfer and still high
 * at the CSB rising edge (Figure 7). SDO returns STATUS[7:0].
 */
uint8_t Max22200::WriteCommand(uint8_t cmd) {
  bus.SetCmd(true);
  bus.Select(dev, true);
  uint8_t flags = bus.Transfer(cmd);
  bus.Select(dev, false);
  bus.SetCmd(false);
  lastFlags = flags;
  return flags;
}

/* 32-bit write (Figure 10): the data byte order on SDI is DATAIN[7:0]
 * first and DATAIN[31:24] last. SDO returns STATUS[7:0], then the command
 * register, then two zero bytes, which lets us check the command took.
 */
void Max22200::Write32(uint8_t reg, uint32_t value) {
  uint8_t cmd = CommandByte(reg, true, false);
  WriteCommand(cmd);
  bus.Select(dev, true);
  bus.Transfer((uint8_t)(value & 0xFF));
  uint8_t echo = bus.Transfer((uint8_t)((value >> 8) & 0xFF));
  bus.Transfer((uint8_t)((value >> 16) & 0xFF));
  bus.Transfer((uint8_t)((value >> 24) & 0xFF));
  bus.Select(dev, false);
  if (echo != cmd)
    echoError = true;
}

/* 8-bit write (Figure 12): one byte, written to the register's top byte
 * (bits 31:24). SDO returns STATUS[7:0].
 */
void Max22200::Write8(uint8_t reg, uint8_t value) {
  WriteCommand(CommandByte(reg, true, true));
  bus.Select(dev, true);
  bus.Transfer(value);
  bus.Select(dev, false);
}

/* 32-bit read (Figure 11): SDO returns DATA[31:24] first. On the
 * MAX22200A a 1 on SDI while reading FAULT clears that bit; SDI follows
 * the same bit order as SDO.
 */
uint32_t Max22200::Read32(uint8_t reg, uint32_t sdiMask) {
  WriteCommand(CommandByte(reg, false, false));
  bus.Select(dev, true);
  uint32_t v = 0;
  for (int i = 3; i >= 0; i--)
    v = (v << 8) | bus.Transfer((uint8_t)((sdiMask >> (8 * i)) & 0xFF));
  bus.Select(dev, false);
  return v;
}

bool Max22200::Init(const Max22200Config &c) {
  cfg = c;
  for (int h = 0; h < 2; h++)
    maxCode[h] = CurrentToCode((int32_t)cfg.maxCurrentmA * 10, cfg.ifsmA,
                               h != 0, MAX22200_CODE_MAX);
  echoError = false;

  // Step 1: read STATUS to clear UVM and release nFAULT.
  Read32(MAX22200_REG_STATUS, 0);

  // Step 2: hardware configuration. All channels off, all pairs
  // independent (CMxy = 0), no fault masked, ACTIVE = 1.
  uint32_t st = MAX22200_ST_ACTIVE;
  if (cfg.freq80k)
    st |= MAX22200_FREQM_80K;
  Write32(MAX22200_REG_STATUS, st);
  onMask = 0;

  // Step 3: configure every channel with zero current.
  for (uint8_t ch = 0; ch < MAX22200_CHANNELS; ch++) {
    holdCode[ch] = 0;
    Write32(MAX22200_REG_CFG_CH(ch),
            CfgWord(cfg, Hfs(ch), 0, hitCode[ch], hitTime[ch]));
  }

  // Step 4: read STATUS again; UVM and the other flags must be clear.
  uint32_t s = Read32(MAX22200_REG_STATUS, 0);
  return ((s & MAX22200_ST_FAULTS) == 0) && ((s & MAX22200_ST_ACTIVE) != 0) &&
         !echoError;
}

void Max22200::SetHit(uint8_t ch, const Max22200Hit &hit) {
  ch &= 7;
  hitCode[ch] = CurrentToCode((int32_t)hit.hitmA * 10, cfg.ifsmA,
                              Hfs(ch), maxCode[Hfs(ch)]);
  hitTime[ch] = hit.hitTime;
  Write32(MAX22200_REG_CFG_CH(ch),
          CfgWord(cfg, Hfs(ch), holdCode[ch], hitCode[ch], hitTime[ch]));
}

void Max22200::SetCurrent(uint8_t ch, int32_t current_0p1mA) {
  ch &= 7;
  uint8_t code =
      CurrentToCode(current_0p1mA, cfg.ifsmA, Hfs(ch), maxCode[Hfs(ch)]);
  if (code == holdCode[ch])
    return;
  holdCode[ch] = code;
  // Top byte of CFG_CHx is HFS + HOLD[6:0].
  Write8(MAX22200_REG_CFG_CH(ch),
         (uint8_t)((Hfs(ch) ? 0x80 : 0) | (code & 0x7F)));
}

void Max22200::SetOnMask(uint8_t mask) {
  if (mask == onMask)
    return;
  onMask = mask;
  Write8(MAX22200_REG_STATUS, mask); // STATUS[31:24] = ONCH[7:0]
}

Max22200Faults Max22200::ReadFaults() {
  Max22200Faults f;
  uint32_t st = Read32(MAX22200_REG_STATUS, 0);
  uint32_t fault = Read32(MAX22200_REG_FAULT, 0);
  if (fault != 0)
    Read32(MAX22200_REG_FAULT, fault); // selective clear (MAX22200A)
  f.status = (uint8_t)(st & 0xFF);
  f.ocp = (uint8_t)(fault >> 24);
  f.hhf = (uint8_t)(fault >> 16);
  f.olf = (uint8_t)(fault >> 8);
  f.dpm = (uint8_t)fault;
  return f;
}
