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
#include "tle92466.h"

// Channel base addresses (Table 24): CH0..CH3 at 0x40..0x70, CH4 at 0x20,
// CH5 at 0x30.
static const uint16_t chBase[TLE_CHANNELS] = { 0x40, 0x50, 0x60, 0x70, 0x20, 0x30 };

namespace Tle
{
uint8_t Crc8(uint32_t frame)
{
   uint8_t crc = 0xFF;
   for (int b = 0; b < 3; b++) // frame[7:0] first, then [15:8], then [23:16]
   {
      crc ^= (uint8_t)(frame >> (8 * b));
      for (int i = 0; i < 8; i++)
         crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
   }
   return crc ^ 0xFF;
}

static uint32_t AddCrc(uint32_t frame24)
{
   frame24 &= 0xFFFFFF;
   return ((uint32_t)Crc8(frame24) << 24) | frame24;
}

uint32_t WriteFrame(uint16_t addr, uint16_t data)
{
   // [23:17] = 7-bit address, [16] = 1 (write), [15:0] = data
   return AddCrc(((uint32_t)(addr & 0x7F) << 17) | (1ul << 16) | data);
}

uint32_t ReadFrame(uint16_t addr)
{
   // [16] = 0 (read), [15:0] = address
   return AddCrc(addr);
}

TleReply Parse(uint32_t frame)
{
   TleReply r;
   r.mode = (frame >> 22) & 3;
   r.status = 0;
   r.rw = false;
   if (r.mode == TLE_REPLY_22)
      r.data = frame & 0x3FFFFF;
   else if (r.mode == TLE_REPLY_CRITICAL)
      r.data = frame & 0xFF;
   else
   {
      r.status = (frame >> 17) & 0x1F;
      r.rw = (frame >> 16) & 1;
      r.data = frame & 0xFFFF;
   }
   // The critical fault frame has no CRC (Table 14: CRC bits "don't care").
   r.crcOk = r.mode == TLE_REPLY_CRITICAL || Crc8(frame) == (frame >> 24);
   return r;
}

uint16_t ChAddr(uint8_t ch, uint16_t offset)
{
   return ch < TLE_CHANNELS ? chBase[ch] + offset : 0xFFFF;
}

uint16_t SetpointCode(int32_t i)
{
   if (i <= 0)
      return 0;
   if (i > TLE_IMAX_0P1MA)
      i = TLE_IMAX_0P1MA;
   // I = 2 A x code / 32767, i in 0.1 mA (2 A = 20000)
   return (uint16_t)((i * 32767 + 10000) / 20000);
}

int32_t AvgCurrent(uint32_t fbIAvg, uint32_t fbDc)
{
   int32_t tp = fbDc & 0x7FF;          // TP_MANT[10:0]
   int32_t mant = fbIAvg & 0xFFF;      // I_AVG_MANT[11:0], signed
   if (mant & 0x800)
      mant -= 0x1000;
   if (tp == 0)
      return 0;
   // I = 4 A x I_AVG_MANT / TP_MANT, 4 A = 40000 x 0.1 mA
   return mant * 40000 / tp;
}

int32_t CoilMilliOhm(uint32_t fbDc, uint32_t fbVbat, uint32_t fbIAvg)
{
   int64_t tp = fbDc & 0x7FF;          // TP_MANT[10:0]
   int64_t to = (fbDc >> 11) & 0x7FF;  // TO_MANT[21:11]
   int64_t vm = fbVbat & 0x7FF;        // VBAT_AVG_MANT[10:0]
   int64_t im = fbIAvg & 0xFFF;        // I_AVG_MANT[11:0], signed
   if (im & 0x800)
      im -= 0x1000;
   if (tp <= 0 || im <= 0)
      return 0;
   // R = (41.47 V x vm / tp) x (to / tp) / (4 A x im / tp)
   //   = 41470 mV x vm x to / (4 x im x tp)  [milliohm]
   int64_t r = 41470 * vm * to / (4 * im * tp);
   return r > 1000000 ? 1000000 : (int32_t)r;
}

uint16_t PeriodReg(uint32_t hz)
{
   if (hz == 0)
      return 0; // MANT = 0 switches the frequency controller off
   uint32_t cycles = TLE_FSYS_HZ / hz; // tPWM = MANT x 2^EXP / fSYS
   uint32_t exp = 0;
   while ((cycles >> exp) > 255 && exp < 7)
      exp++;
   uint32_t mant = cycles >> exp;
   if (mant > 255)
      mant = 255;
   if (mant == 0)
      mant = 1;
   // PWM_CTRL_PARAM (bits 15:12) left at 0, as in Infineon's example.
   return (uint16_t)((exp << 8) | mant);
}

TleDither DitherRegs(int32_t amp, int32_t periodMs)
{
   TleDither d;
   // Dither period T = (4 x STEPS + 2 x FLAT) x tref (5.3.3.8). A triangle
   // with 16 steps per quarter and no flat part gives T = 64 tref.
   const uint32_t steps = 16;
   if (periodMs < 1)
      periodMs = 1;
   if (periodMs > 100)
      periodMs = 100;
   // tref = MANT x 2^EXP / fSYS, so MANT x 2^EXP = T x fSYS / 64
   uint32_t cyc = (uint32_t)periodMs * (TLE_FSYS_HZ / 1000) / 64;
   uint32_t exp = 0;
   while ((cyc >> exp) > 1023 && exp < 15)
      exp++;
   uint32_t mant = cyc >> exp;
   if (mant == 0)
      mant = 1;
   d.clkDiv = (uint16_t)((exp << 10) | (mant & 0x3FF));
   d.step = (uint16_t)(steps << 8); // FLAT = 0
   // Peak amplitude I_dither = STEPS x STEP_SIZE x 2 A / 32767 (5.3.3.9);
   // amp in 0.1 mA, 2 A = 20000.
   int32_t size = 0;
   if (amp > 0)
      size = (int32_t)(((uint32_t)amp * 32767 + 10000 * steps) / (20000 * steps));
   if (size > 0xFFF)
      size = 0xFFF;
   d.ctrl = (uint16_t)size; // DEEP_DITHER off, FAST_MEAS = one dither period
   return d;
}

uint16_t ChConfig(uint8_t slew, bool olCheck, uint8_t olRelEighths)
{
   uint16_t v = slew & 3; // I_DIAG = 80 uA (reset value)
   if (olCheck)
   {
      v |= (uint16_t)((olRelEighths & 7) << 4); // OL_TH, OFF_DIAG_CH = 00 (on)
   }
   else
   {
      // Both open-load thresholds 0 switches the ON-state open-load /
      // short check off; OFF_DIAG_CH = 11 switches the OFF-state check off.
      v |= 3u << 14;
   }
   return v;
}

int32_t TempC(uint32_t fbVoltage2)
{
   int32_t v = fbVoltage2 & 0x7FF;
   // T = (v x 0.000593 - 0.819) / (-0.0016) degC
   return (819000 - 593 * v) / 1600;
}

int32_t VbatmV(uint32_t fbVoltage2)
{
   int32_t v = (fbVoltage2 >> 11) & 0x7FF;
   return (v * 41470 + 1023) / 2047; // 41.47 V x v / 2047, rounded
}
}

Tle92466::Tle92466(Tle92466Bus &b, uint8_t d)
   : bus(b), dev(d), tempC(0), vbatmV(0), commErrors(0), up(false)
{
   cfg = TleConfig();
   for (int i = 0; i < TLE_CHANNELS; i++)
   {
      want[i] = 0;
      sent[i] = -1;
      wantDith[i] = 0;
      sentDith[i] = -1;
      meas[i] = 0;
   }
}

uint32_t Tle92466::Xfer(uint32_t frame)
{
   bus.Select(dev, true);
   uint32_t hi = bus.Transfer16((uint16_t)(frame >> 16));
   uint32_t lo = bus.Transfer16((uint16_t)frame);
   bus.Select(dev, false);
   return (hi << 16) | lo;
}

bool Tle92466::Exchange(const uint32_t *frames, int n, uint32_t *replies)
{
   bool ok = true;
   for (int i = 0; i <= n; i++)
   {
      // The last frame only collects the answer to frame n-1.
      uint32_t r = Xfer(i < n ? frames[i] : Tle::ReadFrame(TLE_ICVID));
      if (i == 0)
         continue; // answers whatever was sent before
      TleReply p = Tle::Parse(r);
      bool good = p.crcOk && p.mode != TLE_REPLY_CRITICAL &&
                  !(p.mode == TLE_REPLY_16 && p.status != 0);
      if (!good)
      {
         ok = false;
         commErrors++;
      }
      if (replies)
         replies[i - 1] = r;
   }
   return ok;
}

bool Tle92466::Write(uint16_t addr, uint16_t data)
{
   uint32_t f = Tle::WriteFrame(addr, data);
   return Exchange(&f, 1, 0);
}

bool Tle92466::Read(uint16_t addr, uint32_t *data)
{
   uint32_t f = Tle::ReadFrame(addr), r = 0;
   bool ok = Exchange(&f, 1, &r);
   *data = Tle::Parse(r).data;
   return ok;
}

bool Tle92466::Init(const TleConfig &c)
{
   cfg = c;
   up = false;
   uint32_t v = 0;

   // Wait for the chip's own start-up (INIT_DONE, datasheet 4.3.2).
   bool done = false;
   for (int tries = 0; tries < 50 && !done; tries++)
      done = Read(TLE_FB_STAT, &v) && (v & TLE_FS_INIT_DONE);
   if (!done)
      return false;

   // Back to Config Mode (all channels off) so the mode registers can be
   // written. A chip that dropped to Config Mode by itself is there already.
   Write(TLE_CH_CTRL, 0);
   // VIO = 3.3 V, CRC on, clock watchdog on, SPI watchdog off.
   Write(TLE_GLOBAL_CONFIG, TLE_GC_CLK_WD_EN | TLE_GC_CRC_EN);

   for (uint8_t ch = 0; ch < TLE_CHANNELS; ch++)
   {
      uint8_t bit = 1u << ch;
      bool used = cfg.usedMask & bit;
      TleDither d = Tle::DitherRegs(0, TLE_DITHER_MS); // amplitude follows in Flush()
      uint32_t f[7] =
      {
         Tle::WriteFrame(Tle::ChAddr(ch, TLE_SETPOINT), 0),
         Tle::WriteFrame(Tle::ChAddr(ch, TLE_MODE), used ? TLE_MODE_ICC : TLE_MODE_OFF),
         Tle::WriteFrame(Tle::ChAddr(ch, TLE_CH_CONFIG),
                         Tle::ChConfig(cfg.slew, used && (cfg.olMask & bit), cfg.olRelEighths)),
         Tle::WriteFrame(Tle::ChAddr(ch, TLE_PERIOD), Tle::PeriodReg(cfg.pwmHz)),
         // The current feedback is measured over one dither period, so the
         // dither clock is always set, even with no dither amplitude.
         Tle::WriteFrame(Tle::ChAddr(ch, TLE_DITHER_CLK_DIV), d.clkDiv),
         Tle::WriteFrame(Tle::ChAddr(ch, TLE_DITHER_STEP), d.step),
         Tle::WriteFrame(Tle::ChAddr(ch, TLE_DITHER_CTRL), d.ctrl),
      };
      if (!Exchange(f, 7, 0))
         return false;
      sent[ch] = -1;
      sentDith[ch] = d.ctrl;
   }

   // Faults that pull FAULTN low: supply faults and channel errors of the
   // used channels only (unused outputs are not wired, their open load is
   // expected). Warnings (FAULT_MASK1) keep only the chip temperature and clock.
   uint32_t m[2] =
   {
      Tle::WriteFrame(TLE_FAULT_MASK0, 0xC000 | cfg.usedMask),
      Tle::WriteFrame(TLE_FAULT_MASK1, 0x7000),
   };
   Exchange(m, 2, 0);

   // Clear the start-up flags (POR, the VIO fault seen before VIO_SEL was
   // set) and any channel flags.
   uint32_t clr[6];
   for (int i = 0; i < 3; i++)
      clr[i] = Tle::WriteFrame(TLE_GLOBAL_DIAG0 + i, 0);
   for (int i = 0; i < 3; i++)
      clr[3 + i] = Tle::WriteFrame(TLE_DIAG_ERR_CHGR(i), 0);
   Exchange(clr, 6, 0);
   Exchange(clr, 6, 0);

   // Mission Mode first, then the channel enables (4.3.4).
   Write(TLE_CH_CTRL, TLE_OP_MODE_MISSION);
   Write(TLE_CH_CTRL, TLE_OP_MODE_MISSION | cfg.usedMask);
   if (!Read(TLE_CH_CTRL, &v) || (v & (TLE_OP_MODE_MISSION | 0x3F)) !=
       (uint32_t)(TLE_OP_MODE_MISSION | cfg.usedMask))
      return false;
   up = true;
   return true;
}

void Tle92466::SetCurrent(uint8_t ch, int32_t i, int32_t dith)
{
   if (ch >= TLE_CHANNELS)
      return;
   bool used = cfg.usedMask & (1u << ch);
   want[ch] = used && i > 0 ? i : 0;
   wantDith[ch] = used && dith > 0 ? dith : 0;
}

void Tle92466::Flush()
{
   if (!up)
      return;
   uint32_t f[2 * TLE_CHANNELS];
   int n = 0;
   for (uint8_t ch = 0; ch < TLE_CHANNELS; ch++)
   {
      if (!(cfg.usedMask & (1u << ch)))
         continue;
      int32_t dith = Tle::DitherRegs(wantDith[ch], TLE_DITHER_MS).ctrl;
      if (dith != sentDith[ch])
      {
         f[n++] = Tle::WriteFrame(Tle::ChAddr(ch, TLE_DITHER_CTRL), (uint16_t)dith);
         sentDith[ch] = dith;
      }
      int32_t code = Tle::SetpointCode(want[ch]);
      if (code != sent[ch])
      {
         f[n++] = Tle::WriteFrame(Tle::ChAddr(ch, TLE_SETPOINT), (uint16_t)code);
         sent[ch] = code;
      }
   }
   if (n > 0 && !Exchange(f, n, 0))
      for (uint8_t ch = 0; ch < TLE_CHANNELS; ch++)
         sent[ch] = sentDith[ch] = -1; // try again next time
}

bool Tle92466::ReadDiag(TleDiag *d)
{
   const uint32_t f[6] =
   {
      Tle::ReadFrame(TLE_FB_STAT),
      Tle::ReadFrame(TLE_GLOBAL_DIAG0),
      Tle::ReadFrame(TLE_DIAG_ERR_CHGR(0)),
      Tle::ReadFrame(TLE_DIAG_ERR_CHGR(1)),
      Tle::ReadFrame(TLE_DIAG_ERR_CHGR(2)),
      Tle::ReadFrame(TLE_CH_CTRL),
   };
   uint32_t r[6];
   d->commOk = Exchange(f, 6, r);
   d->critical = false;
   for (int i = 0; i < 6; i++)
      if (Tle::Parse(r[i]).mode == TLE_REPLY_CRITICAL)
         d->critical = true;
   d->fbStat = Tle::Parse(r[0]).data;
   d->diag0 = (uint16_t)Tle::Parse(r[1]).data;
   d->ol = d->sg = d->oc = d->olsg = d->ote = 0;
   for (int g = 0; g < 3; g++)
   {
      uint32_t e = Tle::Parse(r[2 + g]).data;
      for (int half = 0; half < 2; half++)
      {
         uint8_t bits = (uint8_t)(e >> (8 * half));
         uint8_t chBit = 1u << (2 * g + half);
         if (bits & TLE_DE_OLSG) d->olsg |= chBit;
         if (bits & TLE_DE_OL) d->ol |= chBit;
         if (bits & TLE_DE_OC) d->oc |= chBit;
         if (bits & TLE_DE_SG) d->sg |= chBit;
         if (bits & TLE_DE_OTE) d->ote |= chBit;
      }
   }
   uint32_t ctrl = Tle::Parse(r[5]).data;
   d->mission = d->commOk && (ctrl & TLE_OP_MODE_MISSION);
   d->enabled = ctrl & 0x3F;
   // Unused channels are not wired: forget their flags.
   uint8_t used = cfg.usedMask;
   d->ol &= used; d->sg &= used; d->oc &= used; d->olsg &= used; d->ote &= used;

   if (!d->commOk)
      return false;

   // Clear what was reported. The datasheet calls the flags "clear on
   // write" without saying which value clears them, so each group is
   // written with its own value (clears if writing 1 clears) and then with
   // 0 (clears if writing 0 clears), each twice because an over-current
   // flag needs two clears (4.11.2). Writing 0 to a 1-clears bit, or the
   // flag back to a 0-clears bit, changes nothing.
   uint32_t clr[16];
   int n = 0;
   for (int g = 0; g < 3; g++)
   {
      uint16_t e = (uint16_t)Tle::Parse(r[2 + g]).data;
      if (e == 0)
         continue;
      clr[n++] = Tle::WriteFrame(TLE_DIAG_ERR_CHGR(g), e);
      clr[n++] = Tle::WriteFrame(TLE_DIAG_ERR_CHGR(g), e);
      clr[n++] = Tle::WriteFrame(TLE_DIAG_ERR_CHGR(g), 0);
      clr[n++] = Tle::WriteFrame(TLE_DIAG_ERR_CHGR(g), 0);
   }
   if (d->diag0)
   {
      clr[n++] = Tle::WriteFrame(TLE_GLOBAL_DIAG0, d->diag0);
      clr[n++] = Tle::WriteFrame(TLE_GLOBAL_DIAG0, 0);
   }
   // A channel the chip switched off is enabled again (only in Mission
   // Mode; leaving Config Mode is the board code's call, see ValveOut).
   if (d->mission && (d->enabled & used) != used)
      clr[n++] = Tle::WriteFrame(TLE_CH_CTRL, TLE_OP_MODE_MISSION | used);
   if (n > 0)
      Exchange(clr, n, 0);
   if (!d->mission)
      up = false;
   return true;
}

void Tle92466::ReadCurrents()
{
   uint32_t f[3 * TLE_CHANNELS + 1];
   uint32_t r[3 * TLE_CHANNELS + 1];
   uint8_t chs[TLE_CHANNELS];
   int n = 0, k = 0;
   for (uint8_t ch = 0; ch < TLE_CHANNELS; ch++)
   {
      if (!(cfg.usedMask & (1u << ch)))
         continue;
      chs[k++] = ch;
      f[n++] = Tle::ReadFrame(Tle::ChAddr(ch, TLE_FB_DC));
      f[n++] = Tle::ReadFrame(Tle::ChAddr(ch, TLE_FB_VBAT));
      f[n++] = Tle::ReadFrame(Tle::ChAddr(ch, TLE_FB_I_AVG));
   }
   f[n++] = Tle::ReadFrame(TLE_FB_VOLTAGE2);
   bool ok = Exchange(f, n, r);
   for (int i = 0; i < k; i++)
   {
      uint8_t ch = chs[i];
      uint32_t dc = Tle::Parse(r[3 * i]).data;
      uint32_t vb = Tle::Parse(r[3 * i + 1]).data;
      uint32_t ia = Tle::Parse(r[3 * i + 2]).data;
      meas[ch] = ok ? Tle::AvgCurrent(ia, dc) : 0;
      rCoil[ch] = ok ? Tle::CoilMilliOhm(dc, vb, ia) : 0;
   }
   if (ok)
   {
      uint32_t v2 = Tle::Parse(r[n - 1]).data;
      tempC = Tle::TempC(v2);
      vbatmV = Tle::VbatmV(v2);
   }
}
