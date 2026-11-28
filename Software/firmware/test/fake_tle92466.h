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
#ifndef FAKE_TLE92466_H
#define FAKE_TLE92466_H

#include "tle92466.h"
#include <string.h>

/* A small software model of the TLE92466ED SPI interface, enough to test
 * the driver: 32-bit frames as two 16-bit words, the CRC check, the reply
 * to a frame coming back during the next frame, 16-bit and 22-bit
 * replies, Config / Mission Mode and the enable bits, channel faults that
 * switch a channel off, and the feedback registers at base + 0x200.
 *
 * The real chip's "clear on write" diagnosis bits are modelled as
 * "writing 0 clears" (clr1 = false) or "writing 1 clears" (clr1 = true);
 * the driver must work with both. */
class FakeTleBus : public Tle92466Bus
{
public:
   static const int NDEV = 2;
   uint32_t reg[NDEV][0x300];
   uint32_t pending[NDEV]; // reply for the next frame
   int sel;
   int word;
   uint32_t shiftIn;
   int frames;
   int writes[NDEV][0x80]; // write count per address
   bool clr1;              // diagnosis bits clear on writing 1
   bool corruptNext;       // send the next reply with a bad CRC
   bool critical[NDEV];    // answer with the critical fault frame
   bool dead[NDEV];        // chip does not answer (MISO pulled up)

   FakeTleBus() { Reset(); }
   void Reset()
   {
      memset(reg, 0, sizeof(reg));
      memset(writes, 0, sizeof(writes));
      for (int d = 0; d < NDEV; d++)
      {
         reg[d][TLE_GLOBAL_CONFIG] = 0x4005;
         reg[d][TLE_FB_STAT] = TLE_FS_INIT_DONE;
         // Power-on flag, and VIO "over voltage" until VIO_SEL says 3.3 V
         // (the reset value expects 5 V).
         reg[d][TLE_GLOBAL_DIAG0] = (1u << 10) | (1u << 2);
         reg[d][TLE_ICVID] = 0xC100;
         pending[d] = 0;
         critical[d] = false;
         dead[d] = false;
         for (int ch = 0; ch < TLE_CHANNELS; ch++)
            reg[d][Tle::ChAddr(ch, TLE_CH_CONFIG)] = 0x0003;
      }
      sel = -1;
      word = 0;
      shiftIn = 0;
      frames = 0;
      clr1 = false;
      corruptNext = false;
   }

   bool Mission(int d) const { return reg[d][TLE_CH_CTRL] & TLE_OP_MODE_MISSION; }
   uint8_t Enabled(int d) const { return reg[d][TLE_CH_CTRL] & 0x3F; }
   uint16_t Ch(int d, int ch, uint16_t off) const { return (uint16_t)reg[d][Tle::ChAddr(ch, off)]; }

   /// A channel fault: sets the flag and, like the chip, clears EN_CH.
   void ChannelFault(int d, int ch, uint8_t bits)
   {
      reg[d][TLE_DIAG_ERR_CHGR(ch / 2)] |= (uint32_t)bits << (8 * (ch & 1));
      reg[d][TLE_CH_CTRL] &= ~(1u << ch);
   }
   /// A supply fault: the chip falls back to Config Mode by itself.
   void DropToConfig(int d, uint16_t diag0Bits)
   {
      reg[d][TLE_GLOBAL_DIAG0] |= diag0Bits;
      reg[d][TLE_CH_CTRL] = 0;
   }
   /// Measured current for the feedback registers (TP_MANT 1000).
   void SetMeasured(int d, int ch, int32_t i_0p1mA)
   {
      int32_t mant = i_0p1mA * 1000 / 40000;
      reg[d][Tle::ChAddr(ch, TLE_FB_DC)] = 1000;
      reg[d][Tle::ChAddr(ch, TLE_FB_I_AVG)] = (uint32_t)mant & 0xFFF;
   }

   void Select(uint8_t dev, bool active) override
   {
      if (active)
      {
         sel = dev;
         word = 0;
         shiftIn = 0;
      }
      else
      {
         if (sel >= 0 && word == 2)
            Frame(sel, shiftIn);
         sel = -1;
      }
   }

   uint16_t Transfer16(uint16_t tx) override
   {
      if (sel < 0 || dead[sel])
         return 0xFFFF; // pull-up on MISO
      uint32_t out = pending[sel];
      shiftIn = (shiftIn << 16) | tx;
      uint16_t r = word == 0 ? (uint16_t)(out >> 16) : (uint16_t)out;
      word++;
      return r;
   }

private:
   static uint32_t WithCrc(uint32_t f24)
   {
      f24 &= 0xFFFFFF;
      return ((uint32_t)Tle::Crc8(f24) << 24) | f24;
   }
   static bool Is22(uint16_t a)
   {
      if (a == TLE_FB_STAT || a == TLE_FB_VOLTAGE1 || a == TLE_FB_VOLTAGE2)
         return true;
      return a >= 0x220 && a < 0x280 && (a & 0xF) <= 3; // channel feedback
   }
   uint32_t Clear(uint32_t old, uint32_t data) const
   {
      return clr1 ? old & ~data : old & data;
   }

   void Frame(int d, uint32_t f)
   {
      frames++;
      if (dead[d])
         return;
      uint32_t reply;
      if (critical[d])
      {
         pending[d] = (2ul << 22) | 0x01; // critical fault frame, no CRC
         return;
      }
      if (Tle::Crc8(f) != (f >> 24))
      {
         pending[d] = WithCrc((2ul << 17)); // status 2 = CRC error
         return;
      }
      bool write = (f >> 16) & 1;
      if (write)
      {
         uint16_t a = (f >> 17) & 0x7F;
         uint16_t v = (uint16_t)f;
         writes[d][a]++;
         bool mission = Mission(d);
         if (a == TLE_CH_CTRL)
         {
            if (!(v & TLE_OP_MODE_MISSION))
               v &= ~0x3F; // Config Mode: every channel off
            else if (!mission)
               v &= ~0x3F; // EN_CH only takes in Mission Mode
            reg[d][a] = v;
         }
         else if (a == TLE_GLOBAL_CONFIG || (a >= 0x20 && (a & 0xF) == TLE_MODE))
         {
            if (!mission)
               reg[d][a] = v; // writable in Config Mode only
         }
         else if ((a >= TLE_GLOBAL_DIAG0 && a <= TLE_GLOBAL_DIAG2) ||
                  (a >= TLE_DIAG_ERR_CHGR(0) && a <= TLE_DIAG_ERR_CHGR(2)))
         {
            reg[d][a] = Clear(reg[d][a], v);
         }
         else
         {
            reg[d][a] = v;
         }
         reply = WithCrc((1ul << 16) | (reg[d][a] & 0xFFFF));
      }
      else
      {
         uint16_t a = (uint16_t)f;
         uint32_t v = a < 0x300 ? reg[d][a] : 0;
         if (a == TLE_FB_STAT)
         {
            // CLK_NOK .. POR_EVENT mirror GLOBAL_DIAG0 bits 6..10, and
            // SUP_NOK_EXT sums up its supply bits.
            v = (v & ~0x17C0ul) | (reg[d][TLE_GLOBAL_DIAG0] & 0x7C0);
            if (reg[d][TLE_GLOBAL_DIAG0] & 0x3F)
               v |= TLE_FS_SUP_NOK_EXT;
         }
         if (Is22(a))
            reply = WithCrc((1ul << 22) | (v & 0x3FFFFF));
         else
            reply = WithCrc(v & 0xFFFF);
      }
      if (corruptNext)
      {
         reply ^= 0x01000000; // flip a CRC bit
         corruptNext = false;
      }
      pending[d] = reply;
   }
};

#endif // FAKE_TLE92466_H
