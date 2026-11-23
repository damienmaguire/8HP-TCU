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
#ifndef FAKE_MAX22200_H
#define FAKE_MAX22200_H

#include "max22200.h"
#include <string.h>

/* A small software model of the MAX22200 SPI interface, enough to test
 * the driver: command frames (CMD high), 8 and 32 bit data frames, the
 * write echo, big-endian reads and the selective FAULT clear. */
class FakeMaxBus : public Max22200Bus
{
public:
   static const int NDEV = 2;
   uint32_t reg[NDEV][11];
   uint8_t pendingCmd[NDEV];
   bool cmdHigh;
   int sel;           // selected device, -1 = none
   bool frameIsCmd;
   int byteInFrame;
   uint32_t shiftIn;
   uint32_t clearMask;
   int frames;        // completed frames
   bool cmdHighAtRise; // CMD level when the last command frame ended
   bool badEcho;      // corrupt the echo byte (to test the check)
   int write8Count;

   FakeMaxBus() { Reset(); }
   void Reset()
   {
      memset(reg, 0, sizeof(reg));
      memset(pendingCmd, 0, sizeof(pendingCmd));
      for (int d = 0; d < NDEV; d++)
         reg[d][MAX22200_REG_STATUS] = MAX22200_ST_UVM; // set at power-up
      cmdHigh = false;
      sel = -1;
      frameIsCmd = false;
      byteInFrame = 0;
      shiftIn = 0;
      clearMask = 0;
      frames = 0;
      cmdHighAtRise = false;
      badEcho = false;
      write8Count = 0;
   }
   uint8_t Status8(int d) const { return reg[d][0] & 0xFF; }

   void Select(uint8_t dev, bool active) override
   {
      if (active)
      {
         sel = dev;
         frameIsCmd = cmdHigh;
         byteInFrame = 0;
         shiftIn = 0;
         clearMask = 0;
      }
      else
      {
         EndFrame();
         sel = -1;
      }
   }
   void SetCmd(bool high) override { cmdHigh = high; }

   uint8_t Transfer(uint8_t tx) override
   {
      int d = sel;
      if (d < 0)
         return 0xFF;
      uint8_t ret = 0;
      if (frameIsCmd)
      {
         pendingCmd[d] = tx;
         ret = Status8(d);
      }
      else
      {
         uint8_t cmd = pendingCmd[d];
         bool write = cmd & 0x80;
         bool eight = cmd & 0x01;
         uint8_t r = (cmd >> 1) & 0x0F;
         if (write && !eight)
         {
            // LSB first; SDO: STATUS, command echo, 0, 0
            shiftIn |= (uint32_t)tx << (8 * byteInFrame);
            ret = byteInFrame == 0 ? Status8(d)
                  : byteInFrame == 1 ? (uint8_t)(badEcho ? cmd ^ 1 : cmd) : 0;
         }
         else if (write && eight)
         {
            shiftIn = tx;
            ret = Status8(d);
         }
         else if (!eight)
         {
            // read, MSB first; SDI bits are the clear mask for FAULT
            ret = (uint8_t)(reg[d][r] >> (8 * (3 - byteInFrame)));
            clearMask |= (uint32_t)tx << (8 * (3 - byteInFrame));
         }
      }
      byteInFrame++;
      return ret;
   }

private:
   void EndFrame()
   {
      int d = sel;
      if (d < 0)
         return;
      frames++;
      if (frameIsCmd)
      {
         cmdHighAtRise = cmdHigh;
         return;
      }
      uint8_t cmd = pendingCmd[d];
      bool write = cmd & 0x80;
      bool eight = cmd & 0x01;
      uint8_t r = (cmd >> 1) & 0x0F;
      if (r > 10)
         return;
      if (write && !eight && byteInFrame == 4)
      {
         if (r == MAX22200_REG_STATUS)
            // flags in [7:1] are read-only, ACTIVE is writable
            reg[d][0] = (shiftIn & 0xFFFFFF01u) | (reg[d][0] & 0xFE);
         else
            reg[d][r] = shiftIn;
      }
      else if (write && eight && byteInFrame == 1)
      {
         write8Count++;
         reg[d][r] = (reg[d][r] & 0x00FFFFFFu) | (shiftIn << 24);
      }
      else if (!write && !eight)
      {
         if (r == MAX22200_REG_STATUS)
            reg[d][0] &= ~MAX22200_ST_UVM; // reading STATUS clears UVM
         if (r == MAX22200_REG_FAULT)
            reg[d][r] &= ~clearMask; // MAX22200A selective clear
      }
   }
};

#endif // FAKE_MAX22200_H
