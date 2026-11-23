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
#ifndef SHIFTER_H
#define SHIFTER_H

#include <stdint.h>

/* Gear selector front end: turns either the CAN shifter fields or the
 * V4 digital/analogue inputs into a selector position (P/R/N/D) and
 * one-shot tip up / tip down requests for Manual mode.
 *
 * ShifterType (param_prj.h):
 *  CAN     placeholder fields in the CAN1 command frame (CanCmdId):
 *          byte 0 selector 0=P 1=R 2=N 3=D, byte 6 bit 0 tip up, bit 1
 *          tip down (level, held while the paddle/lever is held).
 *  Analog  PLACEHOLDER mapping until the real shifter is chosen:
 *          park_in high and notpark_in low   -> P
 *          notpark_in high and park_in low   -> R/N/D from ana_in:
 *              below ShAnaR mV -> R, below ShAnaN mV -> N, else D
 *          park_in == notpark_in             -> implausible: N + fault
 *          ana_in outside ShAnaMin..ShAnaMax -> N + fault
 *          din1 = tip up, din2 = tip down (active high)
 *
 * Every input must be stable for two 10 ms samples before it is taken
 * (the board RC is only about 1.1 ms). A tip gives one request on the
 * rising edge; holding it does not repeat.
 */

enum ShifterType
{
   SHIFTER_CAN = 0,
   SHIFTER_ANALOG = 1,
};

struct ShifterRaw
{
   // CAN
   uint8_t canSelector; // 0..3
   bool canTipUp;
   bool canTipDown;
   // Analog
   bool din1, din2, parkIn, notParkIn;
   int32_t anaMv; // ana_in at the connector (after undoing the /2), mV
};

struct ShifterConfig
{
   uint8_t type;      // ShifterType
   int32_t anaRMax;   // mV, below this R
   int32_t anaNMax;   // mV, below this N (else D)
   int32_t anaMin;    // mV, below this the input is faulty
   int32_t anaMax;    // mV, above this the input is faulty
};

class Shifter
{
public:
   Shifter();
   void Configure(const ShifterConfig &c) { cfg = c; }
   /// Every 10 ms.
   void Update(const ShifterRaw &raw);
   uint8_t Selector() const { return selector; }
   /// One-shot: true for one Update() after a tip edge.
   bool TipUp() const { return tipUp; }
   bool TipDown() const { return tipDown; }
   /// Analog inputs implausible (selector forced to N).
   bool Fault() const { return fault; }

private:
   uint8_t Decode(const ShifterRaw &raw, bool *bad) const;

   ShifterConfig cfg;
   uint8_t selector, candSel;
   bool candBad;
   uint8_t selCount;
   bool upRaw, downRaw, upStable, downStable;
   uint8_t upCount, downCount;
   bool tipUp, tipDown, fault;
};

#endif // SHIFTER_H
