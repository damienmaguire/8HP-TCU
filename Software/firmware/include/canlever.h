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
#ifndef CANLEVER_H
#define CANLEVER_H

/* CAN gear levers / selector panels, all behind one lever-state interface
 * (P / R / N / D / M plus one-shot tips) that feeds the normal selector
 * input of the TCU, so the gear-in rules, the interlock and the CAN-loss
 * rules apply unchanged.
 *
 * Ported from Damien Maguire's ZombieVerter VCU (Stm32-vcu, branch
 * BMW-F11-Dev, src/shifters/, GPL v3): RX decode, TX (LEDs /
 * display) and counters as there. Differences, all for the TCU's sake:
 *  - every change into R, D or P (and M) passes the engage gate (output
 *    speed at most the gear-in limit); a blocked request flashes the
 *    display where the lever has one;
 *  - every lever has an RX timeout; when it runs out the TCU uses the
 *    CAN loss rule (COMM_CANLOSS_MODE) (hold the gear or neutral);
 *  - VCU-only features are not ported (PKP2300 regen / heater toggles and
 *    the state-of-charge bar);
 *  - TX only with COMM_LEVER_TX and CanTxEnable (both off by default).
 *    0x202 (BMW backlight) is never sent.
 *
 * Types (ShifterType ids; 1 = Analog is not a CAN lever):
 *  0 BMW GWS F30 / F11 (gwslever.h), RX 0x197, TX 0x3FD
 *  2 BMW E65 lever, RX 0x192, no TX (ZV sends none)
 *  3 JLR rotary knob gen 1, RX 0x312, TX 0x3F3 every 20 ms
 *  4 JLR rotary knob gen 2, RX 0x0E0, TX 0x02C every 10 ms
 *  5 Blink Marine PKP-2300-SI CANopen keypad (node 0x15), RX TPDO1
 *    0x195, TX RPDO1 0x215 LEDs every 100 ms and NMT start 0x000 while
 *    the keypad is silent
 *  6 VCU frame (the COMM_CMD_ID selector, no lever)
 */

#include "canif.h"
#include "gwslever.h"
#include <stdint.h>

enum LeverType
{
   LEVER_BMW_GWS = 0,
   LEVER_ANALOG = 1, // not a CAN lever, see shifter.h
   LEVER_BMW_E65 = 2,
   LEVER_JLR_G1 = 3,
   LEVER_JLR_G2 = 4,
   LEVER_PKP2300 = 5,
   LEVER_VCU = 6,
   LEVER_COUNT
};

#define LEVER_MAX_TX 2

struct LeverConfig
{
   uint8_t type;        // LeverType
   uint16_t timeoutMs;  // 0 = the lever's own default
   uint8_t debounce;    // equal frames before a move / position is taken
   uint8_t mSub;        // GWS 0x3FD sub-field in M [H]
   uint8_t b34;         // GWS 0x3FD B3 / B4 style (GwsB34)
};

/// What the TX side needs from the TCU.
struct LeverTxIn
{
   bool run;            // KL15 on / TCU running
   uint8_t selector;    // the selector the TCU is acting on (SEL_*)
   uint8_t gear;        // current gear 0..8 (GWS ZV B3/B4 style)
};

class CanLever
{
public:
   CanLever();
   void Configure(const LeverConfig &c);
   bool IsCan() const { return cfg.type != LEVER_ANALOG && cfg.type != LEVER_VCU; }
   /// RX ids to register for this lever. Returns the count (<= 2).
   int RxIds(uint32_t ids[2]) const;
   /// One received frame. Returns true if it was this lever's and valid.
   bool OnFrame(uint32_t id, const uint8_t *d, uint8_t dlc);
   /// Every 10 ms. engageOk: a change into R / D / P / M may be taken.
   void Tick10ms(bool engageOk);
   /// Every 10 ms: frames to send now (the lever keeps its own rate).
   int Tx10ms(const LeverTxIn &in, CanFrame out[LEVER_MAX_TX]);

   bool Seen() const { return seen; }
   bool Valid() const;
   uint8_t State() const;      // GwsState
   uint8_t Selector() const;   // SEL_*
   bool TipUp() const;
   bool TipDown() const;
   bool Flash() const;
   uint8_t RawMove() const;    // lever-specific raw code (spot value)
   bool PButton() const;
   uint16_t CrcErrors() const;
   uint16_t CounterErrors() const;
   uint16_t TimeoutMs() const;
   /// JLR G2 knob lock byte (0x40 once raised), for the tests.
   uint8_t KnobLock() const { return knobLock; }

private:
   void Request(uint8_t want, bool engageOk);

   LeverConfig cfg;
   GwsLever gws;
   GwsDisplay gwsDisp;
   // Generic state for the other levers.
   bool seen;
   uint16_t msSinceRx;
   uint8_t state, rawCode, lastCode, pendCode, pendCount;
   uint16_t flashMs;
   bool flashOnce;      // PKP: one red LED cycle for a rejected key
   uint8_t rejected;    // PKP: the rejected state
   uint8_t prevKeys;
   uint16_t cntErr;
   uint8_t txDiv, txCnt, offCount, knobLock;
};

#endif // CANLEVER_H
