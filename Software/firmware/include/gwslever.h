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
#ifndef GWSLEVER_H
#define GWSLEVER_H

#include <stdint.h>

/* BMW GWS gear lever (F-series monostable lever), the EGS side of the
 * GWS <-> EGS link. Spec: flexray-re-notes egs_gws_lever_can.md / .csv;
 * logic after Damien's Stm32-vcu F30_Lever.cpp, with the F11 fixes.
 *
 * RX 0x197 ST_GWS (lever -> EGS), DLC 4, 30 ms, timeout 90 ms [C]:
 *   B0      CRC-8 poly 0x1D init 0 over B1..B3, XOR 0x53 (= BMW E2E
 *           over Data ID 0x003F lo/hi + B1..B3) [C]
 *   B1 0-3  alive counter 0..14 (15 invalid) [C]
 *   B1 4-7  absolute lever zone (not used) [H]
 *   B2 0-3  S button / drive logic: F11 0xF, F30 0xE, masked off
 *   B2 4-7  relative move: 0 centre, 1 Up1, 2 Up2, 3 Down1, 4 Down2 [C];
 *           5 M+, 6 M-, 7 side (M) gate [I, not seen on F11]
 *   B3 0-1  P button (1 = pressed), B3 2-3 / 4-5 redundant copies; all
 *           three must agree [C]
 * Alive counter rules as the LIN pump: the first frame is taken, one
 * repeat is tolerated, a second repeat in a row or a jump of more than 2
 * (mod 15) rejects the frame.
 *
 * The lever only reports moves; the TCU keeps the absolute position
 * (P/R/N/D/M) itself, as the OEM D_Tab_RelToAbsConversion does. A move is
 * taken once it is seen in `debounce` frames in a row and only once per
 * code (returning through Up1 / Down1 to the centre does nothing). Rules,
 * matched against the F11 capture r10md10m (P -Up1-> N -Up2-> R
 * -Down2-> D, P button -> P):
 *   Up1    P -> N, D -> N, M -> N
 *   Up2    P / N / D / M -> R          (engage gate)
 *   Down1  P -> N, R -> N
 *   Down2  P / N / R -> D              (engage gate)
 *   gate   D -> M (held while code 5/6/7), back to the centre -> D
 *   M+ / M-  one tip up / down in M
 *   P button  any -> P                 (engage gate)
 * The engage gate (output speed at most the gear-in limit) blocks a
 * request; the display flashes for 1 s after a blocked one [I].
 */

#define GWS_RX_ID 0x197
#define GWS_TX_ID 0x3FD
#define GWS_RX_DATAID 0x003F
#define GWS_TX_DATAID 0x0039
#define GWS_RX_XOR 0x53
#define GWS_TX_XOR 0x70

enum GwsMove
{
   GWS_CENTRE = 0,
   GWS_UP1 = 1,
   GWS_UP2 = 2,
   GWS_DOWN1 = 3,
   GWS_DOWN2 = 4,
   GWS_MPLUS = 5,
   GWS_MMINUS = 6,
   GWS_GATE = 7,
};

enum GwsState
{
   GWS_P = 0,
   GWS_R = 1,
   GWS_N = 2,
   GWS_D = 3,
   GWS_M = 4,
};

/// 0x3FD B3 / B4 style.
enum GwsB34
{
   GWS_B34_OEM = 0, // FC FF, as the F11 EGS sends [C]
   GWS_B34_ZV = 1,  // 00, gear number (ZombieVerter F30_Lever)
};

struct GwsConfig
{
   uint16_t timeoutMs;   // 90 [C]
   uint8_t debounce;     // equal move frames needed, 2
};

class GwsLever
{
public:
   GwsLever();
   void Configure(const GwsConfig &c) { cfg = c; }
   /// A received 0x197. Returns true if it passed DLC, CRC and counter.
   bool OnFrame(const uint8_t *d, uint8_t dlc);
   /** Every 10 ms. engageOk: a new R / D / P may be taken (output speed
    * at or below the gear-in limit and no fault). */
   void Tick10ms(bool engageOk);
   bool Valid() const { return seen && msSinceRx <= cfg.timeoutMs; }
   uint8_t State() const { return state; }
   /// Selector for the TCU (SEL_P / R / N / D; M = D).
   uint8_t Selector() const;
   bool TipUp() const { return tipUp; }
   bool TipDown() const { return tipDown; }
   bool Flash() const { return flashMs > 0; }
   uint8_t RawMove() const { return rawMove; }
   bool PButton() const { return pRaw; }
   uint16_t CrcErrors() const { return crcErr; }
   uint16_t CounterErrors() const { return cntErr; }
   uint16_t DlcErrors() const { return dlcErr; }

   /// CRC byte of a 0x197 (B1..B3).
   static uint8_t RxCrc(const uint8_t *d);
   /// CRC byte of a 0x3FD (B1..B4).
   static uint8_t TxCrc(const uint8_t *d);

private:
   void Take(uint8_t move, bool engageOk);

   GwsConfig cfg;
   bool seen;
   uint16_t msSinceRx;
   int8_t lastAlive;
   uint8_t stuck;
   uint8_t rawMove, pendMove, pendCount, takenMove;
   bool pRaw, pStable, pEdge;
   uint8_t pCount;
   uint8_t state;
   bool tipUp, tipDown;
   uint16_t flashMs;
   uint16_t crcErr, cntErr, dlcErr;
};

/* TX 0x3FD DT_DISP_GRDT (EGS -> lever LEDs / KOMBI), DLC 5, 100 ms [C]:
 *   B0  CRC-8 over B1..B4 XOR 0x70 (= E2E Data ID 0x0039) [C]
 *   B1  alive counter 0..14 in the low nibble, high nibble 0 [C]
 *   B2  position bits 5-7: 0x20 P, 0x40 R, 0x60 N, 0x80 D, 0x00 off [C];
 *       0x08 flash [I]; bits 0-2 sub-field, `mSub` in M [H]
 *   B3 / B4  OEM FC FF [C] or ZombieVerter 00, gear number
 * 0x00 (off) before KL15 has been seen and from 30 s after KL15 off, as
 * the F11 EGS does [C]. 0x202 (backlight) is NOT sent: it comes from the
 * body side, not the EGS. */
class GwsDisplay
{
public:
   GwsDisplay() : t15Seen(false), offMs(0), counter(0) {}
   /** Every 100 ms: build the next frame. state = GwsState, gear 0..8
    * (only for the ZV B3/B4 style). */
   void Tick100ms(bool t15, uint8_t state, bool flash, uint8_t mSub,
                  uint8_t b34, uint8_t gear, uint8_t d[5]);
   static void Pack(uint8_t pos, uint8_t counter, uint8_t b3, uint8_t b4,
                    uint8_t d[5]);

private:
   bool t15Seen;
   uint32_t offMs;
   uint8_t counter;
};

#endif // GWSLEVER_H
