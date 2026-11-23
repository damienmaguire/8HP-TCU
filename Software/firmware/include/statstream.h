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
#ifndef STATSTREAM_H
#define STATSTREAM_H

/* Optional status stream for dashboards / loggers (StreamEnable, off by
 * default). Our own layout, little-endian, 4 frames from StreamBaseId:
 *
 *  base+0  every 10 ms (100 Hz)  torque
 *    0-1  torque target, Nm, signed (input torque + intervention)
 *    2    torque reduction, % of the input torque (0..100)
 *    3    flags: bit 0 shift active, 1 coast shift, 2 intervention
 *         active, 3 inertia phase, 4 TCC closed
 *    4-5  slip of the current gear, rpm, signed
 *    6    alive counter 0..255
 *    7    0
 *  base+1  every 10 ms (100 Hz)  shaft speeds, rpm, unsigned
 *    0-1  input (turbine) n_t   2-3 output n_ab
 *    4-5  engine / motor n_mot  6-7 n6 (aux, 0 where not fitted)
 *  base+2  every 25 ms (40 Hz; 20 / 30 ms alternating)  gear / state
 *    0 engaged gear (0 N, 1..8, 9 R)  1 target gear  2 selector (0 P,
 *    1 R, 2 N, 3 D)  3 TCU mode  4 lever state  5 drive mode
 *    6 shift phase on-coming (OEM state no.)  7 off-going
 *  base+3  every 50 ms (20 Hz)  status
 *    0 TCU fault  1-2 diagnostic bits  3 ATF temperature + 40 degC
 *    4 board temperature + 40 degC  5 battery, 0.1 V
 *    6 adaptation status (low byte)  7 bit 0 CAN loss, bit 1 lever
 *    timeout, bit 2 torque not from CAN
 */

#include "canif.h"
#include <stdint.h>

struct StreamIn
{
   int32_t tqTarget, tqReductPct;
   bool shifting, coast, tqActive, inertia, tccClosed;
   int32_t slip;
   int32_t nT, nAb, nMot, nAux;
   uint8_t gear, target, selector, mode, leverState, driveMode, zKzu, zKab;
   uint8_t fault;
   uint16_t diagBits;
   int32_t atfC, boardC, vbatMv;
   uint8_t adaStatus;
   bool canLoss, leverTimeout, tqNotCan;
};

#define STREAM_MAX_TX 4

class StatStream
{
public:
   StatStream() : ms(0), next40(0), counter(0) {}
   /// Every 10 ms: frames due now.
   int Tick10ms(uint32_t baseId, const StreamIn &in, CanFrame out[STREAM_MAX_TX]);
   /// Torque reduction in % of the input torque (0 when not reducing).
   static int32_t ReductPct(int32_t inputNm, int32_t deltaNm);

private:
   uint32_t ms, next40;
   uint8_t counter;
};

#endif // STATSTREAM_H
