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
#ifndef CANIF_H
#define CANIF_H

#include "tcu.h"
#include <stdint.h>

/* CAN1 frame handling, kept apart from the hardware so it can be tested.
 *
 * Who the TCU talks to is the EcuType parameter. Each type is one entry in
 * the handler table in canif.cpp: which IDs to receive, how to decode
 * them into CanRx, and which frames to send. Today there is one type:
 *
 *   0 = None/VCU  the placeholder VCU frames (README "CAN"): one command
 *                 frame in (COMM_CMD_ID), status frames out (COMM_STAT_ID,
 *                 COMM_STAT_ID + 1).
 *
 * OEM engine ECUs (DME/DDE) will be added later as more entries; none of
 * their frames are in the firmware yet. To add one: a new EcuType value,
 * its RxIds / Rx / Tx functions, its row in the table, and its name in
 * ECUTYPES (param_prj.h).
 *
 * Nothing here sends by itself: the caller only transmits when CanTxEnable
 * is on (default off; no TX to the car without Damien's go-ahead).
 */

enum EcuType
{
   ECU_NONE_VCU = 0, // placeholder VCU protocol
   // OEM engine ECUs will be added here later.
   ECU_COUNT
};

/// What the TCU takes from the bus (whatever the ECU type).
struct CanRx
{
   uint8_t selector;  // SEL_P .. SEL_D
   uint8_t gearReq;   // 0 = hold, 1..8
   int16_t torque;    // Nm, actual motor torque
   int16_t motorRpm;  // rpm
   uint8_t tips;      // bit 0 tip up, bit 1 tip down (held)
   bool brake;        // footbrake pressed (VCU byte 6 bit 2; false if not sent)
};

/// What the TCU puts on the bus.
struct CanTxIn
{
   const TcuOutputs *o;
   int32_t atfTemp;    // degC
   uint8_t pumpClass;  // LinPumpClass bits
   uint32_t statId;    // COMM_STAT_ID
   int16_t noRedVal;   // COMM_TQ_NO_RED: sent as the intervention when there
                       // is none (0 by default; some ECUs want a code)
};

struct CanFrame
{
   uint32_t id;
   uint8_t dlc;
   uint8_t data[8];
};

#define CANIF_MAX_RX_IDS 4
#define CANIF_MAX_TX 4

namespace CanIf
{
/// IDs to register for an ECU type. Returns how many.
int RxIds(uint8_t ecuType, uint32_t cmdId, uint32_t ids[CANIF_MAX_RX_IDS]);
/// Decode a received frame. True if it was one of ours (rx updated).
bool Rx(uint8_t ecuType, uint32_t cmdId, uint32_t id, const uint8_t *d,
        uint8_t dlc, CanRx *rx);
/// Frames to send every 10 ms. Returns how many were filled in.
int Tx(uint8_t ecuType, const CanTxIn &in, CanFrame out[CANIF_MAX_TX]);
/// Name for the log / web UI.
const char *Name(uint8_t ecuType);

/* Optional torque ceiling frame 0x0A0 (RQ_TORQ_CRSH_GRB_2), PT-CAN, 10 ms,
 * DLC 8, Intel bit numbering, off by default (COMM_TQ_CEIL_TX).
 *   bits 0-7    BMW E2E CRC-8 (poly 0x1D, init 0, no final XOR) over the
 *               Data ID 0x00A0 (low, high) and bytes 1..7 [C]
 *   bits 8-11   alive counter 0..14 (15 skipped)
 *   bits 32-43  maximum allowed engine torque sv_m_EngTqMax, Nm = raw x
 *               0.5 - 1023.5 [C]
 *   every other bit 1.
 * (The 0x0B0 engine torque intervention frame was removed: the TCU is
 * generic and does not talk to an F11 engine ECU.)
 */
#define CANIF_TQ_CEIL_ID 0x0A0
#define CANIF_TQ_CEIL_DATAID 0x00A0

/// BMW E2E CRC-8 (poly 0x1D, init 0, no XOR) over Data ID + bytes 1..7.
uint8_t E2eCrc(const uint8_t d[8], uint16_t dataId);
/// Raw 12-bit value for a torque in Nm (0.5 Nm per bit, -1023.5 offset),
/// clamped to 0..0xFFE so it can never read as "no request".
uint16_t TqRaw(int32_t nm);
/// Build 0x0A0 with the torque ceiling in Nm.
void PackTqCeiling(int32_t maxNm, uint8_t counter, uint8_t d[8]);
} // namespace CanIf

#endif // CANIF_H
