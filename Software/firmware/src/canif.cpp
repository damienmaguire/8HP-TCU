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
#include "canif.h"
#include "linpump.h"

// ---------------------------------------------------------------------
// EcuType 0: None / VCU (placeholder frames, see README "CAN")

static int VcuRxIds(uint32_t cmdId, uint32_t ids[CANIF_MAX_RX_IDS]) {
  ids[0] = cmdId;
  return 1;
}

static bool VcuRx(uint32_t cmdId, uint32_t id, const uint8_t *b, uint8_t dlc,
                  CanRx *rx) {
  if (id != cmdId || dlc < 6)
    return false;
  /* Command frame (little endian):
   *  byte 0  selector 0=P 1=R 2=N 3=D
   *  byte 1  gear request 0 = hold, 1..8
   *  byte 2-3 actual motor torque, Nm, signed
   *  byte 4-5 motor speed, rpm, signed
   *  byte 6  (ShifterType = CAN) bit 0 tip up, bit 1 tip down, held
   *          while the lever is held. Optional: DLC 6 means no tips.
   */
  rx->selector = b[0] & 3;
  rx->gearReq = b[1];
  rx->torque = (int16_t)(b[2] | (b[3] << 8));
  rx->motorRpm = (int16_t)(b[4] | (b[5] << 8));
  rx->tips = dlc >= 7 ? (b[6] & 3) : 0;
  return true;
}

static int VcuTx(const CanTxIn &in, CanFrame out[CANIF_MAX_TX]) {
  const TcuOutputs &o = *in.o;
  /* Status frame CanStatId (little endian):
   *  byte 0  engaged gear (0 = N, 1..8, 9 = R)
   *  byte 1  mode (opmode)
   *  byte 2  target gear
   *  byte 3  fault
   *  byte 4-5 torque intervention request, Nm, signed; TqNoRedVal
   *          (default 0) when there is none
   *  byte 6  ATF temperature + 40 degC
   *  byte 7  bit 0 shifting, bit 1 park request, bit 2 safe state,
   *          bit 3 hold motor torque near 0 (gear-in), bit 4 CAN-loss
   *          gear hold, bit 5 coast shift
   */
  CanFrame &f = out[0];
  f.id = in.statId;
  f.dlc = 8;
  f.data[0] = o.gear;
  f.data[1] = o.mode;
  f.data[2] = o.target;
  f.data[3] = o.fault;
  int32_t tq = o.tqDelta != 0 ? o.tqDelta : in.noRedVal;
  f.data[4] = tq & 0xFF;
  f.data[5] = (tq >> 8) & 0xFF;
  int32_t t = in.atfTemp + 40;
  f.data[6] = t < 0 ? 0 : (t > 255 ? 255 : t);
  f.data[7] = (o.mode == TCU_SHIFT ? 1 : 0) | (o.parkRequest ? 2 : 0) |
              (o.safeOutputs ? 4 : 0) | (o.tqHold ? 8 : 0) |
              (o.canLossHold ? 16 : 0) | (o.coast ? 32 : 0);
  /* Second status frame CanStatId + 1:
   *  byte 0-1 minimum motor speed the TCU asks for, rpm (0 = none): park
   *          release (700 rpm, held 2 s) and PumpFaultAct = Motor. The
   *          VCU should spin the motor with the clutches open.
   *  byte 2  bit 0 waiting to leave P, bit 1 gear-in waiting for a
   *          pressure source, bit 2 Auto tip override running
   *  byte 3  LIN pump fault classes (lpClass)
   *  byte 4-5 channel 7 (K0 / TCC) pressure, mbar
   *  byte 6-7 0
   */
  CanFrame &g = out[1];
  g.id = in.statId + 1;
  g.dlc = 8;
  g.data[0] = o.nMotReq & 0xFF;
  g.data[1] = (o.nMotReq >> 8) & 0xFF;
  g.data[2] = (o.parkWait ? 1 : 0) | (o.gearInWait ? 2 : 0) | (o.tipAuto ? 4 : 0);
  g.data[3] = in.pumpClass;
  g.data[4] = o.pCh7 & 0xFF;
  g.data[5] = (o.pCh7 >> 8) & 0xFF;
  g.data[6] = 0;
  g.data[7] = 0;
  return 2;
}

// ---------------------------------------------------------------------
// Handler table: one row per EcuType.

struct EcuHandler
{
   const char *name;
   int (*rxIds)(uint32_t cmdId, uint32_t ids[CANIF_MAX_RX_IDS]);
   bool (*rx)(uint32_t cmdId, uint32_t id, const uint8_t *d, uint8_t dlc, CanRx *rx);
   int (*tx)(const CanTxIn &in, CanFrame out[CANIF_MAX_TX]);
};

static const EcuHandler handlers[ECU_COUNT] = {
    {"None/VCU", VcuRxIds, VcuRx, VcuTx},
    // OEM engine ECUs will be added here later.
};

static const EcuHandler &H(uint8_t t) {
  return handlers[t < ECU_COUNT ? t : (uint8_t)ECU_NONE_VCU];
}

namespace CanIf {

int RxIds(uint8_t t, uint32_t cmdId, uint32_t ids[CANIF_MAX_RX_IDS]) {
  return H(t).rxIds(cmdId, ids);
}

bool Rx(uint8_t t, uint32_t cmdId, uint32_t id, const uint8_t *d, uint8_t dlc,
        CanRx *rx) {
  return H(t).rx(cmdId, id, d, dlc, rx);
}

int Tx(uint8_t t, const CanTxIn &in, CanFrame out[CANIF_MAX_TX]) {
  return H(t).tx(in, out);
}

const char *Name(uint8_t t) { return H(t).name; }

uint8_t E2eCrc(const uint8_t d[8], uint16_t dataId) {
  uint8_t buf[9];
  buf[0] = dataId & 0xFF;
  buf[1] = dataId >> 8;
  for (int i = 1; i < 8; i++)
    buf[i + 1] = d[i];
  return Lin::Crc8(buf, 9); // same CRC-8 as the LIN pump: 0x1D, init 0
}

uint16_t TqRaw(int32_t nm) {
  // raw = (Nm + 1023.5) / 0.5 = 2 Nm + 2047
  int32_t raw = 2 * nm + 2047;
  if (raw < 0)
    raw = 0;
  if (raw > 0xFFE)
    raw = 0xFFE;
  return (uint16_t)raw;
}

void PackF11Torque(uint8_t qual, int32_t targetNm, uint8_t counter, uint8_t d[8]) {
  if (qual != TQ_QUAL_RAISE && qual != TQ_QUAL_REDUCE)
    qual = TQ_QUAL_NONE;
  uint16_t tgt = qual == TQ_QUAL_NONE ? CANIF_F11_TQ_NONE : TqRaw(targetNm);
  const uint16_t pred = CANIF_F11_TQ_NONE; // slow path: no value
  const uint16_t gbx = CANIF_F11_TQ_NONE;  // absorbing torque: no value
  const uint16_t sync = CANIF_F11_TQ_NONE; // sync speed: none (F11)
  d[1] = (uint8_t)((counter % 15) | ((pred & 0x0F) << 4));
  d[2] = (uint8_t)(pred >> 4);
  d[3] = (uint8_t)(tgt & 0xFF);
  d[4] = (uint8_t)((tgt >> 8) | ((gbx & 0x0F) << 4));
  d[5] = (uint8_t)(gbx >> 4);
  d[6] = (uint8_t)(qual | ((sync & 0x0F) << 4));
  d[7] = (uint8_t)(sync >> 4);
  d[0] = E2eCrc(d, CANIF_F11_TQ_DATAID);
}

void PackTqCeiling(int32_t maxNm, uint8_t counter, uint8_t d[8]) {
  uint16_t raw = TqRaw(maxNm);
  d[1] = (uint8_t)((counter % 15) | 0xF0);
  d[2] = 0xFF;
  d[3] = 0xFF;
  d[4] = (uint8_t)(raw & 0xFF);
  d[5] = (uint8_t)((raw >> 8) | 0xF0);
  d[6] = 0xFF;
  d[7] = 0xFF;
  d[0] = E2eCrc(d, CANIF_TQ_CEIL_DATAID);
}

uint8_t F11TqTarget(int32_t inputNm, int32_t deltaNm, int32_t *targetNm) {
  if (deltaNm == 0) {
    *targetNm = 0;
    return TQ_QUAL_NONE;
  }
  *targetNm = inputNm + deltaNm;
  return deltaNm < 0 ? TQ_QUAL_REDUCE : TQ_QUAL_RAISE;
}

} // namespace CanIf
