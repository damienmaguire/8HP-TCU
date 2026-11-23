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
#include "test.h"

void TestCanIf() {
  // EcuType 0 = None/VCU: the placeholder command frame
  uint32_t ids[CANIF_MAX_RX_IDS];
  CHECK_EQ(CanIf::RxIds(ECU_NONE_VCU, 768, ids), 1);
  CHECK_EQ(ids[0], 768u);
  CHECK_EQ(CanIf::RxIds(77, 768, ids), 1); // unknown type -> None/VCU
  CanRx rx = {};
  const uint8_t f[7] = {3, 4, 0x2C, 0x01, 0xE8, 0x03, 2}; // D, 4th, 300 Nm, 1000 rpm, tip down
  CHECK(CanIf::Rx(ECU_NONE_VCU, 768, 768, f, 7, &rx));
  CHECK_EQ(rx.selector, 3);
  CHECK_EQ(rx.gearReq, 4);
  CHECK_EQ(rx.torque, 300);
  CHECK_EQ(rx.motorRpm, 1000);
  CHECK_EQ(rx.tips, 2);
  CHECK(CanIf::Rx(ECU_NONE_VCU, 768, 768, f, 6, &rx));
  CHECK_EQ(rx.tips, 0); // DLC 6: no tips
  CHECK(!CanIf::Rx(ECU_NONE_VCU, 768, 769, f, 7, &rx));
  CHECK(!CanIf::Rx(ECU_NONE_VCU, 768, 768, f, 5, &rx));

  TcuOutputs o = {};
  o.gear = 3;
  o.mode = TCU_DRIVE;
  o.target = 3;
  o.tqDelta = -50;
  o.nMotReq = 700;
  o.parkWait = true;
  o.pCh7 = 6000;
  CanTxIn in = {&o, 60, LPC_COMM, 769};
  CanFrame out[CANIF_MAX_TX];
  CHECK_EQ(CanIf::Tx(ECU_NONE_VCU, in, out), 2);
  CHECK_EQ(out[0].id, 769u);
  CHECK_EQ(out[0].data[0], 3);
  CHECK_EQ(out[0].data[4], 0xCE);
  CHECK_EQ(out[0].data[5], 0xFF);
  CHECK_EQ(out[0].data[6], 100);
  CHECK_EQ(out[1].id, 770u);
  CHECK_EQ(out[1].data[0] | (out[1].data[1] << 8), 700);
  CHECK_EQ(out[1].data[2], 1);
  CHECK_EQ(out[1].data[3], LPC_COMM);
  CHECK_EQ(out[1].data[4] | (out[1].data[5] << 8), 6000);
  CHECK(CanIf::Name(ECU_NONE_VCU)[0] == 'N');

  // F11 0x0B0 [I], F11_EGS_CAN_LOGCHECK.md section A.
  uint8_t d[8];
  // Idle: no request. Logged idle B2..B7 = FF FF FF 7F F0 FF.
  CanIf::PackF11Torque(CanIf::TQ_QUAL_NONE, 123, 3, d);
  CHECK_EQ(d[1], 0xF3); // counter 3 in the low nibble, aux 0xFFF
  const uint8_t idle[6] = {0xFF, 0xFF, 0xFF, 0x7F, 0xF0, 0xFF};
  for (int i = 0; i < 6; i++)
    CHECK_EQ(d[2 + i], idle[i]);
  // CRC: poly 0x1D init 0 over Data ID 0x0001 + B1..B7, equal to the
  // "CRC0(B1..B7) XOR 0x65" form from the log check.
  CHECK_EQ(d[0], CanIf::E2eCrc(d, CANIF_F11_TQ_DATAID));
  CHECK_EQ(d[0], (uint8_t)(Lin::Crc8(&d[1], 7) ^ 0x65));
  uint8_t z[9] = {0x01, 0x00, 1, 2, 3, 4, 5, 6, 7};
  uint8_t zz[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  CHECK_EQ(CanIf::E2eCrc(zz, 0x0001), Lin::Crc8(z, 9));
  CHECK_EQ(CanIf::E2eCrc(zz, 0x0001), (uint8_t)(Lin::Crc8(&zz[1], 7) ^ 0x65));
  // Scale raw x 0.5 - 1023.5: 0 Nm = 2047, 100 Nm = 2247, -52 Nm = 1943.
  CHECK_EQ(CanIf::TqRaw(0), 2047);
  CHECK_EQ(CanIf::TqRaw(100), 2247);
  CHECK_EQ(CanIf::TqRaw(-52), 1943);
  CHECK_EQ(CanIf::TqRaw(5000), 0xFFE); // never reads as "no request"
  CHECK_EQ(CanIf::TqRaw(-5000), 0);
  // Reduce to 150 Nm: target bits 24-35, aux 0xFFE, qualifier 2.
  CanIf::PackF11Torque(CanIf::TQ_QUAL_REDUCE, 150, 7, d);
  CHECK_EQ(d[1] & 0x0F, 7);
  CHECK_EQ(((d[1] >> 4) | (d[2] << 4)), 0xFFE);
  CHECK_EQ((d[3] | ((d[4] & 0x0F) << 8)), CanIf::TqRaw(150));
  CHECK_EQ(d[6] & 0x0F, 2);
  CHECK_EQ(d[6] & 0xF0, 0xF0);
  CHECK_EQ(d[7], 0xFF);
  CHECK_EQ(d[0], CanIf::E2eCrc(d, CANIF_F11_TQ_DATAID));
  // Raise to -10 Nm: aux = target, qualifier 1.
  CanIf::PackF11Torque(CanIf::TQ_QUAL_RAISE, -10, 0, d);
  CHECK_EQ(((d[1] >> 4) | (d[2] << 4)), CanIf::TqRaw(-10));
  CHECK_EQ((d[3] | ((d[4] & 0x0F) << 8)), CanIf::TqRaw(-10));
  CHECK_EQ(d[6] & 0x0F, 1);
  // Unknown qualifier (3) is sent as none.
  CanIf::PackF11Torque(3, 100, 0, d);
  CHECK_EQ(d[6] & 0x0F, 0);
  CHECK_EQ((d[3] | ((d[4] & 0x0F) << 8)), 0xFFF);
  // Counter 0..14, 15 skipped.
  CanIf::PackF11Torque(CanIf::TQ_QUAL_NONE, 0, 15, d);
  CHECK_EQ(d[1] & 0x0F, 0);
  // Absolute target from the intervention delta.
  int32_t tgt = 1;
  CHECK_EQ(CanIf::F11TqTarget(200, 0, &tgt), CanIf::TQ_QUAL_NONE);
  CHECK_EQ(tgt, 0);
  CHECK_EQ(CanIf::F11TqTarget(200, -80, &tgt), CanIf::TQ_QUAL_REDUCE);
  CHECK_EQ(tgt, 120);
  CHECK_EQ(CanIf::F11TqTarget(-50, 40, &tgt), CanIf::TQ_QUAL_RAISE);
  CHECK_EQ(tgt, -10);
}
