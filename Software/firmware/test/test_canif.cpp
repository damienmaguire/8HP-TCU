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

  // F11 0x0B0: E2E CRC with K = 0x65, counter 0..14, request field
  uint8_t d[8];
  CanIf::PackF11Torque(0, 3, d);
  CHECK_EQ(d[1] & 0x0F, 3);
  int raw = (d[1] >> 4) | (d[2] << 4);
  CHECK_EQ(raw, 0xFFE); // 0 Nm = 2 * 2047
  CanIf::PackF11Torque(-100, 3, d);
  raw = (d[1] >> 4) | (d[2] << 4);
  CHECK_EQ(raw, 2 * (2047 - 100));
  CHECK_EQ(d[0], CanIf::E2eCrc(d, 0x65));
  uint8_t z[8] = {0};
  CHECK_EQ(CanIf::E2eCrc(z, 0x65), 0x65); // CRC of zeros is 0, XOR K
  CanIf::PackF11Torque(-100, 15, d);
  CHECK_EQ(d[1] & 0x0F, 0); // counter wraps at 15
  CanIf::PackF11Torque(-5000, 1, d);
  CHECK_EQ((d[1] >> 4) | (d[2] << 4), 0);
}
