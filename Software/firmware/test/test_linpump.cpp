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
#include "linpump.h"
#include "test.h"

/// Pump response for id: data + LIN checksum.
static void Resp(uint8_t id, const uint8_t d[8], uint8_t out[9], bool enh = true) {
  for (int i = 0; i < 8; i++)
    out[i] = d[i];
  out[8] = Lin::Checksum(enh, Lin::Pid(id), d, 8);
}

/// A valid 0x32: op point op, alive cnt.
static void Status32(uint8_t op, uint8_t cnt, uint8_t out[9], uint8_t eb0 = 0,
                     uint8_t eb1 = 0) {
  uint8_t d[8] = {eb0, eb1, op, 0x40, 25, 135, 0, (uint8_t)(cnt & 0x0F)};
  d[6] = Lin::Crc8(&d[2], 1);
  Resp(LINPUMP_ID_STATUS, d, out);
}

struct Bus {
  int n30, n32, n33, n34;
  uint8_t cmd[8];
  uint8_t alive32, alive33, alive34;
  bool answer;
  uint8_t eb0;
};

/// Run the master for ms; the fake pump answers every poll with valid
/// frames (if b.answer).
static void Run(LinPump &p, Bus &b, int ms, bool active, bool run, int32_t oil) {
  for (int t = 0; t < ms; t++) {
    LinSlot s;
    if (!p.Tick1ms(active, run, oil, &s))
      continue;
    if (s.publish) {
      CHECK_EQ(s.id, LINPUMP_ID_CMD);
      for (int i = 0; i < 8; i++)
        b.cmd[i] = s.data[i];
      b.n30++;
      continue;
    }
    uint8_t r[9];
    if (s.id == LINPUMP_ID_STATUS) {
      b.n32++;
      if (b.answer) {
        Status32(run ? 0x55 : 0xAA, ++b.alive32, r, b.eb0);
        CHECK(p.OnResponse(s.id, r, 9));
      }
    } else if (s.id == LINPUMP_ID_AUX1) {
      b.n33++;
      uint8_t d[8] = {0, 0, 0x50, 0x12, 0x34, 0x56, 0x78, (uint8_t)(++b.alive33 & 15)};
      Resp(s.id, d, r);
      if (b.answer)
        CHECK(p.OnResponse(s.id, r, 9));
    } else {
      b.n34++;
      uint8_t d[8] = {0, 0, 0x21, 0x27, 0x05, 0, 0, (uint8_t)(++b.alive34 & 15)};
      Resp(s.id, d, r);
      if (b.answer)
        CHECK(p.OnResponse(s.id, r, 9));
    }
  }
}

void TestLinPump() {
  // Protected IDs on the wire: F0 / 32 / 73 / B4 (Ian, section 3)
  CHECK_EQ(Lin::Pid(0x30), 0xF0);
  CHECK_EQ(Lin::Pid(0x32), 0x32);
  CHECK_EQ(Lin::Pid(0x33), 0x73);
  CHECK_EQ(Lin::Pid(0x34), 0xB4);
  CHECK_EQ(Lin::Pid(0x3C), 0x3C);
  CHECK_EQ(Lin::Pid(0x3D), 0x7D);

  // CRC-8, poly 0x1D, init 0, no final XOR: AA -> 73, 55 -> B7, CC -> A9,
  // 33 -> 6D (Ian's table)
  const uint8_t aa = 0xAA, x55 = 0x55, cc = 0xCC, x33 = 0x33;
  CHECK_EQ(Lin::Crc8(&aa, 1), 0x73);
  CHECK_EQ(Lin::Crc8(&x55, 1), 0xB7);
  CHECK_EQ(Lin::Crc8(&cc, 1), 0xA9);
  CHECK_EQ(Lin::Crc8(&x33, 1), 0x6D);
  const uint8_t zero = 0;
  CHECK_EQ(Lin::Crc8(&zero, 1), 0x00);

  // LIN checksums (classic / enhanced)
  const uint8_t f[8] = {0xAA, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x73, 0x00};
  CHECK_EQ(Lin::ChecksumClassic(f, 8), 0xD7);
  CHECK_EQ(Lin::ChecksumEnhanced(0xF0, f, 8), 0xE6);
  CHECK_EQ(Lin::Checksum(false, 0xF0, f, 8), 0xD7);
  const uint8_t ff[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  CHECK_EQ(Lin::ChecksumClassic(ff, 8), 0x00);

  // Read-back parsing
  const uint8_t bus[] = {0x00, 0x55, 0x32, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  CHECK_EQ(Lin::FindData(bus, sizeof(bus), 0x32), 3);
  CHECK_EQ(Lin::FindData(bus, sizeof(bus), 0xF0), -1);

  // Inactive: bus silent
  {
    LinPump p;
    Bus b = {};
    Run(p, b, 200, false, true, 40);
    CHECK_EQ(b.n30 + b.n32 + b.n33 + b.n34, 0);
    CHECK(p.State() == LP_OFF);
    CHECK(!p.TransceiverAwake());
  }

  // Schedule 30,32,30,33,30,34 on 10 ms slots: 0x30 every 20 ms, each
  // status frame every 60 ms
  {
    LinPump p;
    Bus b = {};
    b.answer = true;
    int seq[12], k = 0;
    for (int t = 0; t < 120; t++) {
      LinSlot s;
      if (p.Tick1ms(true, false, 40, &s)) {
        CHECK_EQ(t % 10, 0);
        if (k < 12)
          seq[k++] = s.id;
      }
    }
    CHECK_EQ(k, 12);
    const int want[6] = {0x30, 0x32, 0x30, 0x33, 0x30, 0x34};
    for (int i = 0; i < 12; i++)
      CHECK_EQ(seq[i], want[i % 6]);
    Run(p, b, 600, true, false, 40);
    CHECK_EQ(b.n30, 30);
    CHECK_EQ(b.n32, 10);
    CHECK_EQ(b.n33, 10);
    CHECK_EQ(b.n34, 10);
  }
  // Slot time is a parameter
  {
    LinPump p;
    LinPumpConfig c = {true, 15, 1800, 400};
    p.Configure(c);
    Bus b = {};
    Run(p, b, 900, true, false, 40);
    CHECK_EQ(b.n30, 30); // 0x30 every 30 ms
  }

  // 0x30 layout: off then run, oil temperature, CRC, alive counter
  {
    LinPump p;
    Bus b = {};
    b.answer = true;
    Run(p, b, 1, true, false, -40);
    // First frame after the bus comes up: clear-errors flag, op AA
    CHECK_EQ(b.cmd[0], 0xAA);
    CHECK_EQ(b.cmd[1], LINPUMP_FLAG_CLEAR);
    CHECK_EQ(b.cmd[2], 0x0A); // -40 degC + 50
    CHECK_EQ(b.cmd[3], 0);
    CHECK_EQ(b.cmd[4], 0);
    CHECK_EQ(b.cmd[5], 0);
    CHECK_EQ(b.cmd[6], 0x73);
    CHECK_EQ(b.cmd[7], 0);
    // Next 0x30: the clear flag is gone (exactly one frame), counter +1
    Run(p, b, 20, true, false, 16);
    CHECK_EQ(b.n30, 2);
    CHECK_EQ(b.cmd[1], 0);
    CHECK_EQ(b.cmd[2], 0x42); // 16 degC
    CHECK_EQ(b.cmd[7], 1);
    // Run: 55 with CRC B7
    Run(p, b, 20, true, true, 16);
    CHECK_EQ(b.cmd[0], 0x55);
    CHECK_EQ(b.cmd[6], 0xB7);
    CHECK(p.State() == LP_RUN);
    // Alive wraps 0..15, high nibble 0
    Run(p, b, 20 * 13, true, true, 16); // counter 2 -> 15
    CHECK_EQ(b.cmd[7], 0x0F);
    Run(p, b, 20, true, true, 16);
    CHECK_EQ(b.cmd[7], 0x00);
    // Temperature saturates
    Run(p, b, 20, true, true, 300);
    CHECK_EQ(b.cmd[2], 0xFF);
    Run(p, b, 20, true, true, -100);
    CHECK_EQ(b.cmd[2], 0x00);
    // Clear on request: one frame with 0x02
    p.ClearErrors();
    Run(p, b, 20, true, true, 40);
    CHECK_EQ(b.cmd[1], LINPUMP_FLAG_CLEAR);
    Run(p, b, 20, true, true, 40);
    CHECK_EQ(b.cmd[1], 0);
    // Limp / warm-up flags pass through, clear is not sticky
    p.SetFlags(LINPUMP_FLAG_LIMP | 0x80);
    Run(p, b, 20, true, true, 40);
    CHECK_EQ(b.cmd[1], LINPUMP_FLAG_LIMP);
  }

  // Decode 0x32 / 0x33 / 0x34
  {
    LinPump p;
    Bus b = {};
    b.answer = true;
    Run(p, b, 700, true, true, 40);
    const LinPumpStatus &s = p.Status();
    CHECK_EQ(s.opPoint, 0x55);
    CHECK_EQ(s.rpm, 0x40 * 32);
    CHECK_EQ(s.currentmA, 25 * 200);   // 0.2 A/bit
    CHECK_EQ(s.supplymV, 13500);       // 0.1 V/bit
    CHECK_EQ(s.tempRaw, 0x50);
    CHECK_EQ(s.pumpId, 0x12345678u);
    CHECK_EQ(s.hwVersion, 0x21);
    CHECK_EQ(s.swVersion, 0x2705);
    CHECK_EQ(p.Classes(), 0); // IEP85 SW variant, no errors
    CHECK_EQ(p.StatusWord(), 0u);
  }
  // Voltage substitute 14.0 V before any valid 0x32
  {
    LinPump p;
    CHECK_EQ(p.Status().supplymV, 14000);
  }

  // Receive checks
  {
    LinPump p;
    Bus b = {};
    b.answer = true;
    Run(p, b, 500, true, true, 40); // past the 400 ms post-clear block
    uint8_t r[9];
    uint8_t cnt = b.alive32;
    // Wrong LIN checksum: dropped
    Status32(0x55, ++cnt, r);
    r[8] ^= 0xFF;
    CHECK(!p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    CHECK_EQ(p.LinChecksumErrors(), 1);
    // One bad CRC-8: dropped, no fault yet
    uint8_t d[8] = {0, 0, 0x55, 0, 0, 0, 0x00, (uint8_t)(cnt & 15)};
    Resp(LINPUMP_ID_STATUS, d, r);
    CHECK(!p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    CHECK(!(p.StatusWord() & LINPUMP_ST_CRC8));
    // A good one in between resets the count
    Status32(0x55, cnt, r);
    CHECK(p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    CHECK(!p.OnResponse(LINPUMP_ID_STATUS, (Resp(LINPUMP_ID_STATUS, d, r), r), 9));
    CHECK(!(p.StatusWord() & LINPUMP_ST_CRC8));
    // Two in a row: CRC fault (permanent class)
    CHECK(!p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    CHECK(p.StatusWord() & LINPUMP_ST_CRC8);
    CHECK(p.Classes() & LPC_PERMANENT);
    CHECK_EQ(p.CrcErrors(), 3);
  }
  {
    LinPump p;
    Bus b = {};
    b.answer = true;
    Run(p, b, 500, true, true, 40);
    uint8_t r[9];
    uint8_t cnt = b.alive32;
    // One repeat is accepted, the second repeat in a row is a fault
    Status32(0x55, cnt, r);
    CHECK(p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    CHECK(!(p.StatusWord() & LINPUMP_ST_ALIVE));
    CHECK(!p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    CHECK(p.StatusWord() & LINPUMP_ST_ALIVE);
    CHECK(p.Classes() & LPC_COMM);
    // Clear resets the TCU-side bits; diagnosis blocked for 400 ms
    p.ClearErrors();
    Run(p, b, 30, true, true, 40);
    CHECK(!(p.StatusWord() & LINPUMP_ST_ALIVE));
    CHECK_EQ(p.Classes(), 0);
    // Jump by 2 is fine, by 3 is a fault
    cnt = b.alive32;
    Status32(0x55, cnt + 2, r);
    CHECK(p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    Status32(0x55, cnt + 5, r);
    CHECK(!p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    CHECK(p.StatusWord() & LINPUMP_ST_ALIVE);
    // Wrap 15 -> 0 is a step of 1
    p.ClearErrors();
    Status32(0x55, 15, r);
    CHECK(p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    Status32(0x55, 0, r);
    CHECK(p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    CHECK_EQ(p.AliveErrors(), 2);
  }

  // Error bytes and classes: EB0 0x40 = level-2 safety (permanent),
  // EB0 0x10 = pump comms, EB1 bit 0 = temporary
  {
    LinPump p;
    Bus b = {};
    b.answer = true;
    b.eb0 = 0x40;
    Run(p, b, 300, true, true, 40);
    CHECK_EQ(p.Classes(), 0); // inside the 400 ms after the start-up clear
    Run(p, b, 200, true, true, 40);
    CHECK_EQ(p.Classes(), LPC_PERMANENT);
    CHECK_EQ(p.Status().eb0, 0x40);
    CHECK(p.State() == LP_RUN); // report only: keeps running
    b.eb0 = 0x10;
    Run(p, b, 60, true, true, 40);
    CHECK_EQ(p.Classes(), LPC_COMM);
    b.eb0 = 0x80;
    Run(p, b, 60, true, true, 40);
    CHECK(p.Classes() & LPC_PERMANENT);
    uint8_t r[9];
    Status32(0x55, ++b.alive32, r, 0, 0x01);
    CHECK(p.OnResponse(LINPUMP_ID_STATUS, r, 9));
    CHECK_EQ(p.Classes(), LPC_TEMPORARY);
  }
  // Wrong SW variant in 0x34 (not IEP85) -> permanent (version)
  {
    LinPump p;
    Bus b = {};
    b.answer = true;
    Run(p, b, 500, true, true, 40);
    uint8_t d[8] = {0, 0, 0x41, 0x47, 0x00, 0, 0, (uint8_t)(++b.alive34 & 15)};
    uint8_t r[9];
    Resp(LINPUMP_ID_AUX2, d, r);
    CHECK(p.OnResponse(LINPUMP_ID_AUX2, r, 9));
    CHECK(p.StatusWord() & LINPUMP_ST_SWVER);
    CHECK(p.Classes() & LPC_PERMANENT);
  }

  // Bus fault: no valid 0x32 for 1.8 s
  {
    LinPump p;
    Bus b = {};
    b.answer = false;
    Run(p, b, 1790, true, true, 40);
    CHECK(!(p.Classes() & LPC_BUS));
    Run(p, b, 20, true, true, 40);
    CHECK(p.Classes() & LPC_BUS);
    CHECK(p.Classes() & LPC_COMM);
    CHECK(p.State() == LP_RUN); // keeps sending: retry and report
    CHECK(b.n30 > 80);
    b.answer = true;
    p.ClearErrors();
    Run(p, b, 500, true, true, 40);
    CHECK_EQ(p.Classes(), 0);
  }

  // Run rule: n_t < 500 (+50 hysteresis), 0..125 degC, pSys < 20 bar,
  // 800 ms off delay
  {
    LinPumpRule r;
    LinRuleConfig c = {500, 50, 0, 125, 20000, 800};
    r.Configure(c);
    CHECK(!r.Update(LINREQ_OFF, 0, 40, 5000));
    CHECK(r.Update(LINREQ_FORCE, 3000, 200, 30000));
    r.Update(LINREQ_OFF, 0, 40, 5000);
    CHECK(r.Update(LINREQ_AUTO, 100, 40, 5000));
    CHECK(r.Update(LINREQ_AUTO, 540, 40, 5000)); // inside the hysteresis
    // Above 550: off only after the 800 ms delay
    int ticks = 0;
    while (r.Update(LINREQ_AUTO, 600, 40, 5000) && ticks < 200)
      ticks++;
    CHECK_EQ(ticks, 79);
    CHECK(!r.Update(LINREQ_AUTO, 520, 40, 5000)); // above 500 from off
    CHECK(r.Update(LINREQ_AUTO, -300, 40, 5000)); // reverse
    LinPumpRule q;
    LinRuleConfig c0 = {500, 50, 0, 125, 20000, 0};
    q.Configure(c0);
    CHECK(!q.Update(LINREQ_AUTO, 0, -1, 5000));   // too cold
    CHECK(q.Update(LINREQ_AUTO, 0, 0, 5000));
    CHECK(q.Update(LINREQ_AUTO, 0, 125, 5000));
    CHECK(!q.Update(LINREQ_AUTO, 0, 126, 5000));  // too hot
    CHECK(q.Update(LINREQ_AUTO, 0, 40, 19999));
    CHECK(!q.Update(LINREQ_AUTO, 0, 40, 20000));  // pressure high enough
  }
}
