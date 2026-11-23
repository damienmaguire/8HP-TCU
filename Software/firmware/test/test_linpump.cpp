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

/// Build a pump response (8 data bytes + checksum) for id.
static void Resp(uint8_t id, const uint8_t d[8], uint8_t out[9], bool enh = true) {
  for (int i = 0; i < 8; i++)
    out[i] = d[i];
  out[8] = Lin::Checksum(enh, Lin::Pid(id), d, 8);
}

/// Run the pump master for ms milliseconds. Every 0x32 header is answered
/// with status st and echo byte echo (if answer is true). Returns the last
/// 0x30 command seen in cmd.
static int Run(LinPump &p, int ms, bool req, bool answer, uint8_t st,
               uint8_t echo, uint8_t cmd[8], int *n30 = 0, int *n32 = 0) {
  int published = 0, polled = 0;
  for (int t = 0; t < ms; t++) {
    LinSlot s;
    if (p.Tick1ms(req, &s)) {
      if (s.publish) {
        CHECK_EQ(s.id, LINPUMP_ID_CMD);
        for (int i = 0; i < 8; i++)
          cmd[i] = s.data[i];
        published++;
      } else {
        polled++;
        if (answer && s.id == LINPUMP_ID_STATUS) {
          uint8_t d[8] = {st, 0, 0, 0x40, 125, 0x11, echo, 0};
          uint8_t r[9];
          Resp(LINPUMP_ID_STATUS, d, r);
          CHECK(p.OnResponse(LINPUMP_ID_STATUS, r, 9));
        }
      }
    }
  }
  if (n30)
    *n30 = published;
  if (n32)
    *n32 = polled;
  return published;
}

void TestLinPump() {
  // Protected IDs (LIN 2.x parity P0 = ID0^ID1^ID2^ID4, P1 = !(ID1^ID3^ID4^ID5))
  CHECK_EQ(Lin::Pid(0x30), 0xF0);
  CHECK_EQ(Lin::Pid(0x32), 0x32);
  CHECK_EQ(Lin::Pid(0x33), 0x73);
  CHECK_EQ(Lin::Pid(0x34), 0xB4);
  CHECK_EQ(Lin::Pid(0x3C), 0x3C); // master request (diagnostics)
  CHECK_EQ(Lin::Pid(0x3D), 0x7D); // slave response
  CHECK_EQ(Lin::Pid(0x00), 0x80);
  CHECK_EQ(Lin::Pid(0x01), 0xC1);

  // Checksums. Classic: inverted sum with carry over the data only.
  const uint8_t cold[8] = {0xAA, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x73, 0x00};
  // 0xAA + 0x0A + 0x73 = 0x127 -> 0x28 after the carry -> ~ = 0xD7
  CHECK_EQ(Lin::ChecksumClassic(cold, 8), 0xD7);
  // Enhanced adds the PID 0xF0: 0xF0 + 0x28 = 0x118 -> 0x19 -> ~ = 0xE6
  CHECK_EQ(Lin::ChecksumEnhanced(0xF0, cold, 8), 0xE6);
  CHECK_EQ(Lin::Checksum(true, 0xF0, cold, 8), 0xE6);
  CHECK_EQ(Lin::Checksum(false, 0xF0, cold, 8), 0xD7);
  // Carry wraps more than once
  const uint8_t ff[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  CHECK_EQ(Lin::ChecksumClassic(ff, 8), 0x00);
  const uint8_t zero[8] = {0};
  CHECK_EQ(Lin::ChecksumClassic(zero, 8), 0xFF);
  // A frame plus its checksum sums to 0xFF
  {
    uint16_t sum = 0xF0;
    for (int i = 0; i < 8; i++) {
      sum += cold[i];
      if (sum > 0xFF)
        sum -= 0xFF;
    }
    sum += 0xE6;
    if (sum > 0xFF)
      sum -= 0xFF;
    CHECK_EQ(sum, 0xFF);
  }

  // Finding the data in the read-back buffer: break 0x00, sync, PID, data
  const uint8_t bus[] = {0x00, 0x55, 0x32, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  CHECK_EQ(Lin::FindData(bus, sizeof(bus), 0x32), 3);
  CHECK_EQ(Lin::FindData(bus, sizeof(bus), 0xF0), -1);
  CHECK_EQ(Lin::FindData(bus, 2, 0x32), -1);

  // Demand: Auto runs at low input speed while pressure is needed
  CHECK(!LinPumpDemand(LINREQ_AUTO, false, true, 0, 500, 200, false)); // disabled
  CHECK(!LinPumpDemand(LINREQ_OFF, true, true, 0, 500, 200, false));
  CHECK(LinPumpDemand(LINREQ_FORCE, true, false, 3000, 500, 200, false));
  CHECK(LinPumpDemand(LINREQ_AUTO, true, true, 100, 500, 200, false));
  CHECK(!LinPumpDemand(LINREQ_AUTO, true, false, 100, 500, 200, false)); // P
  CHECK(!LinPumpDemand(LINREQ_AUTO, true, true, 600, 500, 200, false));
  CHECK(LinPumpDemand(LINREQ_AUTO, true, true, 600, 500, 200, true)); // hyst
  CHECK(!LinPumpDemand(LINREQ_AUTO, true, true, 700, 500, 200, true));
  CHECK(LinPumpDemand(LINREQ_AUTO, true, true, -100, 500, 200, false));

  // Start-up sequence, default timing (25 ms cycle, 900 ms cold start)
  LinPump p;
  uint8_t cmd[8] = {0};
  CHECK(p.State() == LP_OFF);
  CHECK(!p.TransceiverAwake());
  int n30 = 0, n32 = 0;
  Run(p, 100, false, false, 0, 0, cmd, &n30, &n32);
  CHECK_EQ(n30, 0); // not requested: bus silent
  CHECK(p.State() == LP_OFF);

  // Phase 1: AA 00 0A 00 00 00 73 cc, one frame every 25 ms, no polling
  Run(p, 500, true, true, 0, 0x5A, cmd, &n30, &n32);
  CHECK(p.State() == LP_COLD1);
  CHECK(p.TransceiverAwake());
  CHECK_EQ(n30, 20);
  CHECK_EQ(n32, 0);
  CHECK_EQ(cmd[0], 0xAA);
  CHECK_EQ(cmd[1], 0x00);
  CHECK_EQ(cmd[2], 0x0A);
  CHECK_EQ(cmd[3], 0x00);
  CHECK_EQ(cmd[4], 0x00);
  CHECK_EQ(cmd[5], 0x00);
  CHECK_EQ(cmd[6], 0x73);
  CHECK_EQ(cmd[7], 19 & 0x0F); // alive counter 0..15
  // Phase 2 after ~900 ms: byte 2 = 0x42
  Run(p, 410, true, false, 0, 0, cmd);
  CHECK(p.State() == LP_COLD2 || p.State() == LP_START);
  CHECK_EQ(cmd[0], 0xAA);
  CHECK_EQ(cmd[2], 0x42);
  // Run start: 55 00 42 00 00 00 73 until a 0x32 with status 00.
  {
    LinPump q;
    LinPumpConfig c = {true, 25, 12, 900, 25, 200, 500, false};
    q.Configure(c);
    Run(q, 950, true, false, 0, 0, cmd);
    CHECK(q.State() == LP_START);
    CHECK_EQ(cmd[0], 0x55);
    CHECK_EQ(cmd[2], 0x42);
    CHECK_EQ(cmd[6], 0x73);
    // The pump answers 00 with echo 0x5A -> run, echo copied to byte 6
    Run(q, 50, true, true, 0x00, 0x5A, cmd);
    CHECK(q.State() == LP_RUN);
    CHECK_EQ(cmd[0], 0x55);
    CHECK_EQ(cmd[6], 0x5A);
    // The echo follows the last 0x32 (live echo)
    Run(q, 50, true, true, 0x00, 0x3C, cmd);
    CHECK_EQ(cmd[6], 0x3C);
    // Decode: rpm = byte 3 x 32, voltage = byte 4 x 0.1 V
    CHECK_EQ(q.Status().rpm, 0x40 * 32);
    CHECK_EQ(q.Status().supplymV, 12500);
    CHECK_EQ(q.Status().status, 0);
    CHECK_EQ(q.Status().raw32[5], 0x11);
    // A 0x40 reply is counted as a communication fault; its echo byte is
    // still the "last 0x32" and is echoed (summary wording) [H]
    int before = q.CommFaults();
    Run(q, 25, true, true, 0x40, 0x77, cmd);
    Run(q, 25, true, true, 0x40, 0x77, cmd);
    CHECK(q.CommFaults() > before);
    CHECK_EQ(cmd[6], 0x77);
    CHECK(q.State() == LP_RUN);
    // Only 0x40 replies for longer than the timeout -> fault, bus silent,
    // then a fresh cold start after the retry pause
    Run(q, 220, true, true, 0x40, 0x77, cmd);
    CHECK(q.State() == LP_FAULT);
    CHECK(q.Timeouts() >= 1);
    int quiet = 0;
    Run(q, 400, true, true, 0, 0, cmd, &quiet);
    CHECK_EQ(quiet, 0);
    Run(q, 120, true, true, 0, 0, cmd);
    CHECK(q.State() == LP_COLD1);
    CHECK_EQ(cmd[0], 0xAA);
    CHECK_EQ(cmd[2], 0x0A);
    // A bad checksum is rejected and does not change the echo
    uint8_t d[8] = {0, 0, 0, 0, 0, 0, 0x99, 0};
    uint8_t r[9];
    Resp(LINPUMP_ID_STATUS, d, r);
    r[8] ^= 1;
    CHECK(!q.OnResponse(LINPUMP_ID_STATUS, r, 9));
    CHECK_EQ(q.ChecksumErrors(), 1);
    CHECK(q.Status().echo != 0x99);
    // Classic-checksum response is rejected while enhanced is configured
    Resp(LINPUMP_ID_STATUS, d, r, false);
    CHECK(!q.OnResponse(LINPUMP_ID_STATUS, r, 9));
    // Request dropped -> off, transceiver asleep
    Run(q, 1, false, false, 0, 0, cmd);
    CHECK(q.State() == LP_OFF);
    CHECK(!q.TransceiverAwake());
  }

  // No answer at all in the start phase -> fault after the timeout
  {
    LinPump q;
    Run(q, 900 + 25 + 210, true, false, 0, 0, cmd);
    CHECK(q.State() == LP_FAULT);
  }

  // Schedule: in run, one 0x30 and one 0x32 per 25 ms cycle; 0x32 header
  // LinRxDelay after the 0x30 header; aux frames only when enabled.
  {
    LinPump q;
    LinPumpConfig c = {true, 30, 12, 0, 0, 200, 500, true};
    q.Configure(c);
    int ids[3] = {0, 0, 0};
    int t30 = -1, t32 = -1;
    for (int t = 0; t < 3000; t++) {
      LinSlot s;
      if (q.Tick1ms(true, &s)) {
        if (s.id == LINPUMP_ID_CMD)
          t30 = t;
        if (s.id == LINPUMP_ID_STATUS) {
          t32 = t;
          uint8_t d[8] = {0, 0, 0, 10, 120, 0, 0x21, 0};
          uint8_t r[9];
          Resp(LINPUMP_ID_STATUS, d, r);
          q.OnResponse(LINPUMP_ID_STATUS, r, 9);
          if (t > 1000)
            CHECK_EQ(t32 - t30, 12);
        }
        if (t > 1000)
          ids[s.id == LINPUMP_ID_CMD ? 0 : (s.id == LINPUMP_ID_STATUS ? 1 : 2)]++;
        if (s.id == LINPUMP_ID_AUX1 || s.id == LINPUMP_ID_AUX2) {
          uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
          uint8_t r[9];
          Resp(s.id, d, r);
          CHECK(q.OnResponse(s.id, r, 9));
        }
      }
    }
    CHECK(q.State() == LP_RUN);
    CHECK(ids[0] >= 66 && ids[0] <= 67); // 2000 ms / 30 ms
    // one 0x32 and one aux poll per 0x30 (+-1 at the window edge)
    CHECK(ids[1] - ids[0] >= -1 && ids[1] - ids[0] <= 1);
    CHECK(ids[2] - ids[0] >= -1 && ids[2] - ids[0] <= 1);
    CHECK_EQ(q.Status().raw33[7], 8);
    CHECK_EQ(q.Status().raw34[0], 1);
  }
}
