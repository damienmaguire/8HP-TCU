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
/* CAN gear levers: BMW GWS against the F11 capture, the ZV ports with
 * frames from the ZV source. */
#include "canlever.h"
#include "gwslever.h"
#include "linpump.h"
#include "tcu.h"
#include "test.h"

struct LogFrame
{
   uint32_t t;
   uint16_t id;
   uint8_t dlc;
   uint8_t d[8];
};

static const LogFrame f11Log[] = {
#include "gws_f11_frames.inc"
};

// A 0x197 frame with a valid CRC.
static void Gws197(uint8_t cnt, uint8_t b2, uint8_t b3, uint8_t d[4]) {
  d[1] = cnt & 0x0F;
  d[2] = b2;
  d[3] = b3;
  d[0] = GwsLever::RxCrc(d);
}

static CanLever MakeLever(uint8_t type) {
  CanLever l;
  LeverConfig c = {type, 0, 2, 1, GWS_B34_OEM};
  l.Configure(c);
  return l;
}

static void TestGwsCapture() {
  const int n = sizeof(f11Log) / sizeof(f11Log[0]);
  CHECK(n > 1000);
  // Every captured 0x197 passes CRC (E2E 0x003F == XOR 0x53) and every
  // captured 0x3FD is rebuilt bit for bit by our packer (E2E 0x0039 ==
  // XOR 0x70, OEM B3 / B4 = FC FF).
  int rx = 0, tx = 0, txSame = 0;
  for (const LogFrame &f : f11Log) {
    if (f.id == GWS_RX_ID) {
      rx++;
      CHECK_EQ(GwsLever::RxCrc(f.d), f.d[0]);
      CHECK_EQ(f.d[2] & 0x0F, 0x0F); // F11 low nibble
    } else {
      tx++;
      uint8_t d[5];
      GwsDisplay::Pack(f.d[2], f.d[1], f.d[3], f.d[4], d);
      int same = 1;
      for (int i = 0; i < 5; i++)
        same &= d[i] == f.d[i];
      txSame += same;
      // E2E form: CRC over Data ID 0x0039 lo / hi + B1..B4.
      uint8_t e[6] = {0x39, 0x00, f.d[1], f.d[2], f.d[3], f.d[4]};
      CHECK_EQ(Lin::Crc8(e, 6), f.d[0]);
    }
  }
  CHECK(rx > 900);
  CHECK(tx > 100);
  CHECK_EQ(txSame, tx);

  // Replay the lever frames in time; the TCU-side state must walk the
  // same positions the OEM EGS displayed: P, N, R, D, P.
  CanLever l = MakeLever(LEVER_BMW_GWS);
  uint32_t t = f11Log[0].t;
  uint8_t seq[8], ns = 0, last = 0xFF;
  int rejected = 0;
  for (const LogFrame &f : f11Log) {
    while (t + 10 <= f.t) {
      l.Tick10ms(true);
      t += 10;
      if (l.State() != last && ns < 8)
        seq[ns++] = last = l.State();
    }
    if (f.id == GWS_RX_ID && !l.OnFrame(f.id, f.d, f.dlc))
      rejected++;
  }
  CHECK_EQ(rejected, 0);
  CHECK_EQ(l.CrcErrors(), 0);
  CHECK_EQ(l.CounterErrors(), 0);
  const uint8_t want[5] = {GWS_P, GWS_N, GWS_R, GWS_D, GWS_P};
  CHECK_EQ(ns, 5);
  for (int i = 0; i < 5 && i < ns; i++)
    CHECK_EQ(seq[i], want[i]);
  // The display positions in the log, in order of change.
  uint8_t dseq[8], nd = 0, lastPos = 0xFF;
  for (const LogFrame &f : f11Log)
    if (f.id == GWS_TX_ID && f.d[2] != lastPos && nd < 8)
      dseq[nd++] = lastPos = f.d[2];
  const uint8_t wantPos[5] = {0x20, 0x60, 0x40, 0x80, 0x20};
  CHECK_EQ(nd, 5);
  for (int i = 0; i < 5 && i < nd; i++)
    CHECK_EQ(dseq[i], wantPos[i]);
}

static void TestGwsRules() {
  GwsLever g;
  uint8_t d[4];
  uint8_t c = 0;
  auto feed = [&](uint8_t b2, uint8_t b3, int frames, bool ok = true) {
    for (int i = 0; i < frames; i++) {
      Gws197(c, b2, b3, d);
      c = (uint8_t)((c + 1) % 15);
      CHECK(g.OnFrame(d, 4));
      g.Tick10ms(ok);
      g.Tick10ms(ok);
      g.Tick10ms(ok);
    }
  };
  CHECK(!g.Valid());
  CHECK_EQ(g.State(), GWS_P);
  // F30 lever: low nibble 0xE (Damien's frames 0x?E, P c0 / d5).
  feed(0x0E, 0xC0, 3);
  CHECK(g.Valid());
  feed(0x3E, 0xC0, 3); // Down1: P -> N
  CHECK_EQ(g.State(), GWS_N);
  feed(0x0E, 0xC0, 2);
  feed(0x4E, 0xC0, 3); // Down2: N -> D
  CHECK_EQ(g.State(), GWS_D);
  CHECK_EQ(g.Selector(), SEL_D);
  feed(0x0E, 0xC0, 2);
  // Side gate: M, M+ / M- tips, back to centre -> D.
  feed(0x7E, 0xC0, 3);
  CHECK_EQ(g.State(), GWS_M);
  CHECK_EQ(g.Selector(), SEL_D);
  Gws197(c, 0x5E, 0xC0, d); c++; g.OnFrame(d, 4); g.Tick10ms(true);
  Gws197(c, 0x5E, 0xC0, d); c++; g.OnFrame(d, 4); g.Tick10ms(true);
  CHECK(g.TipUp());
  g.Tick10ms(true);
  CHECK(!g.TipUp()); // one-shot
  feed(0x7E, 0xC0, 2);
  Gws197(c, 0x6E, 0xC0, d); c++; g.OnFrame(d, 4); g.Tick10ms(true);
  Gws197(c, 0x6E, 0xC0, d); c++; g.OnFrame(d, 4); g.Tick10ms(true);
  CHECK(g.TipDown());
  feed(0x0E, 0xC0, 3);
  CHECK_EQ(g.State(), GWS_D);
  // Engage gate: Up2 to R refused at speed, display flashes.
  feed(0x2F, 0xC0, 3, false);
  CHECK_EQ(g.State(), GWS_D);
  CHECK(g.Flash());
  feed(0x0F, 0xC0, 3);
  feed(0x1F, 0xC0, 3); // Up1: D -> N (never gated)
  CHECK_EQ(g.State(), GWS_N);
  // P button (F11 d5), needs the redundant copies to agree.
  feed(0x0F, 0xD1, 3); // copies disagree: not a press
  CHECK_EQ(g.State(), GWS_N);
  feed(0x0F, 0xD5, 3);
  CHECK_EQ(g.State(), GWS_P);
  CHECK(g.PButton());
  feed(0x0F, 0xC0, 3);
  // One-frame move is not taken (debounce 2).
  feed(0x3F, 0xC0, 1);
  feed(0x0F, 0xC0, 2);
  CHECK_EQ(g.State(), GWS_P);

  // Frame checks.
  uint16_t e0 = g.CounterErrors();
  Gws197(c, 0x0F, 0xC0, d);
  CHECK(g.OnFrame(d, 4));
  CHECK(g.OnFrame(d, 4));  // one repeat tolerated
  CHECK(!g.OnFrame(d, 4)); // the second is not
  Gws197((uint8_t)((c + 4) % 15), 0x0F, 0xC0, d);
  CHECK(!g.OnFrame(d, 4)); // jump of 4
  Gws197(15, 0x0F, 0xC0, d);
  CHECK(!g.OnFrame(d, 4)); // 15 is not a counter value
  CHECK_EQ(g.CounterErrors(), e0 + 3);
  Gws197(c, 0x0F, 0xC0, d);
  d[0] ^= 1;
  CHECK(!g.OnFrame(d, 4));
  CHECK_EQ(g.CrcErrors(), 1);
  CHECK(!g.OnFrame(d, 5)); // DLC 4 only
  CHECK_EQ(g.DlcErrors(), 1);
  // Timeout 90 ms.
  for (int i = 0; i < 10; i++)
    g.Tick10ms(true);
  CHECK(!g.Valid());
}

static void TestGwsDisplay() {
  GwsDisplay disp;
  uint8_t d[5];
  disp.Tick100ms(false, GWS_D, false, 1, GWS_B34_OEM, 3, d);
  CHECK_EQ(d[2], 0x00); // before KL15: off
  CHECK_EQ(d[3], 0xFC);
  CHECK_EQ(d[4], 0xFF);
  CHECK_EQ(d[1], 0);
  disp.Tick100ms(true, GWS_P, false, 1, GWS_B34_OEM, 0, d);
  CHECK_EQ(d[2], 0x20);
  CHECK_EQ(d[1], 1);
  disp.Tick100ms(true, GWS_M, true, 1, GWS_B34_ZV, 4, d);
  CHECK_EQ(d[2], 0x80 | 0x08 | 0x01); // D, flash, M sub-field
  CHECK_EQ(d[3], 0x00);
  CHECK_EQ(d[4], 4);
  CHECK_EQ(d[0], GwsLever::TxCrc(d));
  for (int i = 0; i < 299; i++)
    disp.Tick100ms(false, GWS_P, false, 1, GWS_B34_OEM, 0, d);
  CHECK_EQ(d[2], 0x20); // still P up to OFF + 30 s
  disp.Tick100ms(false, GWS_P, false, 1, GWS_B34_OEM, 0, d);
  CHECK_EQ(d[2], 0x00);
  // Counter wraps 14 -> 0.
  GwsDisplay d2;
  for (int i = 0; i < 16; i++)
    d2.Tick100ms(true, GWS_N, false, 0, 0, 0, d);
  CHECK_EQ(d[1], 0); // 16th frame: 15 -> wraps to 0
  // Through CanLever: one frame every 100 ms, never 0x202.
  CanLever l = MakeLever(LEVER_BMW_GWS);
  LeverTxIn in = {true, SEL_P, 0};
  CanFrame f[LEVER_MAX_TX];
  int frames = 0;
  for (int i = 0; i < 100; i++) {
    int n = l.Tx10ms(in, f);
    for (int k = 0; k < n; k++) {
      CHECK_EQ(f[k].id, GWS_TX_ID);
      CHECK(f[k].id != 0x202);
      CHECK_EQ(f[k].dlc, 5);
    }
    frames += n;
  }
  CHECK_EQ(frames, 10);
}

static void Feed(CanLever &l, uint32_t id, const uint8_t *d, uint8_t dlc, int n, bool ok = true) {
  for (int i = 0; i < n; i++) {
    CHECK(l.OnFrame(id, d, dlc));
    l.Tick10ms(ok);
  }
}

static void TestE65() {
  CanLever l = MakeLever(LEVER_BMW_E65);
  uint32_t ids[2];
  CHECK_EQ(l.RxIds(ids), 1);
  CHECK_EQ(ids[0], 0x192);
  // ZV E65_Lever.cpp: data[0] & 0xFFFFFF patterns, little-endian.
  const uint8_t rest[3] = {0x6A, 0x00, 0x80};  // 0x80006a not pressed
  const uint8_t rPlus[3] = {0x2D, 0x04, 0x80}; // 0x80042d R+
  const uint8_t dPlus[3] = {0x74, 0x03, 0x80}; // 0x800374 D+
  const uint8_t park[3] = {0x6A, 0x50, 0x80};  // 0x80506a P button
  const uint8_t junk[3] = {0x11, 0x22, 0x33};
  Feed(l, 0x192, rest, 3, 3);
  CHECK(l.Valid());
  CHECK_EQ(l.State(), GWS_P);
  Feed(l, 0x192, rPlus, 3, 3);
  CHECK_EQ(l.State(), GWS_R);
  Feed(l, 0x192, rest, 3, 2);
  Feed(l, 0x192, dPlus, 3, 3);
  CHECK_EQ(l.State(), GWS_D);
  Feed(l, 0x192, rest, 3, 2);
  Feed(l, 0x192, park, 3, 3, false); // P refused at speed
  CHECK_EQ(l.State(), GWS_D);
  CHECK(l.Flash());
  Feed(l, 0x192, rest, 3, 2);
  Feed(l, 0x192, park, 3, 3);
  CHECK_EQ(l.State(), GWS_P);
  CHECK(!l.OnFrame(0x192, junk, 3));
  CHECK(!l.OnFrame(0x197, rest, 3));
  LeverTxIn in = {true, SEL_P, 0};
  CanFrame f[LEVER_MAX_TX];
  CHECK_EQ(l.Tx10ms(in, f), 0); // no TX for the E65
  for (int i = 0; i < 50; i++)
    l.Tick10ms(true);
  CHECK(!l.Valid());
}

static void TestJlr() {
  // Gen 1: RX 0x312 B3 high nibble, TX 0x3F3 every 20 ms (ZV JLR_G1.cpp).
  CanLever l = MakeLever(LEVER_JLR_G1);
  uint8_t rx[8] = {0, 0, 0, 0x00, 0, 0, 0, 0};
  Feed(l, 0x312, rx, 8, 3);
  CHECK_EQ(l.State(), GWS_P);
  rx[3] = 0x30; // D
  Feed(l, 0x312, rx, 8, 3, false);
  CHECK_EQ(l.State(), GWS_P); // gate
  CHECK(l.Flash());
  Feed(l, 0x312, rx, 8, 1, true);
  CHECK_EQ(l.State(), GWS_D);
  rx[3] = 0x70; // Sport
  Feed(l, 0x312, rx, 8, 3);
  CHECK_EQ(l.State(), GWS_M);
  rx[3] = 0x20;
  Feed(l, 0x312, rx, 8, 3, false); // N is never gated
  CHECK_EQ(l.State(), GWS_N);
  CHECK(!l.OnFrame(0x312, rx, 7));
  LeverTxIn in = {true, SEL_N, 0};
  CanFrame f[LEVER_MAX_TX];
  int n = 0;
  for (int i = 0; i < 2; i++)
    n += l.Tx10ms(in, f);
  CHECK_EQ(n, 1);
  // N frame, counter 0: 7C 25 00 04 FD FF 02 00.
  const uint8_t wantN[8] = {0x7C, 0x25, 0x00, 0x04, 0xFD, 0xFF, 0x02, 0x00};
  CHECK_EQ(f[0].id, 0x3F3);
  for (int i = 0; i < 8; i++)
    CHECK_EQ(f[0].data[i], wantN[i]);
  // Not running: park frames, counter skips 2 -> 4, TX stops after 2 s.
  in.run = false;
  uint8_t cnts[4];
  int k = 0;
  for (int i = 0; i < 8 && k < 4; i++)
    if (l.Tx10ms(in, f))
      cnts[k++] = f[0].data[2];
  CHECK_EQ(f[0].data[0], 0x5C);
  CHECK_EQ(f[0].data[1], 0x66);
  CHECK_EQ(cnts[0], 1);
  CHECK_EQ(cnts[1], 4); // 2 skipped in park
  CHECK_EQ(f[0].data[3], (uint8_t)(cnts[3] + 130));
  int late = 0;
  for (int i = 0; i < 300; i++)
    late += l.Tx10ms(in, f);
  CHECK(late < 100);
  CHECK_EQ(l.Tx10ms(in, f), 0);

  // Gen 2: RX 0x0E0 B4 high nibble, TX 0x02C every 10 ms (ZV JLR_G2.cpp).
  CanLever g = MakeLever(LEVER_JLR_G2);
  uint8_t r2[8] = {0, 0, 0, 0, 0x10, 0, 0, 0}; // R
  Feed(g, 0x0E0, r2, 8, 3);
  CHECK_EQ(g.State(), GWS_R);
  LeverTxIn in2 = {true, SEL_R, 0};
  CHECK_EQ(g.Tx10ms(in2, f), 1);
  CHECK_EQ(f[0].id, 0x02C);
  const uint8_t wantR0[8] = {0x00, 0x00, 0x09, 0x40, 0x00, 0x67, 0x01, 0x00};
  for (int i = 0; i < 8; i++)
    CHECK_EQ(f[0].data[i], wantR0[i]);
  CHECK_EQ(g.KnobLock(), 0x40);
  r2[4] = 0x30; // D
  Feed(g, 0x0E0, r2, 8, 3);
  in2.selector = SEL_D;
  g.Tx10ms(in2, f); // counter 1
  const uint8_t wantD1[8] = {0x00, 0x00, 0x09, 0x44, 0x1B, 0x75, 0x04, 0x00};
  for (int i = 0; i < 8; i++)
    CHECK_EQ(f[0].data[i], wantD1[i]);
  in2.run = false; // lower the knob, show P
  g.Tx10ms(in2, f); // counter 2
  const uint8_t wantP2[8] = {0x00, 0x00, 0x10, 0x52, 0x20, 0x06, 0x00, 0x80};
  for (int i = 0; i < 8; i++)
    CHECK_EQ(f[0].data[i], wantP2[i]);
}

static void TestPkp() {
  CanLever l = MakeLever(LEVER_PKP2300);
  uint8_t k[8] = {0};
  LeverTxIn in = {true, SEL_P, 0};
  CanFrame f[LEVER_MAX_TX];
  // Silent keypad: NMT start (0x000: 01 15) with the LED frame.
  int n = 0;
  for (int i = 0; i < 10; i++)
    n = l.Tx10ms(in, f);
  CHECK_EQ(n, 2);
  CHECK_EQ(f[0].id, 0x000);
  CHECK_EQ(f[0].data[0], 0x01);
  CHECK_EQ(f[0].data[1], 0x15);
  CHECK_EQ(f[1].id, 0x215);
  CHECK_EQ(f[1].data[2], 1 << 3); // P blue
  Feed(l, 0x195, k, 8, 3);
  CHECK(l.Valid());
  k[0] = 1 << 0; // K3 drive
  Feed(l, 0x195, k, 8, 3);
  CHECK_EQ(l.State(), GWS_D);
  k[0] = 0;
  Feed(l, 0x195, k, 8, 3);
  k[0] = 1 << 2; // K6 reverse, refused at speed
  Feed(l, 0x195, k, 8, 3, false);
  CHECK_EQ(l.State(), GWS_D);
  for (int i = 0; i < 10; i++)
    n = l.Tx10ms(in, f);
  CHECK_EQ(n, 1); // keypad alive: no NMT
  CHECK_EQ(f[0].id, 0x215);
  CHECK_EQ(f[0].data[0], 1 << 2); // one red cycle on R
  CHECK_EQ(f[0].data[2], 1 << 0); // D blue
  for (int i = 0; i < 10; i++)
    l.Tx10ms(in, f);
  CHECK_EQ(f[0].data[0], 0);
  k[0] = 0;
  Feed(l, 0x195, k, 8, 3);
  k[0] = 1 << 1; // K2 neutral, never gated
  Feed(l, 0x195, k, 8, 3, false);
  CHECK_EQ(l.State(), GWS_N);
}

void TestLever() {
  TestGwsCapture();
  TestGwsRules();
  TestGwsDisplay();
  TestE65();
  TestJlr();
  TestPkp();
  // Analog / VCU frame are not CAN levers.
  CHECK(!MakeLever(LEVER_ANALOG).IsCan());
  CHECK(!MakeLever(LEVER_VCU).IsCan());
  CHECK(MakeLever(LEVER_PKP2300).IsCan());
}
