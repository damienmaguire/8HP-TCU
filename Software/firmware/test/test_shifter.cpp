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
#include "shifter.h"
#include "tcu.h"
#include "test.h"

static void Upd(Shifter &s, const ShifterRaw &r, int n) {
  for (int i = 0; i < n; i++)
    s.Update(r);
}

void TestShifter() {
  // CAN shifter: selector and tips from the command frame
  Shifter s;
  ShifterConfig c = {SHIFTER_CAN, 1500, 3000, 200, 6000, 2};
  s.Configure(c);
  ShifterRaw r = {};
  r.canSelector = SEL_D;
  s.Update(r);
  CHECK_EQ(s.Selector(), SEL_N); // needs two equal samples
  Upd(s, r, 2);
  CHECK_EQ(s.Selector(), SEL_D);
  r.canSelector = 7; // out of range -> N
  Upd(s, r, 3);
  CHECK_EQ(s.Selector(), SEL_N);
  // Tip: one request on the stable rising edge, no repeat while held
  r.canSelector = SEL_D;
  Upd(s, r, 3);
  r.canTipUp = true;
  int ups = 0;
  for (int i = 0; i < 20; i++) {
    s.Update(r);
    ups += s.TipUp();
    CHECK(!s.TipDown());
  }
  CHECK_EQ(ups, 1);
  r.canTipUp = false;
  Upd(s, r, 3);
  // A one-sample glitch is not a tip
  r.canTipDown = true;
  s.Update(r);
  CHECK(!s.TipDown());
  r.canTipDown = false;
  s.Update(r);
  CHECK(!s.TipDown());
  Upd(s, r, 3);
  r.canTipDown = true;
  int downs = 0;
  for (int i = 0; i < 5; i++) {
    s.Update(r);
    downs += s.TipDown();
  }
  CHECK_EQ(downs, 1);
  // Both at once: ignored
  r.canTipDown = false;
  Upd(s, r, 3);
  r.canTipUp = r.canTipDown = true;
  ups = downs = 0;
  for (int i = 0; i < 5; i++) {
    s.Update(r);
    ups += s.TipUp();
    downs += s.TipDown();
  }
  CHECK_EQ(ups + downs, 0);

  // Analog placeholder mapping
  Shifter a;
  c.type = SHIFTER_ANALOG;
  a.Configure(c);
  ShifterRaw q = {};
  q.parkIn = true;
  q.notParkIn = false;
  q.anaMv = 0; // ana_in is not looked at in P
  Upd(a, q, 2);
  CHECK_EQ(a.Selector(), SEL_P);
  CHECK(!a.Fault());
  q.parkIn = false;
  q.notParkIn = true;
  q.anaMv = 1000;
  Upd(a, q, 2);
  CHECK_EQ(a.Selector(), SEL_R);
  q.anaMv = 2000;
  Upd(a, q, 2);
  CHECK_EQ(a.Selector(), SEL_N);
  q.anaMv = 4000;
  Upd(a, q, 2);
  CHECK_EQ(a.Selector(), SEL_D);
  CHECK(!a.Fault());
  // ana_in out of range -> N and a fault
  q.anaMv = 6500;
  Upd(a, q, 2);
  CHECK_EQ(a.Selector(), SEL_N);
  CHECK(a.Fault());
  // park_in and notpark_in both high (or both low) -> N and a fault
  q.anaMv = 4000;
  q.parkIn = true;
  Upd(a, q, 2);
  CHECK_EQ(a.Selector(), SEL_N);
  CHECK(a.Fault());
  q.parkIn = q.notParkIn = false;
  Upd(a, q, 2);
  CHECK(a.Fault());
  q.notParkIn = true;
  Upd(a, q, 2);
  CHECK(!a.Fault());
  CHECK_EQ(a.Selector(), SEL_D);
  // din1 = tip up, din2 = tip down; the CAN fields are ignored
  q.canTipUp = true;
  Upd(a, q, 3);
  q.din1 = true;
  ups = 0;
  for (int i = 0; i < 4; i++) {
    a.Update(q);
    ups += a.TipUp();
  }
  CHECK_EQ(ups, 1);
  q.din1 = false;
  q.din2 = true;
  downs = 0;
  for (int i = 0; i < 4; i++) {
    a.Update(q);
    downs += a.TipDown();
  }
  CHECK_EQ(downs, 1);

  // 12 V shift inputs: debounced (3 samples), one tip per press, a
  // bouncing contact gives no extra tips, works with either ShifterType.
  {
    Shifter z;
    ShifterConfig zc = {SHIFTER_CAN, 1500, 3000, 200, 6000, 3};
    z.Configure(zc);
    ShifterRaw q = {};
    int ups = 0, downs = 0;
    const bool bounce[] = {1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0};
    for (unsigned i = 0; i < sizeof(bounce); i++) {
      q.up12 = bounce[i];
      z.Update(q);
      ups += z.TipUp();
      downs += z.TipDown();
    }
    CHECK_EQ(ups, 1);
    CHECK_EQ(downs, 0);
    // Two short samples only: not taken
    q.down12 = true;
    z.Update(q);
    z.Update(q);
    q.down12 = false;
    z.Update(q);
    CHECK(!z.TipDown());
    // A held press: one tip
    q.down12 = true;
    for (int i = 0; i < 50; i++) {
      z.Update(q);
      downs += z.TipDown();
    }
    CHECK_EQ(downs, 1);
    q.down12 = false;
    for (int i = 0; i < 5; i++)
      z.Update(q);
    // Both at once: ignored
    q.up12 = q.down12 = true;
    for (int i = 0; i < 5; i++) {
      z.Update(q);
      CHECK(!z.TipUp() && !z.TipDown());
    }
    // Analog shifter type: still active
    zc.type = SHIFTER_ANALOG;
    Shifter w;
    w.Configure(zc);
    ShifterRaw a = {};
    a.parkIn = true;
    a.up12 = true;
    ups = 0;
    for (int i = 0; i < 5; i++) {
      w.Update(a);
      ups += w.TipUp();
    }
    CHECK_EQ(ups, 1);
  }
}
