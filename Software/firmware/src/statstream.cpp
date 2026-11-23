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
/*
 * Status stream: our own four-frame CAN layout with the TCU state for
 * loggers and dashboards (rates and byte layout in statstream.h).
 */
#include "statstream.h"

// 16-bit value, little-endian.
static void Put16(uint8_t *b, int32_t v) {
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
}

// Limit v to lo..hi.
static int32_t Clamp(int32_t v, int32_t lo, int32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Torque reduction asked for, as % of the input torque (0 = none).
int32_t StatStream::ReductPct(int32_t inputNm, int32_t deltaNm) {
  if (deltaNm >= 0 || inputNm <= 0)
    return 0;
  return Clamp(-deltaNm * 100 / inputNm, 0, 100);
}

// Frames due this 10 ms tick (rates in statstream.h).
int StatStream::Tick10ms(uint32_t base, const StreamIn &in, CanFrame out[STREAM_MAX_TX]) {
  int n = 0;
  CanFrame *f = &out[n++];
  f->id = base;
  f->dlc = 8;
  Put16(f->data, Clamp(in.tqTarget, -32768, 32767));
  f->data[2] = (uint8_t)Clamp(in.tqReductPct, 0, 100);
  f->data[3] = (in.shifting ? 1 : 0) | (in.coast ? 2 : 0) | (in.tqActive ? 4 : 0) |
               (in.inertia ? 8 : 0) | (in.tccClosed ? 16 : 0);
  Put16(f->data + 4, Clamp(in.slip, -32768, 32767));
  f->data[6] = counter++;
  f->data[7] = 0;

  f = &out[n++];
  f->id = base + 1;
  f->dlc = 8;
  Put16(f->data, Clamp(in.nT, 0, 65535));
  Put16(f->data + 2, Clamp(in.nAb, 0, 65535));
  Put16(f->data + 4, Clamp(in.nMot, 0, 65535));
  Put16(f->data + 6, Clamp(in.nAux, 0, 65535));

  if (ms >= next40) { // 40 Hz on average: every 25 ms
    next40 += 25;
    f = &out[n++];
    f->id = base + 2;
    f->dlc = 8;
    f->data[0] = in.gear;
    f->data[1] = in.target;
    f->data[2] = in.selector;
    f->data[3] = in.mode;
    f->data[4] = in.leverState;
    f->data[5] = in.driveMode;
    f->data[6] = in.zKzu;
    f->data[7] = in.zKab;
  }
  if (ms % 50 == 0) { // 20 Hz
    f = &out[n++];
    f->id = base + 3;
    f->dlc = 8;
    f->data[0] = in.fault;
    Put16(f->data + 1, in.diagBits);
    f->data[3] = (uint8_t)Clamp(in.atfC + 40, 0, 255);
    f->data[4] = (uint8_t)Clamp(in.boardC + 40, 0, 255);
    f->data[5] = (uint8_t)Clamp(in.vbatMv / 100, 0, 255);
    f->data[6] = in.adaStatus;
    f->data[7] = (in.canLoss ? 1 : 0) | (in.leverTimeout ? 2 : 0) | (in.tqNotCan ? 4 : 0);
  }
  ms += 10;
  return n;
}
