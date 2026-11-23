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
#include "adapt.h"
#include "gears.h"
#include "tcu.h"

Adapt::Adapt()
    : cfg(), shifts(0), status(ADA_BLK_DISABLED), dirty(false), inShift(false),
      startOk(false), from(0), to(0), shiftMs(0), flare(0) {
  Reset();
  dirty = false;
}

void Adapt::Reset() {
  for (int c = 0; c < ADA_CLUTCHES; c++)
    for (int k = 0; k < ADA_CLASSES; k++)
      ofs[c][k] = 0;
  shifts = 0;
  dirty = true;
}

int Adapt::Class(int32_t t) {
  if (t >= 10 && t <= 30)
    return ADA_COLD;
  if (t >= 50 && t <= 110)
    return ADA_WARM;
  return -1;
}

int Adapt::LearnClutch(int f, int t) {
  if (f == 3 && t == 4) return CL_D;
  if (f == 4 && t == 5) return CL_C;
  if (f == 5 && t == 6) return CL_E;
  if (f == 6 && t == 7) return CL_A;
  if (f == 6 && t == 5) return CL_B; // downshift: B is never on-coming upwards
  return -1;
}

int32_t Adapt::Offset(int clutch, int32_t temp) const {
  if (clutch < 0 || clutch >= ADA_CLUTCHES)
    return 0;
  // Nearer class: cold below 40 degC (the middle of the gap), else warm.
  return ofs[clutch][temp < 40 ? ADA_COLD : ADA_WARM];
}

uint32_t Adapt::Gate(const AdaInputs &in) const {
  uint32_t b = 0;
  if (!cfg.enable) b |= ADA_BLK_DISABLED;
  if (in.m_t < cfg.tqMin || in.m_t > cfg.tqMax) b |= ADA_BLK_TORQUE;
  if (in.n_mot < cfg.rpmMin || in.n_mot > cfg.rpmMax) b |= ADA_BLK_RPM;
  if (in.temp < cfg.oilMin || in.temp > cfg.oilMax) b |= ADA_BLK_OIL;
  if (Class(in.temp) < 0) b |= ADA_BLK_CLASS;
  int g = in.mode == TCU_SHIFT ? in.target : in.gear;
  int lo = in.mode == TCU_SHIFT && in.gear < g ? in.gear : g;
  int hi = in.mode == TCU_SHIFT && in.gear > g ? in.gear : g;
  if (lo < cfg.gearMin || hi > cfg.gearMax || !Gears::IsDriveGear(g) || g == GEAR_R)
    b |= ADA_BLK_GEAR;
  if (in.pedal < cfg.pedalMin) b |= ADA_BLK_PEDAL;
  if (in.drivenS < cfg.driveMinS) b |= ADA_BLK_DRIVETIME;
  if (!in.normalMode) b |= ADA_BLK_MODE;
  if (!in.faultFree) b |= ADA_BLK_FAULT;
  return b;
}

void Adapt::Learn(int clutch, int cls, int32_t ms, int32_t fl) {
  if (shifts < 0xFFFF)
    shifts++; // every valid learning shift counts, changed or not
  int32_t o = ofs[clutch][cls];
  if (fl > cfg.flareRpm || ms > cfg.shiftMs + cfg.bandMs)
    o += cfg.step; // late fill: a little more fill pressure
  else if (ms < cfg.shiftMs - cfg.bandMs)
    o -= cfg.step; // harsh: a little less
  else
    return;
  if (o > cfg.maxOfs) o = cfg.maxOfs;
  if (o < -cfg.maxOfs) o = -cfg.maxOfs;
  if (o != ofs[clutch][cls]) {
    ofs[clutch][cls] = (int16_t)o;
    dirty = true;
  }
}

void Adapt::Tick10ms(const AdaInputs &in) {
  status = Gate(in);
  if (in.mode == TCU_SHIFT) {
    if (!inShift) {
      inShift = true;
      from = in.gear;
      to = in.target;
      shiftMs = 0;
      flare = 0;
      startOk = status == 0 && LearnClutch(from, to) >= 0;
    }
    shiftMs += 10;
    if (from < to) {
      // Upshift flare: input speed above the old gear's sync speed.
      int32_t f = in.n_t - Gears::SyncSpeed(from, in.n_ab);
      if (f > flare)
        flare = f;
    }
    return;
  }
  if (inShift) {
    inShift = false;
    // Only a completed shift into the planned gear counts.
    if (startOk && status == 0 && in.mode == TCU_DRIVE && in.gear == to) {
      int cl = LearnClutch(from, to);
      int cls = Class(in.temp);
      if (cl >= 0 && cls >= 0)
        Learn(cl, cls, shiftMs, flare);
    }
  }
}

uint32_t Adapt::Crc(const AdaImage &img) {
  // CRC-32 (poly 0xEDB88320) over everything before the crc field.
  const uint8_t *p = (const uint8_t *)&img;
  uint32_t n = (uint32_t)((const uint8_t *)&img.crc - p);
  uint32_t c = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < n; i++) {
    c ^= p[i];
    for (int k = 0; k < 8; k++)
      c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
  }
  return ~c;
}

void Adapt::Pack(AdaImage *img) {
  *img = AdaImage();
  img->magic = ADA_MAGIC;
  img->version = ADA_VERSION;
  img->shifts = shifts;
  for (int c = 0; c < ADA_CLUTCHES; c++)
    for (int k = 0; k < ADA_CLASSES; k++)
      img->ofs[c][k] = ofs[c][k];
  img->crc = Crc(*img);
  dirty = false;
}

bool Adapt::Unpack(const AdaImage &img) {
  if (img.magic != ADA_MAGIC || img.version != ADA_VERSION || img.crc != Crc(img)) {
    Reset();
    dirty = false;
    return false;
  }
  for (int c = 0; c < ADA_CLUTCHES; c++)
    for (int k = 0; k < ADA_CLASSES; k++) {
      int16_t v = img.ofs[c][k];
      ofs[c][k] = v; // limited again by Learn; a reset clears them
    }
  shifts = img.shifts;
  dirty = false;
  return true;
}
