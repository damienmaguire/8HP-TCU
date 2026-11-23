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
#include "valveout.h"

ValveOut::ValveOut() : ok(false), lost(false) {
  for (int i = 0; i < VALVE_MAX_CHIPS; i++) {
    chips[i] = 0;
    flags[i] = 0;
  }
  for (int v = 0; v < EDS_COUNT; v++)
    map[v] = VALVE_NOT_FITTED;
}

void ValveOut::AttachChip(uint8_t idx, Max22200 *chip) {
  if (idx < VALVE_MAX_CHIPS)
    chips[idx] = chip;
}

uint16_t ValveOut::CriticalMask() {
  return (1u << EDS_A) | (1u << EDS_B) | (1u << EDS_C) | (1u << EDS_D) |
         (1u << EDS_E) | (1u << EDS_SYS);
}

bool ValveOut::Validate() const {
  uint16_t used = 0;
  for (int v = 0; v < EDS_COUNT; v++) {
    uint8_t g = map[v];
    if (g == VALVE_NOT_FITTED) {
      if (CriticalMask() & (1u << v))
        return false; // a pressure valve must be driven
      continue;
    }
    if (g >= VALVE_MAX_CHIPS * MAX22200_CHANNELS || chips[g >> 3] == 0)
      return false;
    if (used & (1u << g))
      return false;
    used |= 1u << g;
  }
  return true;
}

bool ValveOut::Init(const Max22200Config &cfg, uint16_t halfScaleMask) {
  ok = false;
  lost = false;
  if (!Validate())
    return false;
  bool good = true;
  for (int i = 0; i < VALVE_MAX_CHIPS; i++) {
    Max22200Config c = cfg;
    c.halfScaleMask = (uint8_t)(halfScaleMask >> (8 * i));
    if (chips[i] && !chips[i]->Init(c))
      good = false;
  }
  ok = good;
  return good;
}

void ValveOut::Update(const Eds &eds) {
  if (!ok)
    return;
  uint8_t on[VALVE_MAX_CHIPS] = {0};
  for (int v = 0; v < EDS_COUNT; v++) {
    uint8_t g = map[v];
    if (g == VALVE_NOT_FITTED)
      continue;
    int32_t i = eds.Output((EdsValve)v);
    chips[g >> 3]->SetCurrent(g & 7, i);
    if (i > 0)
      on[g >> 3] |= 1u << (g & 7);
  }
  for (int c = 0; c < VALVE_MAX_CHIPS; c++) {
    if (chips[c] && chips[c]->GetOnMask() != on[c])
      chips[c]->SetOnMask(on[c]);
  }
}

void ValveOut::AllOff() {
  for (int c = 0; c < VALVE_MAX_CHIPS; c++)
    if (chips[c])
      chips[c]->SetOnMask(0);
}

uint16_t ValveOut::PollFaults() {
  uint16_t valves = 0;
  for (int c = 0; c < VALVE_MAX_CHIPS; c++) {
    if (!chips[c])
      continue;
    Max22200Faults f = chips[c]->ReadFaults();
    flags[c] = f.status;
    bool chipLost = (f.status & MAX22200_ST_UVM) || chips[c]->EchoError();
    if (chipLost)
      lost = true;
    uint8_t chFault = f.ocp | f.hhf | f.olf | f.dpm;
    // Over-temperature and a lost chip affect every channel of that chip.
    if ((f.status & MAX22200_ST_OVT) || chipLost)
      chFault = 0xFF;
    for (int v = 0; v < EDS_COUNT; v++) {
      uint8_t g = map[v];
      if (g != VALVE_NOT_FITTED && (g >> 3) == c && (chFault & (1u << (g & 7))))
        valves |= 1u << v;
    }
  }
  return valves;
}
