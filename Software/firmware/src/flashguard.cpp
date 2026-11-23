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
#include "flashguard.h"
#include "param_prj.h"

namespace FlashGuard {

int UnknownKeys(const StoredParam *page, int entries, bool (*known)(uint16_t id)) {
  int n = 0;
  for (int i = 0; i < entries; i++) {
    uint16_t k = page[i].key;
    if (k != KEY_BLANK && !known(k))
      n++;
  }
  return n;
}

bool Retired(uint16_t id) {
  static const uint16_t ids[] = {PARAM_RETIRED_IDS};
  for (uint16_t r : ids)
    if (r == id)
      return true;
  return false;
}

int32_t InRange(int32_t value, int32_t min, int32_t max, int32_t def) {
  return (value < min || value > max) ? def : value;
}

} // namespace FlashGuard
