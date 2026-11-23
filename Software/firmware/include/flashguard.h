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
#ifndef FLASHGUARD_H
#define FLASHGUARD_H

#include <stdint.h>

/* Checks on what is stored in flash before we trust it.
 *
 * The TCU uses the same flash pages as the ZombieVerter (libopeninv
 * layout: parameters in the last 2 KiB page, the CAN map in the one
 * below). A board that last ran the ZombieVerter therefore still holds
 * the ZombieVerter's parameters and CAN map, with valid CRCs. Their ids
 * mean different things here (for example ZV id 116 BrkVacHyst = 2500
 * lands on our Can1Baud, which only allows 0 to 4), and libopeninv
 * loads them without a range check. So before loading we look at the
 * keys in the parameter page: if any key is not one of our parameter
 * ids (or one we used before, see Retired), the page was written by other firmware and we keep our defaults
 * (and do not load the CAN map either).
 *
 * Hardware free, so the host tests can run it.
 */
namespace FlashGuard {

// One entry of the libopeninv parameter page (param_save.cpp).
struct StoredParam {
  uint16_t key;
  uint8_t dummy;
  uint8_t flags;
  uint32_t value;
};

// Key of an unused entry (the page is filled with 0xFF before saving).
const uint16_t KEY_BLANK = 0xFFFF;

/* Number of used entries whose key is not one of ours. known(id) says
 * whether id is one of our parameters. 0 means the page is ours (or blank).
 */
int UnknownKeys(const StoredParam *page, int entries, bool (*known)(uint16_t id));

/* True for an id this firmware used before and has removed
 * (PARAM_RETIRED_IDS in param_prj.h). A page we saved with an older build
 * holds these, and must still load. */
bool Retired(uint16_t id);

// The value if it is inside min..max, else the default.
int32_t InRange(int32_t value, int32_t min, int32_t max, int32_t def);

} // namespace FlashGuard

#endif // FLASHGUARD_H
