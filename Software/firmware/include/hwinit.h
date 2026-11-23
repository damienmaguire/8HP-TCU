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
#ifndef HWINIT_H_INCLUDED
#define HWINIT_H_INCLUDED

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void clock_setup(void);
void rtc_setup(void);
void nvic_setup(void);
/// SPI for the MAX22200s: mode 0 (CPOL 0, CPHA 0), MSB first, 8 bit.
void max_spi_setup(void);
/// Free-running timer for the speed sensor timestamps.
void speed_timer_setup(void);
/// EXTI on the three speed sensor inputs, rising edge.
void speed_exti_setup(void);
uint8_t max_spi_transfer(uint8_t tx);

#ifdef __cplusplus
}
#endif

#endif // HWINIT_H_INCLUDED
