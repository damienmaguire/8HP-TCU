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
/// SWD only (PA15 free), CAN2 on PB5/PB6, TIM3_CH1 on PC6.
void afio_setup(void);
void nvic_setup(void);
/// SPI1 for the TLE92466ED: mode 1 (CPOL 0, CPHA 1), MSB first, 16 bit.
void tle_spi_setup(void);
uint16_t tle_spi_xfer16(uint16_t tx);
/// OUT3 (PC6) PWM on TIM3_CH1: frequency once at start-up, duty in %.
void gp_pwm_setup(uint32_t hz);
void gp_pwm_duty(uint32_t pct);
/// Speed sensor pins: ADC1 continuous scan, circular DMA into buf
/// (2 x SPEED_HALF_SCANS x SPEED_NUM samples).
void speed_adc_setup(volatile uint16_t *buf);
/// UART4 in LIN mode for the aux oil pump.
void lin_usart_setup(void);

#ifdef __cplusplus
}
#endif

#endif // HWINIT_H_INCLUDED
