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
/// SWD only, TIM3 full remap (PC6..9), CAN2 on PB5/PB6.
void afio_setup(void);
void nvic_setup(void);
/// SPI1 for the MAX22200: mode 0 (CPOL 0, CPHA 0), MSB first, 8 bit.
void max_spi_setup(void);
uint8_t max_spi_transfer(uint8_t tx);
/// Speed sensor pins. V4: ADC2 injected sampling, TIM1-triggered (buf
/// unused). Rev B: ADC1 continuous scan, circular DMA into buf
/// (2 x SPEED_HALF_SCANS x SPEED_NUM samples).
void speed_adc_setup(volatile uint16_t *buf);
/// gp_pwm on PC6 (TIM3_CH1), inverted polarity. V4 only (rev B: IC4 OUT3).
void gp_pwm_setup(uint32_t freqHz);
void gp_pwm_set(uint32_t dutyPct);
/// UART4 in LIN mode for the aux oil pump.
void lin_usart_setup(void);

#ifdef __cplusplus
}
#endif

#endif // HWINIT_H_INCLUDED
