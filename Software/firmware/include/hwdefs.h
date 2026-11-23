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
#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED

/* STM32F107VCT6: 256 KiB flash in 2 KiB pages, 64 KiB RAM.
 * Clock: 8 MHz HSE -> 72 MHz SYSCLK, APB1 36 MHz, APB2 72 MHz
 * (RCC_CLOCK_HSE8_72MHZ, same as the ZombieVerter). The crystal on Bob's
 * board is TBD; change clock_setup() in hwinit.cpp if it is not 8 MHz.
 */

#define USART_BAUDRATE 115200
#define PERIPH_CLK ((uint32_t)36000000)

/* Terminal on USART3 like the ZombieVerter. TX/RX pins are TBD. */
#define TERM_USART USART3
#define TERM_USART_TXPORT GPIOB
#define TERM_USART_TXPIN GPIO_USART3_TX
#define TERM_USART_DR USART3_DR
#define TERM_USART_DMARX DMA_CHANNEL3
#define TERM_USART_DMATX DMA_CHANNEL2
#define TERM_BUFSIZE 128

/* Parameter and CAN map storage at the top of flash (libopeninv F1
 * layout): parameters in the last 2 KiB page, the CAN map in the two
 * pages below it. The linker script keeps the image clear of them.
 */
#define FLASH_PAGE_SIZE 2048
#define PARAM_BLKNUM 1
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define CAN1_BLKNUM 2
#define CAN2_BLKNUM 4

/* SPI port for the MAX22200 solenoid drivers. TBD: SPI1 on PA5/PA6/PA7
 * is a placeholder until Bob's pinout is final.
 */
#define MAX_SPI SPI1
#define MAX_SPI_RCC RCC_SPI1
#define MAX_SPI_PORT GPIOA
#define MAX_SPI_SCK GPIO5
#define MAX_SPI_MISO GPIO6
#define MAX_SPI_MOSI GPIO7

/* Speed sensor edge timestamps: TIM2 free running at 500 kHz (16 bit,
 * wraps after 131 ms). The sensor pins are EXTI inputs, see digio_prj.h.
 */
#define SPEED_TIMER TIM2
#define SPEED_TIMER_RCC RCC_TIM2
#define SPEED_TIMER_HZ 500000u

#endif // HWDEFS_H_INCLUDED
