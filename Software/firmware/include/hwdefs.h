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

/* STM32F107VCT6 on Bob's Stm32_TCU_V4 board: 256 KiB flash in 2 KiB
 * pages, 64 KiB RAM. Clock: 8 MHz HSE (V4) -> 72 MHz SYSCLK, APB1 36 MHz,
 * APB2 72 MHz (RCC_CLOCK_HSE8_72MHZ, same as the ZombieVerter).
 * Pin map: Bob's Stm32_TCU_V4/pinmap/firmware_answers.md, approved by
 * Damien. See also digio_prj.h, anain_prj.h and README "Pin map".
 */

/* Board revision. 4 = Stm32_TCU_V4 (default, what is built today).
 * 5 = Bob's next revision. Its pins are not final, so everything it adds
 * is behind the HW_HAS_* switches below with TBD placeholders, and a
 * HW_REV=5 build stops with #error until the TBD pins are filled in.
 * What the next revision adds (Bob, pins TBD unless given):
 *  - a second MAX22200 on SPI1 with its own CSB (TBD); PZYL on its OUT0
 *    (global channel 8, ChanPZYL);
 *  - a third speed input (n6), probably PA2 = ADC12_IN2;
 *  - 12 V shift up / down inputs (TBD);
 *  - terminal 15 (ignition) input (TBD);
 *  - a board NTC like the ZombieVerter V1.3 (TH1: 10k NTC B3450 to GND,
 *    10k pull-up to 3V3, 100 nF): ADC pin and NTC values TBD (Bob).
 *    tBoard then comes from the NTC, else from the STM32 die sensor.
 */
#ifndef HW_REV
#define HW_REV 4
#endif

#if HW_REV >= 5
#define HW_HAS_MAX2 1
#define HW_HAS_SPD3 1
#define HW_HAS_SHIFT12V 1
#define HW_HAS_T15 1
#define HW_HAS_BOARD_NTC 1
/* TBD pins: set these from Bob's final V5 pin map. */
#if !defined(MAX_CS1_PORT) || !defined(MAX_CS1_PIN)
#error "HW_REV 5: MAX22200 #2 chip select pin is TBD (MAX_CS1_PORT / MAX_CS1_PIN)"
#endif
#if !defined(SHIFT_UP_PORT) || !defined(SHIFT_UP_PIN) || !defined(SHIFT_DN_PORT) || !defined(SHIFT_DN_PIN)
#error "HW_REV 5: 12 V shift up / down input pins are TBD (SHIFT_UP_* / SHIFT_DN_*)"
#endif
#if !defined(T15_PORT) || !defined(T15_PIN)
#error "HW_REV 5: terminal 15 input pin is TBD (T15_PORT / T15_PIN)"
#endif
#if !defined(BOARD_NTC_PORT) || !defined(BOARD_NTC_PIN)
#error "HW_REV 5: board NTC ADC pin is TBD (BOARD_NTC_PORT / BOARD_NTC_PIN)"
#endif
#else
#define HW_HAS_MAX2 0
#define HW_HAS_SPD3 0
#define HW_HAS_SHIFT12V 0
#define HW_HAS_T15 0
#define HW_HAS_BOARD_NTC 0
#endif

#define USART_BAUDRATE 115200 // baud, terminal
#define PERIPH_CLK ((uint32_t)36000000) // Hz, APB1 clock

/* Terminal on USART3, PB10 TX / PB11 RX (no remap), as on the ZombieVerter. */
#define TERM_USART USART3
#define TERM_USART_TXPORT GPIOB
#define TERM_USART_TXPIN GPIO_USART3_TX
#define TERM_USART_DR USART3_DR
#define TERM_USART_DMARX DMA_CHANNEL3
#define TERM_USART_DMATX DMA_CHANNEL2
#define TERM_BUFSIZE 128 // bytes, terminal line buffer

/* Parameter and CAN map storage at the top of flash (libopeninv F1
 * layout): block numbers count 2 KiB pages down from the top of flash.
 * Parameters in the last page (1), the CAN1 map in page 2, the CAN2 map in
 * page 4. The linker script keeps the image clear of them.
 */
#define FLASH_PAGE_SIZE 2048 // bytes
#define PARAM_BLKNUM 1
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define CAN1_BLKNUM 2
#define CAN2_BLKNUM 4
/* Clutch adaptation (adapt.h): its own page, 3 pages from the top
 * (0x0803E800 on a 256 KiB part, above the image end 0x0803E000 in
 * stm32_tcu.ld), so parameter and CAN map saves, loads and uploads never
 * touch it. */
#define ADAPT_BLKNUM 3

/* MAX22200 on SPI1, no remap: SCK PA5, MISO PA6 (1k pull-up R28), MOSI
 * PA7. CSB is a software chip select on PA4 (max_cs0, digio_prj.h).
 */
#define MAX_SPI SPI1
#define MAX_SPI_RCC RCC_SPI1
#define MAX_SPI_PORT GPIOA
#define MAX_SPI_SCK GPIO5
#define MAX_SPI_MISO GPIO6
#define MAX_SPI_MOSI GPIO7

/* Speed sensors ISS PA0 (ADC12_IN0) and OSS PA1 (ADC12_IN1). The two-wire
 * current sensors only give about 0.7 V / 1.4 V on 100 ohm, below VIH, so
 * ADC2 samples both pins as an injected group triggered by TIM1 TRGO, and
 * the ISR finds the edges with hysteresis (anaconv.h, AdcEdge). The
 * sample count is the speed time base. TIM1 runs no outputs (PE9/PE11
 * stay GPIOs held low), so it needs no remap.
 */
#define SPEED_ADC ADC2
#define SPEED_ADC_RCC RCC_ADC2
#define SPEED_ADC_CH_IN 0
#define SPEED_ADC_CH_OUT 1
#define SPEED_ADC_CH_AUX 2 // next revision: n6 on PA2 (ADC12_IN2), probably
#define SPEED_NUM (HW_HAS_SPD3 ? 3 : 2)
#define SPEED_TRIG_TIMER TIM1
#define SPEED_TRIG_RCC RCC_TIM1
#define SPEED_TIMER_HZ 40000u // ADC2 samples per second per sensor

/* gp_pwm on PC6 = TIM3_CH1 with the TIM3 full remap (PC6..PC9). The
 * FAN3121 driver inverts: PC6 low = +12 V at J1.12, so the channel runs
 * with inverted output polarity and duty = time at +12 V.
 */
#define GP_PWM_TIMER TIM3
#define GP_PWM_RCC RCC_TIM3

/* LIN for the aux oil pump: UART4, TX PC10 / RX PC11, transceiver SLP_N
 * on PD11 (lin_nslp). Not on V4: the transceiver is planned for the next
 * revision, so the driver is off by default (LinPumpEnable = 0).
 */
#define LIN_USART UART4
#define LIN_USART_RCC RCC_UART4
#define LIN_USART_IRQ NVIC_UART4_IRQ
#define LIN_BAUD 19200 // baud

#endif // HWDEFS_H_INCLUDED
