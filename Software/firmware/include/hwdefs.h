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

/* One board: Bob's Stm32_TCU_V4, rev C (PROVISIONAL: still being routed,
 * Bob's full firmware answers to follow). STM32F107VCT6, 256 KiB flash in
 * 2 KiB pages, 64 KiB RAM. Clock: 8 MHz HSE -> 72 MHz SYSCLK, APB1
 * 36 MHz, APB2 72 MHz (RCC_CLOCK_HSE8_72MHZ, same as the ZombieVerter).
 * Pin map: Bob's Stm32_TCU_V4/pinmap/firmware_answers.md, approved by
 * Damien. See also digio_prj.h and README "Pin map".
 *
 * What the board has:
 *  - two MAX22200 valve drivers on SPI1: IC2 (CSB PA4, nFAULT PE10) and
 *    IC4 (CSB PE12, nFAULT PE13), CMD PE8 and ENABLE PE7 shared. IC2
 *    OUT0..7 = A, B, C, D, E, SYS, POS (bench-check), TCC / K0. IC4 OUT0 =
 *    PZYL (J1.42, global channel 8); OUT1..OUT3 (J1.10/11/12, nets
 *    OUT1_DRV..OUT3_DRV) are the general-purpose outputs; OUT4..7
 *    (J1.16..19) are spare. The park magnet is on PC7 (NCV8402 Q3).
 *  - speed inputs on the ADC with a 10k / 1 nF RC: ISS PA0 (IN0), OSS PA1
 *    (IN1), n6 PA2 (IN2), scanned continuously by ADC1 into a circular
 *    DMA buffer (DMA1 channel 1); the slow analogue inputs are on ADC2
 *    (no DMA on F1 ADC2: software started, read every 10 ms).
 *  - 12 V inputs (ZV1.3 circuit, active high): T15 PD0, BRAKE_IN PD1
 *    (J1.46), SHIFT_UP PD2, SHIFT_DN PD3 (J1.49), D1 / D2 / PARK /
 *    NOT_PARK PD4..PD7.
 *  - CAN1 PA11 / PA12 = vehicle PT-CAN, CAN2 PB5 / PB6 (CAN2_REMAP) =
 *    ZombieVerter link, both 500 kbit/s.
 *  - LIN transceiver TJA1021 on UART4 PC10 / PC11, SLP_N PD11.
 *  - board NTC TH1 on PB0 (ADC12_IN8): 10k at 25 degC, B 3950 K, 10k
 *    pull-up to 3V3, NTC to GND.
 *  - temperature input pull-ups 2.7k / 2.7k to 5 V.
 */

/* Pins of the second MAX22200 and the 12 V inputs not in digio_prj.h's
 * fixed list (Bob, firmware_answers.md sections 0 and 0a). */
#define MAX_CS1_PORT GPIOE
#define MAX_CS1_PIN GPIO12
#define MAX_NFAULT1_PORT GPIOE
#define MAX_NFAULT1_PIN GPIO13
#define SHIFT_UP_PORT GPIOD
#define SHIFT_UP_PIN GPIO2
#define SHIFT_DN_PORT GPIOD
#define SHIFT_DN_PIN GPIO3
#define T15_PORT GPIOD
#define T15_PIN GPIO0
#define BRAKE_IN_PORT GPIOD
#define BRAKE_IN_PIN GPIO1    // PD1, J1.46 BRAKE_IN
#define BOARD_NTC_PORT GPIOB
#define BOARD_NTC_PIN 0       // PB0 = ADC12_IN8
#define BOARD_NTC_ADC_CH 8
/* IC4 OUT1..OUT3 (global channels 9, 10, 11) as the general-purpose
 * outputs: GpOut1, GpOut2 and the GP PWM function. */
#define GP_AUX_CH1 9          // IC4 OUT1, J1.10 OUT1_DRV
#define GP_AUX_CH2 10         // IC4 OUT2, J1.11 OUT2_DRV
#define GP_AUX_PWM 11         // IC4 OUT3, J1.12 OUT3_DRV

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

/* Speed inputs: ADC1 regular scan IN0 (ISS), IN1 (OSS), IN2 (n6),
 * continuous, 13.5 + 12.5 = 26 ADC cycles per conversion at 12 MHz, so one
 * scan of 3 takes 6.5 us = 153.8 kS/s per channel (Bob asks >= 100 kS/s:
 * 8-10 samples per period at 10 kHz = 10000 rpm x 60 ppr). DMA1 channel 1
 * (the only ADC1 request on the F1) writes a circular buffer of
 * 2 x SPEED_HALF_SCANS scans; the half- and full-transfer interrupts hand
 * one half to ScanEdges (speedsens.h). The scan count is the time base.
 * DMA1 users: CH1 ADC1 (here), CH2 / CH3 USART3 terminal TX / RX. SPI1
 * (MAX22200, blocking) and UART4 (LIN, interrupt) use no DMA; SPI1's own
 * requests would be CH2 / CH3 and so must stay unused.
 */
#define SPEED_ADC ADC1
#define SPEED_ADC_RCC RCC_ADC1
#define SPEED_ADC_CH_IN 0
#define SPEED_ADC_CH_OUT 1
#define SPEED_ADC_CH_AUX 2
#define SPEED_NUM 3
#define SPEED_SMP ADC_SMPR_SMP_13DOT5CYC
#define SPEED_TIMER_HZ 153846u // scans per second (12 MHz / 78), the time base
#define SPEED_HALF_SCANS 64    // scans per DMA half buffer (416 us)
#define SPEED_DMA DMA1
#define SPEED_DMA_CH DMA_CHANNEL1
#define SPEED_DMA_IRQ NVIC_DMA1_CHANNEL1_IRQ
/* Slow analogue inputs on ADC2: injected group IN10 vbat, IN11
 * TempIn1, IN12 TempIn2, IN13 ana_in, plus IN8 board NTC as a regular
 * conversion, all software started every 10 ms with 239.5-cycle samples.
 * The die sensor (IN16) exists on ADC1 only: one injected conversion every
 * 100 ms, which pauses the speed scan for about 21 us (< half a period at
 * 10 kHz; 0.02 % on the time base). */
#define SLOW_ADC ADC2
#define SLOW_ADC_RCC RCC_ADC2

/* LIN for the aux oil pump: UART4, TX PC10 / RX PC11, transceiver SLP_N
 * on PD11 (lin_nslp, high = awake). TJA1021 (U7, master termination
 * 1k + diode). Only the 330e needs it, so the
 * driver is still off by default (LinPumpEnable = 0) and U7 sleeps.
 */
#define LIN_USART UART4
#define LIN_USART_RCC RCC_UART4
#define LIN_USART_IRQ NVIC_UART4_IRQ
#define LIN_BAUD 19200 // baud

#endif // HWDEFS_H_INCLUDED
