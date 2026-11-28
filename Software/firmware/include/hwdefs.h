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

/* One board: Bob's Stm32_TCU_V5 (replaces the V4 rev C/D layout).
 * STM32F107VCT6, 256 KiB flash in 2 KiB pages, 64 KiB RAM. Clock: 8 MHz
 * HSE -> 72 MHz SYSCLK, APB1 36 MHz, APB2 72 MHz (RCC_CLOCK_HSE8_72MHZ,
 * same as the ZombieVerter). Pin map: Bob's for_sam_v5_1349 pack
 * (pinmap CSVs, firmware_answers.md) and the netlist Stm32_TCU_V5. The
 * 56-pin header is J5 in the netlist (some of Bob's tables say J1).
 * See also digio_prj.h and README "Pin map".
 *
 * What the board has:
 *  - two TLE92466ED six-channel current-controlled low-side drivers on
 *    SPI1 (tle92466.h): IC10 (CSN PA4) and IC11 (CSN PE9). EN PE7 and
 *    RESN PE11 are shared and only have the chips' internal pull-downs;
 *    FAULTN PE10 (shared, open drain, 10k pull-up to 3.3 V). CLK and
 *    DRV0/1 are tied to GND, so every channel is SPI controlled and the
 *    chips run on their internal clock. TLE VDD 5 V, VIO 3.3 V, VBAT =
 *    switched 12 V.
 *      IC10: CH0 A (J5.8), CH1 B (J5.7), CH2 C (J5.6), CH3 D (J5.5),
 *            CH4 PHOLD (J5.9), CH5 PZYL (J5.42)
 *      IC11: CH0 E (J5.1), CH1 LINE (J5.2), CH2 PREL (J5.3), CH3 TCC
 *            (J5.4), CH4 / CH5 not connected
 *    The chips measure the coil currents and their own VBAT and
 *    temperature. There is no separate VBAT sense line: the board 12 V
 *    is read on PC0 (8.2k / 1.8k divider).
 *  - OUT1 PE8, OUT2 PE12, OUT3 PC6: NCV8402 low-side switches, active
 *    high. OUT1 / OUT2 plain GPIO (GpOut1Func / GpOut2Func), OUT3 PWM on
 *    TIM3_CH1 (full remap; GpPwmFreq / GpPwmDuty).
 *  - speed inputs on the ADC with a 10k / 1 nF RC: ISS PA0 (IN0), OSS PA1
 *    (IN1), n6 PA2 (IN2), scanned continuously by ADC1 into a circular
 *    DMA buffer (DMA1 channel 1); the slow analogue inputs are on ADC2
 *    (no DMA on F1 ADC2: software started, read every 10 ms).
 *  - 12 V inputs (ZV1.3 circuit: SMF16A TVS, 4.7k series, 1.5k to GND,
 *    1 uF; active high): T15 PD0, BRAKE_IN PD1 (EXTI1), SHIFT_UP PD2,
 *    SHIFT_DN PD3, D1 / D2 / PARK / NOT_PARK PD4..PD7, clutch input
 *    CL_IN on PA15 (J5.41, read only, no logic uses it yet).
 *  - CAN1 PA11 / PA12 = vehicle PT-CAN, CAN2 PB5 / PB6 (CAN2_REMAP) =
 *    ZombieVerter link, both 500 kbit/s.
 *  - LIN transceiver TJA1021 on UART4 PC10 / PC11, SLP_N PD11.
 *  - board NTC on PB0 (ADC12_IN8): 10k at 25 degC, B 3950 K, 10k
 *    pull-up to 3V3, NTC to GND.
 *  - free pins: PC7, PC8, PC9, PE13, PE14, PE15, PB2, PB3, PB4.
 */

/* Pins not in digio_prj.h's fixed list (Bob's pin map). */
#define TLE_CS1_PORT GPIOE
#define TLE_CS1_PIN GPIO9
#define SHIFT_UP_PORT GPIOD
#define SHIFT_UP_PIN GPIO2
#define SHIFT_DN_PORT GPIOD
#define SHIFT_DN_PIN GPIO3
#define T15_PORT GPIOD
#define T15_PIN GPIO0
#define BRAKE_IN_PORT GPIOD
#define BRAKE_IN_PIN GPIO1    // PD1, BRAKE_IN
#define BOARD_NTC_PORT GPIOB
#define BOARD_NTC_PIN 0       // PB0 = ADC12_IN8
#define BOARD_NTC_ADC_CH 8
/* PHOLD (park-hold magnet) on IC10 CH4: global channel 4. Fixed by the
 * board, so not a parameter. */
#define PHOLD_CH 4

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

/* TLE92466ED on SPI1, no remap: SCK PA5, MISO PA6 (1k pull-up to 3.3 V),
 * MOSI PA7. CSN is a software chip select: PA4 (tle_cs0, IC10) and PE9
 * (tle_cs1, IC11), digio_prj.h.
 */
#define TLE_SPI SPI1
#define TLE_SPI_RCC RCC_SPI1
#define TLE_SPI_PORT GPIOA
#define TLE_SPI_SCK GPIO5
#define TLE_SPI_MISO GPIO6
#define TLE_SPI_MOSI GPIO7

/* Speed inputs: ADC1 regular scan IN0 (ISS), IN1 (OSS), IN2 (n6),
 * continuous, 13.5 + 12.5 = 26 ADC cycles per conversion at 12 MHz, so one
 * scan of 3 takes 6.5 us = 153.8 kS/s per channel (Bob asks >= 100 kS/s:
 * 8-10 samples per period at 10 kHz = 10000 rpm x 60 ppr). DMA1 channel 1
 * (the only ADC1 request on the F1) writes a circular buffer of
 * 2 x SPEED_HALF_SCANS scans; the half- and full-transfer interrupts hand
 * one half to ScanEdges (speedsens.h). The scan count is the time base.
 * DMA1 users: CH1 ADC1 (here), CH2 / CH3 USART3 terminal TX / RX. SPI1
 * (TLE92466ED, blocking) and UART4 (LIN, interrupt) use no DMA; SPI1's own
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
 * SLP_N has only the chip's own pull-down, so U7 is asleep until the
 * firmware drives PD11 high; see LinWake (stm32_tcu.cpp, 330e only) for
 * the wake-up timing. R38 (2k7) pulls TXD up, and RXD has a 2k7 pull-up too.
 */
#define LIN_USART UART4
#define LIN_USART_RCC RCC_UART4
#define LIN_USART_IRQ NVIC_UART4_IRQ
#define LIN_BAUD 19200 // baud

#endif // HWDEFS_H_INCLUDED
