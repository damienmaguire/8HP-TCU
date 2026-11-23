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
#include "hwinit.h"
#include "hwdefs.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>

/* 8 MHz crystal -> 72 MHz, as on the ZombieVerter. Crystal TBD. */
void clock_setup(void) {
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

  // 16 pre-emption priorities, no sub-priorities.
  SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_GPIOD);
  rcc_periph_clock_enable(RCC_GPIOE);
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_USART3); // terminal
  rcc_periph_clock_enable(RCC_TIM4);   // scheduler
  rcc_periph_clock_enable(SPEED_TIMER_RCC);
  rcc_periph_clock_enable(MAX_SPI_RCC);
  rcc_periph_clock_enable(RCC_DMA1); // ADC and UART
  rcc_periph_clock_enable(RCC_ADC1);
  rcc_periph_clock_enable(RCC_CAN1);
  rcc_periph_clock_enable(RCC_CAN2);
}

/* 1 s RTC tick, used by libopeninv for CAN timestamps. */
void rtc_setup(void) {
  // HSE/128 = 62.5 kHz, / 62500 = 1 Hz
  rtc_auto_awake(RCC_HSE, 62499);
  rtc_set_counter_val(0);
}

void nvic_setup(void) {
  nvic_enable_irq(NVIC_TIM4_IRQ); // scheduler, highest priority
  nvic_set_priority(NVIC_TIM4_IRQ, 0);

  // Speed sensor edges: same priority as the scheduler, so neither can
  // interrupt the other and the edge counters stay consistent.
  // Placeholder lines 7..9 share EXTI9_5 (see digio_prj.h).
  nvic_enable_irq(NVIC_EXTI9_5_IRQ);
  nvic_set_priority(NVIC_EXTI9_5_IRQ, 0);
}

void max_spi_setup(void) {
  gpio_set_mode(MAX_SPI_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, MAX_SPI_SCK | MAX_SPI_MOSI);
  gpio_set_mode(MAX_SPI_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                MAX_SPI_MISO);
  spi_reset(MAX_SPI);
  /* MAX22200 data sheet, "SPI Detailed Description": mode 0, MSB first,
   * up to 10 MHz for a single chip (5 MHz daisy-chained). SPI1 is on
   * APB2 (72 MHz): /16 = 4.5 MHz, inside both limits.
   */
  spi_init_master(MAX_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_16,
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT,
                  SPI_CR1_MSBFIRST);
  spi_enable_software_slave_management(MAX_SPI);
  spi_set_nss_high(MAX_SPI);
  spi_enable(MAX_SPI);
}

uint8_t max_spi_transfer(uint8_t tx) {
  return (uint8_t)spi_xfer(MAX_SPI, tx);
}

void speed_timer_setup(void) {
  timer_set_mode(SPEED_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  // APB1 timer clock is 72 MHz (APB1 prescaler 2 doubles it).
  timer_set_prescaler(SPEED_TIMER, (72000000u / SPEED_TIMER_HZ) - 1);
  timer_set_period(SPEED_TIMER, 0xFFFF);
  timer_enable_counter(SPEED_TIMER);
}

void speed_exti_setup(void) {
  /* Placeholder pins PE7, PE8, PE9 (TBD, see digio_prj.h). */
  exti_select_source(EXTI7 | EXTI8 | EXTI9, GPIOE);
  exti_set_trigger(EXTI7 | EXTI8 | EXTI9, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI7 | EXTI8 | EXTI9);
}
