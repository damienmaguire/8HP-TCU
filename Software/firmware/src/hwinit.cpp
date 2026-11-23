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
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>

/* 8 MHz crystal -> 72 MHz, as on the ZombieVerter. */
void clock_setup(void) {
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6); // 12 MHz ADC clock

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
  rcc_periph_clock_enable(LIN_USART_RCC);
  rcc_periph_clock_enable(MAX_SPI_RCC);
  rcc_periph_clock_enable(RCC_DMA1); // ADC1 and terminal UART
  rcc_periph_clock_enable(RCC_ADC1);
  rcc_periph_clock_enable(RCC_ADC2);
  rcc_periph_clock_enable(RCC_CAN1);
  rcc_periph_clock_enable(RCC_CAN2); // CAN2 needs the CAN1 clock as well
  // CRC unit: libopeninv checks the stored parameters and CAN map with
  // it. Without its clock it reads as 0, so nothing stored ever loads (the
  // ZombieVerter turns it on here too).
  rcc_periph_clock_enable(RCC_CRC);
  // Power and backup domain: the RTC and the backup register that keeps
  // the start-up stage across a watchdog reset (see BootStage).
  rcc_periph_clock_enable(RCC_PWR);
  rcc_periph_clock_enable(RCC_BKP);
}

/* 1 s RTC tick, used by libopeninv for CAN timestamps. */
void rtc_setup(void) {
  // HSE/128 = 62.5 kHz, / 62500 = 1 Hz
  rtc_auto_awake(RCC_HSE, 62499);
  rtc_set_counter_val(0);
}

/* AFIO: SWD only (JTAG pins freed). CAN1 stays on PA11/PA12 (no remap).
 * CAN2 is remapped to PB5 RX / PB6 TX, where its transceiver is.
 */
void afio_setup(void) {
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN2_REMAP);
}

// Interrupt priorities and enables (CAN, LIN UART, speed ADC, scheduler timer).
void nvic_setup(void) {
  nvic_enable_irq(NVIC_TIM4_IRQ); // scheduler, highest priority
  nvic_set_priority(NVIC_TIM4_IRQ, 0);

  // Speed sensor samples: same priority as the scheduler, so neither can
  // interrupt the other and the edge counters stay consistent.
  nvic_enable_irq(SPEED_DMA_IRQ); // DMA half / full buffer
  nvic_set_priority(SPEED_DMA_IRQ, 0);

  // LIN bytes: lower priority, 19200 baud leaves 520 us per byte.
  nvic_enable_irq(LIN_USART_IRQ);
  nvic_set_priority(LIN_USART_IRQ, 2 << 4);
}

// SPI1 for the MAX22200 valve drivers (mode 0, MSB first).
void max_spi_setup(void) {
  gpio_set_mode(MAX_SPI_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, MAX_SPI_SCK | MAX_SPI_MOSI);
  gpio_set_mode(MAX_SPI_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                MAX_SPI_MISO);
  spi_reset(MAX_SPI);
  /* MAX22200 data sheet, "SPI Detailed Description": mode 0, MSB first,
   * up to 10 MHz for a single chip. SPI1 is on APB2 (72 MHz): /16 =
   * 4.5 MHz.
   */
  spi_init_master(MAX_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_16,
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT,
                  SPI_CR1_MSBFIRST);
  spi_enable_software_slave_management(MAX_SPI);
  spi_set_nss_high(MAX_SPI);
  spi_enable(MAX_SPI);
}

// Send and receive one SPI byte (blocking, a few microseconds).
uint8_t max_spi_transfer(uint8_t tx) {
  return (uint8_t)spi_xfer(MAX_SPI, tx);
}

/* Speed inputs: ADC1 regular group IN0, IN1, IN2 in continuous scan
 * mode, DMA1 channel 1 circular into buf (2 x SPEED_HALF_SCANS scans of
 * SPEED_NUM samples), half- and full-transfer interrupts on. 13.5 + 12.5
 * cycles per conversion: one scan every 6.5 us (153.8 kS/s per input).
 * Source impedance: the 10k of the RC is above the data sheet RAIN for
 * 13.5 cycles, but the 1 nF holds the pin and the 8 pF sample capacitor
 * takes under 1 % of its charge, a few mV at the 1.05 V threshold, which
 * the AdcEdge hysteresis covers. ADC1 injected IN16 (die sensor,
 * 239.5 cycles) is also set up here, software started by SlowAdc.
 */
void speed_adc_setup(volatile uint16_t *buf) {
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG,
                GPIO0 | GPIO1 | GPIO2);
  adc_power_off(SPEED_ADC);
  adc_enable_scan_mode(SPEED_ADC);
  adc_set_continuous_conversion_mode(SPEED_ADC);
  adc_set_right_aligned(SPEED_ADC);
  adc_set_sample_time(SPEED_ADC, SPEED_ADC_CH_IN, SPEED_SMP);
  adc_set_sample_time(SPEED_ADC, SPEED_ADC_CH_OUT, SPEED_SMP);
  adc_set_sample_time(SPEED_ADC, SPEED_ADC_CH_AUX, SPEED_SMP);
  adc_set_sample_time(SPEED_ADC, ADC_CHANNEL_TEMP, ADC_SMPR_SMP_239DOT5CYC);
  uint8_t ch[SPEED_NUM] = {SPEED_ADC_CH_IN, SPEED_ADC_CH_OUT, SPEED_ADC_CH_AUX};
  adc_set_regular_sequence(SPEED_ADC, SPEED_NUM, ch);
  uint8_t tch[1] = {ADC_CHANNEL_TEMP};
  adc_set_injected_sequence(SPEED_ADC, 1, tch);
  adc_enable_external_trigger_regular(SPEED_ADC, ADC_CR2_EXTSEL_SWSTART);
  adc_enable_external_trigger_injected(SPEED_ADC, ADC_CR2_JEXTSEL_JSWSTART);
  adc_enable_temperature_sensor();
  adc_enable_dma(SPEED_ADC);
  adc_power_on(SPEED_ADC);
  for (volatile int i = 0; i < 800; i++) {
  } // tSTAB
  adc_reset_calibration(SPEED_ADC);
  adc_calibrate(SPEED_ADC);

  dma_channel_reset(SPEED_DMA, SPEED_DMA_CH);
  dma_set_peripheral_address(SPEED_DMA, SPEED_DMA_CH, (uint32_t)&ADC_DR(SPEED_ADC));
  dma_set_memory_address(SPEED_DMA, SPEED_DMA_CH, (uint32_t)buf);
  dma_set_number_of_data(SPEED_DMA, SPEED_DMA_CH, 2 * SPEED_HALF_SCANS * SPEED_NUM);
  dma_set_read_from_peripheral(SPEED_DMA, SPEED_DMA_CH);
  dma_enable_memory_increment_mode(SPEED_DMA, SPEED_DMA_CH);
  dma_set_peripheral_size(SPEED_DMA, SPEED_DMA_CH, DMA_CCR_PSIZE_16BIT);
  dma_set_memory_size(SPEED_DMA, SPEED_DMA_CH, DMA_CCR_MSIZE_16BIT);
  dma_enable_circular_mode(SPEED_DMA, SPEED_DMA_CH);
  dma_set_priority(SPEED_DMA, SPEED_DMA_CH, DMA_CCR_PL_VERY_HIGH);
  dma_enable_half_transfer_interrupt(SPEED_DMA, SPEED_DMA_CH);
  dma_enable_transfer_complete_interrupt(SPEED_DMA, SPEED_DMA_CH);
  dma_enable_channel(SPEED_DMA, SPEED_DMA_CH);
  adc_start_conversion_regular(SPEED_ADC); // runs from here on
}

/* LIN on UART4: PC10 TX (AF push-pull), PC11 RX, 19200 8N1, LIN mode
 * (LINEN, break generation with SBK). RXNE interrupt on; TXE is turned on
 * per frame by the driver.
 */
void lin_usart_setup(void) {
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO10);
  gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11);
  usart_set_baudrate(LIN_USART, LIN_BAUD);
  usart_set_databits(LIN_USART, 8);
  usart_set_stopbits(LIN_USART, USART_STOPBITS_1);
  usart_set_parity(LIN_USART, USART_PARITY_NONE);
  usart_set_flow_control(LIN_USART, USART_FLOWCONTROL_NONE);
  usart_set_mode(LIN_USART, USART_MODE_TX_RX);
  USART_CR2(LIN_USART) |= USART_CR2_LINEN; // LIN mode, 10 bit break detect
  usart_enable_rx_interrupt(LIN_USART);
  usart_enable(LIN_USART);
}
