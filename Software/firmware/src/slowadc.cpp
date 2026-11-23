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
#include "slowadc.h"
#include "hwdefs.h"
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>

#if HW_SPEED_DMA
/* Rev B: ADC2 for the slow inputs, ADC1 injected for the die sensor. */
static uint16_t avg[SlowAdc::COUNT]; // x 4 (sum of the last 4, running)
static uint8_t seeded = 0; // bit per channel: first reading taken
static uint8_t tick = 0;

// Running average over about 4 readings, kept as a x4 sum.
static void Put(SlowAdc::Ch ch, uint16_t v) {
  if (!(seeded & (1u << ch))) {
    avg[ch] = (uint16_t)(v * 4);
    seeded |= (uint8_t)(1u << ch);
  } else {
    avg[ch] = (uint16_t)(avg[ch] - avg[ch] / 4 + v);
  }
}

void SlowAdc::Start() {
  gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG,
                GPIO0 | GPIO1 | GPIO2 | GPIO3);
  gpio_set_mode(BOARD_NTC_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG,
                1u << BOARD_NTC_PIN);
  adc_power_off(SLOW_ADC);
  adc_enable_scan_mode(SLOW_ADC); // injected group of 4
  adc_set_single_conversion_mode(SLOW_ADC);
  adc_set_right_aligned(SLOW_ADC);
  adc_set_sample_time_on_all_channels(SLOW_ADC, ADC_SMPR_SMP_239DOT5CYC);
  uint8_t inj[4] = {10, 11, 12, 13};
  adc_set_injected_sequence(SLOW_ADC, 4, inj);
  uint8_t reg[1] = {BOARD_NTC_ADC_CH};
  adc_set_regular_sequence(SLOW_ADC, 1, reg);
  adc_enable_external_trigger_injected(SLOW_ADC, ADC_CR2_JEXTSEL_JSWSTART);
  adc_enable_external_trigger_regular(SLOW_ADC, ADC_CR2_EXTSEL_SWSTART);
  adc_power_on(SLOW_ADC);
  for (volatile int i = 0; i < 800; i++) {
  } // tSTAB
  adc_reset_calibration(SLOW_ADC);
  adc_calibrate(SLOW_ADC);
  adc_start_conversion_injected(SLOW_ADC);
  adc_start_conversion_regular(SLOW_ADC);
  // The ADC1 side (die sensor, injected IN16) is set up together with
  // the speed scan in speed_adc_setup(), the first reading started here.
  adc_start_conversion_injected(SPEED_ADC);
}

/* 4 injected x 252 cycles + 252 = 105 us at 12 MHz, well inside 10 ms,
 * so the results are always there when the next tick comes. */
void SlowAdc::Tick10ms() {
  if (ADC_SR(SLOW_ADC) & ADC_SR_JEOC) {
    ADC_SR(SLOW_ADC) &= ~ADC_SR_JEOC;
    Put(VBAT, (uint16_t)adc_read_injected(SLOW_ADC, 1));
    Put(ATF1, (uint16_t)adc_read_injected(SLOW_ADC, 2));
    Put(ATF2, (uint16_t)adc_read_injected(SLOW_ADC, 3));
    Put(ANA, (uint16_t)adc_read_injected(SLOW_ADC, 4));
    adc_start_conversion_injected(SLOW_ADC);
  }
  if (ADC_SR(SLOW_ADC) & ADC_SR_EOC) {
    Put(BOARD_NTC, (uint16_t)adc_read_regular(SLOW_ADC)); // clears EOC
    adc_start_conversion_regular(SLOW_ADC);
  }
  // Die sensor: every 100 ms (the injected conversion pauses the speed
  // scan for about 21 us). JEOC has no interrupt enabled on ADC1.
  if (++tick >= 10) {
    tick = 0;
    if (ADC_SR(SPEED_ADC) & ADC_SR_JEOC) {
      ADC_SR(SPEED_ADC) &= ~ADC_SR_JEOC;
      Put(MCU_TEMP, (uint16_t)adc_read_injected(SPEED_ADC, 1));
      adc_start_conversion_injected(SPEED_ADC);
    }
  }
}

uint16_t SlowAdc::Get(Ch ch) {
  return (uint16_t)(avg[ch] / 4);
}

#else
/* V4: libopeninv AnaIn (ADC1 scan + DMA1 channel 1). */
#include "anain.h"

void SlowAdc::Start() {
  ANA_IN_CONFIGURE(ANA_IN_LIST);
  AnaIn::Start();
}

void SlowAdc::Tick10ms() {}

uint16_t SlowAdc::Get(Ch ch) {
  switch (ch) {
  case VBAT:
    return AnaIn::vbat.Get();
  case ATF1:
    return AnaIn::atf_temp1.Get();
  case ATF2:
    return AnaIn::atf_temp2.Get();
  case ANA:
    return AnaIn::ana_in.Get();
  case MCU_TEMP:
    return AnaIn::mcu_temp.Get();
  default:
    return 0; // no board NTC on V4
  }
}
#endif
