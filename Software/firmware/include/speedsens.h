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
#ifndef SPEEDSENS_H
#define SPEEDSENS_H

#include <stdint.h>

/* Hall speed sensor: edge periods from a timer input capture -> rpm.
 *
 * Source: 330E_MECHATRONIC_PINOUT.md section 1. The OEM firmware is set
 * up for 60 pulses per revolution on all three channels, a 30 rpm minimum
 * and a 100 ms timeout [C], but public 8HP data says about 28 teeth
 * (input) and 40 (output) [H]. The tooth count is therefore a parameter
 * per sensor, to be set after a bench spin test (open question Q3).
 *
 * The capture ISR calls Edge() with the timer ticks since the previous
 * edge. Tick10ms() turns the edges seen in the last 10 ms into an average
 * speed. No edge for timeoutMs means 0 rpm. The timeout must be shorter
 * than one wrap of the capture timer (131 ms at 500 kHz, 16 bit) so a
 * wrapped period is never used.
 */
class SpeedSensor
{
public:
   SpeedSensor();
   void Configure(uint32_t timerHz, uint16_t teeth, uint16_t timeoutMs,
                  uint16_t minRpm);
   /// From the capture ISR: timer ticks between this edge and the last.
   void Edge(uint32_t periodTicks);
   /// Every 10 ms: update the speed.
   void Tick10ms();
   int32_t Rpm() const { return rpm; }
   /// True while edges are arriving (false after a timeout).
   bool Valid() const { return valid; }

   /// rpm from an average period: 60 * f / (period * teeth).
   static int32_t PeriodToRpm(uint32_t timerHz, uint32_t periodTicks,
                              uint16_t teeth);

private:
   uint32_t timerHz;
   uint16_t teeth;
   uint16_t timeoutMs;
   uint16_t minRpm;
   volatile uint32_t sumTicks;
   volatile uint32_t count;
   volatile bool edgeSeen;
   uint32_t msSinceEdge;
   int32_t rpm;
   bool valid;
};

/* ATF temperature from the ADC through a parameter table.
 * The sensor curve is not known yet (open question Q4): the OEM converts
 * through an internal divider whose values are unknown. Until it is
 * measured, the TCU can run on a fixed temperature (AtfSource = fixed).
 */
namespace AtfTemp
{
/// Interpolate adc -> degC on an ascending-adc table of n points.
int32_t FromAdc(int32_t adc, const int32_t *adcPts, const int32_t *tempPts,
                int n);
/// OEM substitute temperature when the sensor is faulty [C].
const int32_t SUBSTITUTE_C = 120;
} // namespace AtfTemp

#endif // SPEEDSENS_H
