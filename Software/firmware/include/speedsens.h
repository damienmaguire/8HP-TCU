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

#include "anaconv.h" // AdcEdge
#include <stdint.h>

/* Hall speed sensor: edge periods -> rpm.
 *
 * Source: 330E_MECHATRONIC_PINOUT.md section 1. The OEM firmware is set
 * up for 60 pulses per revolution on all three channels, a 30 rpm minimum
 * and a 100 ms timeout [C], but public 8HP data says about 28 teeth
 * (input) and 40 (output) [H]. The tooth count is therefore a parameter
 * per sensor, to be set after a bench spin test (open question Q3).
 *
 * The sensor pins are scanned by ADC1 at SPEED_TIMER_HZ (hwdefs.h) and
 * ScanEdges below finds the edges; it calls Edge() with the number of
 * scans since the previous edge (32 bit, no wrap in practice).
 * Tick10ms() turns the edges seen in the last 10 ms into an average
 * speed. No edge for timeoutMs means 0 rpm.
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

/// OEM substitute temperature when the ATF sensor is faulty [C].
const int32_t ATF_SUBSTITUTE_C = 120;

/** Speed front end: ADC1 scans the speed pins back to back
 * (continuous scan, circular DMA), and the DMA half / full interrupts hand
 * over a block of scans. Block() walks one block, channel by channel,
 * finds the rising edges with AdcEdge (hysteresis) and gives each sensor
 * the period since its last edge in scans. The scan count is the time
 * base, so the sensors are configured with the scan rate as timerHz.
 * Buffer layout: buf[scan * nch + ch]. Also keeps the lowest and highest
 * level seen per channel for the sensor diagnosis (mV levels).
 */
#define SCAN_MAX_CH 3
class ScanEdges
{
public:
   ScanEdges();
   AdcEdge edge[SCAN_MAX_CH];
   /// Process `scans` scans of `nch` channels; sens[ch] gets the edges.
   void Block(const volatile uint16_t *buf, int scans, int nch, SpeedSensor *const *sens);
   uint32_t Scans() const { return n; }
   uint16_t Min(int ch) const { return mn[ch]; }
   uint16_t Max(int ch) const { return mx[ch]; }
   /// Start a new min / max window (100 ms task).
   void ResetLevels();

private:
   uint32_t n;                 // scans processed so far
   uint32_t last[SCAN_MAX_CH]; // scan count at the last edge per channel
   uint16_t mn[SCAN_MAX_CH], mx[SCAN_MAX_CH];
};

#endif // SPEEDSENS_H
