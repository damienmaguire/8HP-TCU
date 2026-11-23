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
#ifndef DIAG_H
#define DIAG_H

/* Per-channel and system fault detection. These functions only classify;
 * what the TCU then does is the existing limp / hold logic (README
 * "Fault detection"):
 *  - a pressure valve open / short / out of band  -> safe state (all coils
 *    off = mechanical 6th gear with full line pressure), as any driver
 *    fault;
 *  - TCC lock-up slipping                         -> TCC opened, reported;
 *  - speed sensor fault                           -> the speed check
 *    (TF_SPEED) and the existing gear-hold rules;
 *  - frozen torque                                -> torque substitute
 *    (high clamping pressure);
 *  - CAN timeout                                  -> COMM_CANLOSS_MODE;
 *  - firmware / calibration mismatch at boot      -> configuration error,
 *    safe state.
 *
 * Valve faults come from the MAX22200 status flags [C, data sheet]:
 *   OLF (open load)          -> open circuit
 *   OCP (over-current)       -> short to battery (the low-side output is
 *                               pulled to the supply) or a shorted coil
 *   HHF (hit current not reached) -> resistance too high / supply low
 * A short to ground on a low-side output keeps the coil on and is not a
 * MAX22200 flag; it shows only in the resistance check (below 1 ohm),
 * which needs a coil current read-back: the board has none, so the check
 * is not fed (README). */

#include <stdint.h>

enum DiagCode
{
   DIAG_OK = 0,
   DIAG_OPEN = 1,       // open circuit
   DIAG_SHORT_BAT = 2,  // short to battery / over-current
   DIAG_SHORT_GND = 3,  // short to ground (resistance < 1 ohm)
   DIAG_R_HIGH = 4,     // resistance above the band (or hit not reached)
   DIAG_R_LOW = 5,      // resistance below the band
};

enum SpeedDiag
{
   SPD_OK = 0,
   SPD_OPEN = 1,     // level stuck high: sensor or wire open
   SPD_SHORT = 2,    // level stuck low: short to ground
   SPD_NOSIGNAL = 3, // no edges while the other shaft shows the car moving
};

/* Coil resistance reference [C, 330e A2L]: OutMRstn_resActr20_C = 5.15 ohm
 * at 20 degC, OutMRstn_resRmn_C about 0.15 ohm wiring / contacts. Copper
 * coil: +0.393 % per degC. */
#define DIAG_R20_MOHM 5150 // milliohm, coil at 20 degC
#define DIAG_TC_PPM 3930 // ppm per degC (copper)
#define DIAG_RSHORT_MOHM 1000 // milliohm: below this = short to ground

namespace Diag
{
/// Expected coil + wiring resistance at a temperature, milliohm.
int32_t ExpectedMilliOhm(int32_t tempC, int32_t wireMilliOhm);
/** Resistance check from a measured voltage (mV) and current (mA).
 * bandPct: allowed deviation in % of the expected value. Returns a
 * DiagCode (DIAG_OK if the current is too small to judge). */
uint8_t Resistance(int32_t mV, int32_t mA, int32_t tempC, int32_t bandPct,
                   int32_t wireMilliOhm);
/** Code for a channel from its ValveOut fault bits (VALVE_F_*) and the
 * resistance result. The flags win; OCP before OLF before HHF. */
uint8_t Channel(uint16_t valveFlags, uint8_t rCheck);
/** Paired-channel check: the MAX22200 drives its channels in pairs
 * (OUT0/1, 2/3, 4/5, 6/7 share a half of the chip) [I]. Both channels
 * of a pair faulted at once points at the shared supply, ground or
 * harness rather than two coils. Bit k set = pair k (chip channels
 * 2k, 2k+1) faulted together. codes[] = DiagCode per global channel. */
uint8_t Pairs(const uint8_t *codes, int n);
/** Speed sensor from the 100 ms ADC window (min / max in mV), whether
 * edges arrive, and whether the other shaft shows the car moving.
 * openMv / shortMv = 0 switch the level checks off. */
uint8_t Speed(int32_t minMv, int32_t maxMv, bool edges, bool otherMoving,
              int32_t openMv, int32_t shortMv);
/// Calibration signature check at boot (expected 0 = do not check).
bool CalOk(uint32_t expected, uint32_t actual);
}

/* TCC lock-up slip: with the lock-up clutch commanded fully closed, the
 * engine (motor) speed and the turbine speed must agree. More than
 * maxRpm for longer than ms = slipping lock-up clutch. maxRpm = 0
 * switches it off: the OEM limit was not traced, so there is no default
 * (TBD). */
class SlipMonitor
{
public:
   SlipMonitor() : t(0), fault(false) {}
   bool Tick10ms(bool locked, int32_t nMot, int32_t nT, int32_t maxRpm, int32_t ms);
   bool Fault() const { return fault; }
   void Clear() { t = 0; fault = false; }

private:
   int32_t t;
   bool fault;
};

#endif // DIAG_H
