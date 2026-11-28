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
#ifndef TLE92466_H
#define TLE92466_H

#include <stdint.h>

/* Driver for the Infineon TLE92466ED six-channel low-side solenoid driver.
 * The V5 board has two of them on SPI1 (IC10 and IC11).
 *
 * Every register address, bit position and formula below comes from the
 * TLE92466ED datasheet Rev. 1.2 (2022-02-01):
 *  - 5.1.2 CRC (CRC-8 SAE J1850, polynomial 0x1D, start 0xFF, result
 *    XOR 0xFF, bytes taken in the order frame[7:0], [15:8], [23:16])
 *  - 5.2.1 data flow: the answer to a frame comes back during the NEXT
 *    frame
 *  - 5.2.3 frame layouts: write = CRC | 7-bit address | R/W=1 | data;
 *    read = CRC | R/W=0 | 16-bit address; replies are 16-bit (with a
 *    status field), 22-bit (feedback registers) or the critical fault frame
 *  - Table 19 / 24 / 25 register maps; channel registers sit at a channel
 *    base address (CH0 0x40, CH1 0x50, CH2 0x60, CH3 0x70, CH4 0x20,
 *    CH5 0x30) plus the offset; the read-only feedback registers of a
 *    channel are at base + 0x200 (FB_I_AVG of CH0 = 0x0242)
 *  - 4.3 operating states: after reset the chip is in Config Mode (all
 *    channels off); OP_MODE = 1 in CH_CTRL enters Mission Mode, and only
 *    then can the channel enable bits be set
 *  - 4.9 diagnosis: an open load / short to ground / over-current found by
 *    the chip switches that channel off by itself until the flag is
 *    cleared ("clear on write") and the enable bit is set again
 *
 * What it does for the TCU:
 *  - every used channel runs in current control (ICC): the chip regulates
 *    the AVERAGE coil current to the setpoint and adds the dither itself,
 *    so the firmware dither of the old board is gone. The dither amplitude
 *    can be changed with the current (DITHER_CTRL is written when it
 *    changes); the period stays at TLE_DITHER_MS;
 *  - it reads back the measured average current of each channel;
 *  - it reads and clears the chip's fault flags.
 * It never pulls EN or RESN: those pins belong to the board code, which
 * follows the park-lock safety rule (parksafe.h).
 *
 * The chip timing is fSYS = 28 MHz (datasheet Table 8, internal clock).
 * Not verified on hardware yet.
 */

#define TLE_CHANNELS 6
#define TLE_FSYS_HZ 28000000u // Hz, internal system clock

// Central registers (Table 19)
#define TLE_CH_CTRL 0x0000
#define TLE_GLOBAL_CONFIG 0x0002
#define TLE_GLOBAL_DIAG0 0x0003
#define TLE_GLOBAL_DIAG1 0x0004
#define TLE_GLOBAL_DIAG2 0x0005
#define TLE_DIAG_ERR_CHGR(x) (0x000A + (x)) // x = 0..2, channels 2x and 2x+1
#define TLE_DIAG_WARN_CHGR(x) (0x0010 + (x))
#define TLE_FAULT_MASK0 0x0016
#define TLE_FAULT_MASK1 0x0017
#define TLE_FAULT_MASK2 0x0018
#define TLE_ICVID 0x0200
#define TLE_PIN_STAT 0x0201
#define TLE_FB_STAT 0x0202
#define TLE_FB_VOLTAGE1 0x0203
#define TLE_FB_VOLTAGE2 0x0204

// Channel register offsets (Table 24), added to the channel base
#define TLE_SETPOINT 0x0000
#define TLE_CTRL 0x0001
#define TLE_PERIOD 0x0002
#define TLE_DITHER_CLK_DIV 0x0004
#define TLE_DITHER_STEP 0x0005
#define TLE_DITHER_CTRL 0x0006
#define TLE_CH_CONFIG 0x0007
#define TLE_MODE 0x000C
#define TLE_FB_DC 0x0200
#define TLE_FB_VBAT 0x0201
#define TLE_FB_I_AVG 0x0202

// CH_CTRL (5.3.2.3)
#define TLE_OP_MODE_MISSION 0x8000
// GLOBAL_CONFIG (5.3.2.4); reset value 0x4005
#define TLE_GC_CLK_WD_EN 0x0001
#define TLE_GC_SPI_WD_EN 0x0002
#define TLE_GC_CRC_EN 0x0004
#define TLE_GC_VIO_5V 0x4000
// FB_STAT (5.3.2.21), a 22-bit register
#define TLE_FS_CLK_NOK (1ul << 6)
#define TLE_FS_COTERR (1ul << 7)
#define TLE_FS_COTWARN (1ul << 8)
#define TLE_FS_RES_EVENT (1ul << 9)
#define TLE_FS_POR_EVENT (1ul << 10)
#define TLE_FS_DATA_ERR (1ul << 11)
#define TLE_FS_SUP_NOK_EXT (1ul << 12)
#define TLE_FS_SUP_NOK_INT (1ul << 13)
#define TLE_FS_SPI_WD_ERR (1ul << 20)
#define TLE_FS_INIT_DONE (1ul << 21)
// DIAG_ERR_CHGRx (5.3.2.12): bits for the even channel, shifted by 8 for the odd one
#define TLE_DE_OLSG 0x01
#define TLE_DE_OL 0x02
#define TLE_DE_OC 0x04
#define TLE_DE_SG 0x08
#define TLE_DE_OTE 0x10
// MODE (5.3.3.11)
#define TLE_MODE_OFF 0x0
#define TLE_MODE_ICC 0x1
// Reply modes (5.2.3.3)
#define TLE_REPLY_16 0
#define TLE_REPLY_22 1
#define TLE_REPLY_CRITICAL 2

/// Largest setpoint the driver sends: 1.5 A, the datasheet's setpoint
/// limit for one channel (codes above 0x6000 saturate in the chip).
#define TLE_IMAX_0P1MA 15000

/// One decoded reply frame.
struct TleReply
{
   uint8_t mode;   // TLE_REPLY_16, _22 or _CRITICAL
   uint8_t status; // 16-bit replies only: 0 = no error (5.2.3.3)
   bool rw;        // R/W bit of the previous frame (16-bit replies)
   uint32_t data;  // 16 or 22 data bits
   bool crcOk;     // CRC matched (the critical fault frame carries none)
};

/// Dither register values for one channel (5.3.3.7 .. 5.3.3.9).
struct TleDither
{
   uint16_t clkDiv;
   uint16_t step;
   uint16_t ctrl;
};

/// SPI wiring of the chips. Select() drives the chip select (and keeps
/// the datasheet gaps, see hwinit); Transfer16() clocks 16 bits in mode 1.
class Tle92466Bus
{
public:
   virtual void Select(uint8_t dev, bool active) = 0;
   virtual uint16_t Transfer16(uint16_t tx) = 0;
};

/// Dither period: 8 ms = 125 Hz, the OEM value (Ian, V5_VALVE_DEFAULTS
/// [C]). The chip's dither is a triangle. The current read-back is
/// averaged over one dither period, so every channel gets this clock.
#define TLE_DITHER_MS 8

struct TleConfig
{
   uint8_t usedMask;      // channels with a coil; the others stay off
   uint8_t olMask;        // channels where the chip's own open-load / short
                          // check may run (it switches the channel off!)
   uint8_t olRelEighths;  // open-load threshold, 0..7 eighths of the setpoint
   uint16_t pwmHz;        // current-control chopping target, Hz
   uint8_t slew;          // output slew rate, 0..3 = 1, 2.5, 5, 10 V/us
};

/// Fault state of one chip from ReadDiag().
struct TleDiag
{
   uint32_t fbStat;   // FB_STAT (22 bits)
   uint16_t diag0;    // GLOBAL_DIAG0
   uint8_t ol, sg, oc, olsg, ote; // bit per channel
   uint8_t enabled;   // channel enable bits the chip reports
   bool mission;      // chip is in Mission Mode
   bool critical;     // chip answers with the critical fault frame
   bool commOk;       // every reply had a good CRC and status
};

namespace Tle
{
/// CRC-8 SAE J1850 over frame bits 23:0, as the datasheet orders the bytes.
uint8_t Crc8(uint32_t frame);
/// Write frame for a register (only addresses below 0x80 are writable).
uint32_t WriteFrame(uint16_t addr, uint16_t data);
/// Read frame for a register.
uint32_t ReadFrame(uint16_t addr);
/// Decode a reply frame.
TleReply Parse(uint32_t frame);
/// Address of a channel register (offset = TLE_SETPOINT ... TLE_FB_I_AVG).
uint16_t ChAddr(uint8_t ch, uint16_t offset);
/// SETPOINT code for a current in 0.1 mA (2 A x code / 32767), limited to 1.5 A.
uint16_t SetpointCode(int32_t i_0p1mA);
/// Average current in 0.1 mA from FB_I_AVG and FB_DC (4 A x I_AVG_MANT / TP_MANT).
int32_t AvgCurrent(uint32_t fbIAvg, uint32_t fbDc);
/** Coil resistance estimate in milliohm from one channel's FB_DC,
 * FB_VBAT and FB_I_AVG (all over the same measurement period):
 * R = VBAT x duty / I_avg, with VBAT = 41.47 V x VBAT_AVG_MANT / TP_MANT,
 * duty = TO_MANT / TP_MANT and I = 4 A x I_AVG_MANT / TP_MANT. It
 * includes the wiring, connector and the switch, and is a little high
 * because the coil voltage during the off-time (freewheeling) is ignored.
 * 0 if there is no current. */
int32_t CoilMilliOhm(uint32_t fbDc, uint32_t fbVbat, uint32_t fbIAvg);
/// PERIOD register for a chopping frequency (0 = controller off).
uint16_t PeriodReg(uint32_t hz);
/// Triangle dither registers for a peak amplitude (0.1 mA) and period (ms).
TleDither DitherRegs(int32_t amp_0p1mA, int32_t periodMs);
/// CH_CONFIG for one channel: slew rate, and the chip's open-load / short
/// check on (relative threshold, OFF-state check) or completely off.
uint16_t ChConfig(uint8_t slew, bool olCheck, uint8_t olRelEighths);
/// Chip temperature in degC from FB_VOLTAGE2.
int32_t TempC(uint32_t fbVoltage2);
/// VBAT in mV from FB_VOLTAGE2.
int32_t VbatmV(uint32_t fbVoltage2);
}

class Tle92466
{
public:
   Tle92466(Tle92466Bus &b, uint8_t dev);
   /** Configure the chip from Config Mode and enter Mission Mode with every
    * used channel enabled at 0 mA. Returns false if the chip does not
    * answer, never finishes its own start-up or refuses Mission Mode.
    * Safe to call again at any time: it never switches EN or RESN, and the
    * setpoints are written again by the next Flush(). */
   bool Init(const TleConfig &c);
   /// Wanted current and dither amplitude (peak) for a channel, 0.1 mA.
   /// Sent by Flush().
   void SetCurrent(uint8_t ch, int32_t i_0p1mA, int32_t dither_0p1mA = 0);
   int32_t Wanted(uint8_t ch) const { return ch < TLE_CHANNELS ? want[ch] : 0; }
   /// Write the setpoints that changed (and all of them after Init()).
   void Flush();
   /** Read the fault state, then clear the channel fault flags so a channel
    * the chip switched off can run again, and set the enable bits of used
    * channels again if the chip dropped one. */
   bool ReadDiag(TleDiag *d);
   /// Read the measured average current (and the duty cycle and VBAT for
   /// the resistance estimate) of every used channel.
   void ReadCurrents();
   /// Last measured average current, 0.1 mA.
   int32_t Current(uint8_t ch) const { return ch < TLE_CHANNELS ? meas[ch] : 0; }
   /// Coil resistance estimate of the last ReadCurrents(), milliohm (0 = none).
   int32_t CoilMilliOhm(uint8_t ch) const { return ch < TLE_CHANNELS ? rCoil[ch] : 0; }
   /// Chip temperature (degC) and VBAT (mV) from the last ReadCurrents().
   int32_t TempC() const { return tempC; }
   int32_t VbatmV() const { return vbatmV; }
   /// Replies with a bad CRC, a status error or a critical fault frame.
   uint32_t CommErrors() const { return commErrors; }
   const TleConfig &Config() const { return cfg; }

   /** Send n frames and return the n replies (reply i answers frame i).
    * One extra read frame is clocked at the end to collect the last
    * answer. Returns false if any reply was bad. Public for the tests. */
   bool Exchange(const uint32_t *frames, int n, uint32_t *replies);

private:
   uint32_t Xfer(uint32_t frame);
   bool Write(uint16_t addr, uint16_t data);
   bool Read(uint16_t addr, uint32_t *data);

   Tle92466Bus &bus;
   uint8_t dev;
   TleConfig cfg;
   int32_t want[TLE_CHANNELS];
   int32_t sent[TLE_CHANNELS]; // -1 = must be sent
   int32_t wantDith[TLE_CHANNELS];
   int32_t sentDith[TLE_CHANNELS]; // DITHER_CTRL code sent, -1 = must be sent
   int32_t meas[TLE_CHANNELS];
   int32_t rCoil[TLE_CHANNELS];
   int32_t tempC;
   int32_t vbatmV;
   uint32_t commErrors;
   bool up;
};

#endif // TLE92466_H
