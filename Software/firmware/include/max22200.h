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
#ifndef MAX22200_H
#define MAX22200_H

#include <stdint.h>

/* Driver for the Analog Devices (Maxim) MAX22200 / MAX22200A octal
 * serial-controlled solenoid driver, used on this board for the
 * current-controlled valves.
 *
 * All register numbers, bit positions and transfer formats below are from
 * the MAX22200/MAX22200A data sheet, Rev 1 (3/25), www.analog.com:
 *  - "SPI Detailed Description" and "Programming Procedure" (Figure 6)
 *  - Table 9 Command Register, Table 10 Status Register,
 *    Table 11 Configuration Register, Table 12 Fault Register
 *  - Table 4 "HIT and HOLD Current Setting (CDR)" and
 *    "Full-Scale Current Setting (IFS)"
 *  - Table 1 Chopping Frequency
 *  - "Clearing the FAULT Register" (selective clear on the MAX22200A)
 *  - Figures 10 to 13 (8 and 32 bit read and write timing)
 *
 * How the chip is used here:
 *  - Every valve channel runs in current-drive regulation (CDR), low side,
 *    against the common +12 V of the mechatronic. CDR is low-side only.
 *  - The chip regulates the *peak* current of each chopping cycle
 *    (I_TRIG = HOLD / 127 x IFS). The average coil current is lower by
 *    about half the ripple. See "Fixed-Frequency Current Control".
 *  - IFS is set by the RREF resistor on the IREF pin: IFS = 15 kV / RREF
 *    (HFS = 0) or 7.5 kV / RREF (HFS = 1). RREF is Bob's choice, so IFS is
 *    a parameter (Max22200IfsmA). The data sheet allows RREF 15k..100k.
 *  - The proportional valves need 50..850 mA, so HFS = 0 (full scale up to
 *    1 A) is the default. One HOLD step is then IFS / 127, about 7.9 mA at
 *    IFS = 1 A.
 *  - The chip has no dither generator (the data sheet has no dither
 *    function), so the 125 Hz dither is done in firmware by rewriting the
 *    HOLD byte from the 1 ms task (see eds.h). A HOLD-only update is a
 *    one-byte transfer after an 8-bit command, as the data sheet suggests
 *    for "on-the-fly" current changes.
 *  - HOLD code 127 means "low side fully on" with no regulation, which
 *    would put about 2.3 A through a 5 ohm coil. This driver never writes
 *    127: the largest code it sends is 126, and lower if MaxCode says so.
 *
 * Not verified on hardware yet. Bob's board decides the IFS resistor, chip
 * selects, the CMD/ENABLE/nFAULT wiring and whether chips are daisy-chained.
 * This driver assumes one chip select per chip (no daisy chain).
 */

#define MAX22200_CHANNELS 8

// Register addresses (A_BNK field of the command byte)
#define MAX22200_REG_STATUS 0x00
#define MAX22200_REG_CFG_CH(ch) (0x01 + (ch))
#define MAX22200_REG_FAULT 0x09
#define MAX22200_REG_CFG_DPM 0x0A

// Command byte (Table 9): bit 7 RB/W (1 = write), bits 4:1 A_BNK,
// bit 0 8bit/n32bits (1 = the next transfer is only the top byte).
#define MAX22200_CMD_WRITE 0x80
#define MAX22200_CMD_8BIT 0x01

// STATUS[7:0] flag bits (Table 10). They also come back on SDO during
// every command-byte write.
#define MAX22200_ST_ACTIVE (1u << 0)
#define MAX22200_ST_UVM (1u << 1)   // under-voltage, set at power-up
#define MAX22200_ST_COMER (1u << 2) // SPI communication error
#define MAX22200_ST_DPM (1u << 3)
#define MAX22200_ST_HHF (1u << 4) // HIT current not reached
#define MAX22200_ST_OLF (1u << 5) // open load
#define MAX22200_ST_OCP (1u << 6) // over-current
#define MAX22200_ST_OVT (1u << 7) // over-temperature
#define MAX22200_ST_FAULTS 0xFEu

// STATUS[23:16] mask bits and master frequency
#define MAX22200_M_OVT (1ul << 23)
#define MAX22200_M_OCP (1ul << 22)
#define MAX22200_M_OLF (1ul << 21)
#define MAX22200_M_HHF (1ul << 20)
#define MAX22200_M_DPM (1ul << 19)
#define MAX22200_M_COMF (1ul << 18)
#define MAX22200_M_UVM (1ul << 17)
#define MAX22200_FREQM_80K (1ul << 16) // 0 = 100 kHz, 1 = 80 kHz

// CFG_CHx fields (Table 11)
#define MAX22200_CFG_HFS (1ul << 31)
#define MAX22200_CFG_HOLD_SHIFT 24
#define MAX22200_CFG_TRGNSPI (1ul << 23)
#define MAX22200_CFG_HIT_SHIFT 16
#define MAX22200_CFG_HIT_T_SHIFT 8
#define MAX22200_CFG_VDRNCDR (1ul << 7)
#define MAX22200_CFG_HSNLS (1ul << 6)
#define MAX22200_CFG_FREQ_SHIFT 4
#define MAX22200_CFG_SRC (1ul << 3)
#define MAX22200_CFG_OL_EN (1ul << 2)
#define MAX22200_CFG_DPM_EN (1ul << 1)
#define MAX22200_CFG_HHF_EN (1ul << 0)

#define MAX22200_CODE_MAX 126 // 127 = unregulated full on, never used

/// Low-level access to the chip pins. The firmware supplies an STM32
/// implementation, the host tests a recording fake.
class Max22200Bus
{
public:
   /// Drive CSB of chip 'dev' low (active = true) or high.
   virtual void Select(uint8_t dev, bool active) = 0;
   /// Drive the CMD pin. High while writing the command byte.
   virtual void SetCmd(bool high) = 0;
   /// Clock one byte out on SDI (MSB first, SPI mode 0) and return SDO.
   virtual uint8_t Transfer(uint8_t tx) = 0;

protected:
   // Not virtual: buses are static objects that are never deleted, and a
   // virtual destructor would pull operator delete into the firmware.
   ~Max22200Bus() = default;
};

/// Settings shared by all eight channels of one chip.
struct Max22200Config
{
   uint16_t ifsmA;     // full-scale current for HFS = 0, set by RREF
   uint8_t halfScaleMask; // bit per channel: HFS = 1, full scale halved
                          // (finer steps, RON doubles; only for coils
                          // that stay below IFS / 2)
   bool freq80k;       // master oscillator 80 kHz instead of 100 kHz
   uint8_t freqDiv;    // FREQ_CFG: 0 = /4, 1 = /3, 2 = /2, 3 = /1
   bool slewControl;   // SRC bit, only for chopping below 50 kHz
   bool openLoadDetect; // OL_EN
   uint16_t maxCurrentmA; // hard ceiling on any HOLD code (e.g. 850 mA)
};

/// Optional HIT phase for an on/off solenoid (none on the board).
struct Max22200Hit
{
   uint16_t hitmA;  // 0 = no HIT phase
   uint8_t hitTime; // HIT_T code: t = HIT_T x 40 / f_chop, 255 = continuous
};

/// Faults read back from the chip.
struct Max22200Faults
{
   uint8_t status; // STATUS[7:0] flags, see MAX22200_ST_*
   uint8_t ocp;    // per channel, bit 0 = channel 0
   uint8_t hhf;
   uint8_t olf;
   uint8_t dpm;
};

class Max22200
{
public:
   Max22200(Max22200Bus &bus, uint8_t dev);

   /** Run the data sheet power-up procedure (Figure 6): read STATUS to
    * clear UVM, write the hardware configuration with ACTIVE = 1, write
    * all eight channel configurations with every channel off, then read
    * STATUS again. Returns true if no fault flag is left set.
    */
   bool Init(const Max22200Config &cfg);

   /// Configure a HIT phase on one channel (call after Init, channel off).
   void SetHit(uint8_t ch, const Max22200Hit &hit);

   /** Set the regulated current of one channel in 0.1 mA. Only the HOLD
    * byte is rewritten, and only if the code changed. 0 turns the channel's
    * regulation target to zero; use SetOnMask() to switch it off.
    */
   void SetCurrent(uint8_t ch, int32_t current_0p1mA);

   /// Switch channels on (bit = 1) or off. One-byte write of STATUS[31:24].
   void SetOnMask(uint8_t mask);
   uint8_t GetOnMask() const { return onMask; }

   /** Read STATUS and FAULT. On the MAX22200A the faults that were seen are
    * then cleared one by one (selective clear on read), so a fault that
    * appears between the read and the clear is not lost. On the older
    * MAX22200 the first read already clears everything.
    */
   Max22200Faults ReadFaults();

   /// Flags returned on SDO by the most recent command-byte write.
   uint8_t LastFlags() const { return lastFlags; }
   /// True if a 32-bit write echoed a command byte that did not match.
   bool EchoError() const { return echoError; }

   /// HOLD code for a current in 0.1 mA, clamped to 0..maxCode.
   static uint8_t CurrentToCode(int32_t current_0p1mA, uint16_t ifsmA,
                                bool halfScale, uint8_t maxCode);
   /// Current in 0.1 mA that a HOLD code produces (for logging).
   static int32_t CodeToCurrent(uint8_t code, uint16_t ifsmA, bool halfScale);
   /// Build a command byte (Table 9).
   static uint8_t CommandByte(uint8_t reg, bool write, bool eightBit);
   /// Build a 32-bit CFG_CHx value for a CDR low-side channel.
   static uint32_t CfgWord(const Max22200Config &cfg, bool hfs, uint8_t holdCode,
                           uint8_t hitCode, uint8_t hitTime);

   uint8_t HoldCode(uint8_t ch) const { return holdCode[ch & 7]; }

private:
   uint8_t WriteCommand(uint8_t cmd);
   void Write32(uint8_t reg, uint32_t value);
   void Write8(uint8_t reg, uint8_t value);
   uint32_t Read32(uint8_t reg, uint32_t sdiMask);

   Max22200Bus &bus;
   uint8_t dev;
   Max22200Config cfg;
   uint8_t maxCode[2]; // HOLD ceiling for HFS = 0 and HFS = 1
   bool Hfs(uint8_t ch) const { return (cfg.halfScaleMask >> ch) & 1; }
   uint8_t holdCode[MAX22200_CHANNELS];
   uint8_t hitCode[MAX22200_CHANNELS];
   uint8_t hitTime[MAX22200_CHANNELS];
   uint8_t onMask;
   uint8_t lastFlags;
   bool echoError;
};

#endif // MAX22200_H
