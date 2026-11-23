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
#ifndef LINPUMP_H
#define LINPUMP_H

#include <stdint.h>

/* LIN master for the ZF integrated electric oil pump (IEP85, Melexis LIN
 * slave) of the 330e, after Ian's read of the OEM driver
 * (330E_LIN_OIL_PUMP.md, CDD_IEP_Control). Labels are his: [C] confirmed
 * in code or calibration, [I] inference, [H] hypothesis.
 *
 * Bus [C]: 19200 baud, TCU is master, all frames 8 bytes. Schedule
 * 0x30, 0x32, 0x30, 0x33, 0x30, 0x34, repeating, one slot each (slot
 * time 10 ms [I], parameter LinSlotMs), so 0x30 goes every 20 ms.
 * PIDs F0 / 32 / 73 / B4. Checksum: not found in the firmware [H:
 * enhanced, LIN 2.x default]; classic selectable.
 *
 * 0x30 TCU -> pump [C]:
 *  b0 operating point: 0xAA off/sleep, 0x55 run (BP1). 0xCC / 0x33 exist
 *     but the 330e calibration never asks for them, so they are not used.
 *  b1 flags: 0x02 clear pump errors (for exactly one frame), 0x04 limp-home
 *     request, 0x08 warm-up request
 *  b2 gearbox oil temperature, degC + 50, saturated 0..255
 *  b3 after-run time if LIN is lost, minutes (0 in the 330e calibration)
 *  b4, b5 0
 *  b6 CRC-8 over b0 (poly 0x1D, init 0, no final XOR): AA -> 73, 55 -> B7
 *  b7 alive counter in the low nibble, +1 per 0x30 sent
 * There is no speed or pressure setpoint.
 *
 * 0x32 pump -> TCU [C]: b0/b1 error bytes EB0/EB1, b2 actual operating
 * point, b3 speed x 32 rpm, b4 current 0.2 A/bit, b5 KL30 voltage
 * 0.1 V/bit (14.0 V substitute when invalid), b6 CRC-8 over b2, b7 alive.
 * (b4/b5 scaling is from the A2L; to be bench-checked.)
 * 0x33: EB0/EB1, b2 pump temperature raw (7.3 mV/bit, curve unverified),
 * b3..b6 32-bit pump ID (big endian), b7 alive.
 * 0x34: EB0/EB1, b2 HW version, b3..b4 SW version (big endian), b7 alive.
 *
 * Receive checks as in the OEM [C]: PID must match; a 0x32 with a bad
 * CRC-8 is dropped and two in a row set the CRC fault; an alive counter
 * repeated twice in a row, or jumping by more than 2, sets the alive
 * fault and the frame is dropped; no valid 0x32 for 1.8 s (LinBusFault)
 * is a bus fault. After a clear-errors frame the diagnosis is ignored for
 * 400 ms.
 *
 * Faults are collected in a 32-bit word laid out like the OEM
 * ctl_IEP_Status: EB0 in bits 16..23, EB1 in 24..31, the TCU's own checks
 * in the low bits. The OEM masks then give the fault classes.
 *
 * Fault reaction: report only. The firmware keeps sending as before and
 * does not change the TCU (no neutral, no gear-in block); there is no OEM
 * answer for that yet (README "LIN oil pump").
 */

namespace Lin
{
/// Protected identifier: ID plus parity bits P0 (bit 6) and P1 (bit 7).
uint8_t Pid(uint8_t id);
/// LIN 1.x classic checksum: inverted 8-bit sum with carry over the data.
uint8_t ChecksumClassic(const uint8_t *data, int n);
/// LIN 2.x enhanced checksum: as classic but the PID is included.
uint8_t ChecksumEnhanced(uint8_t pid, const uint8_t *data, int n);
/// Checksum of the selected type.
uint8_t Checksum(bool enhanced, uint8_t pid, const uint8_t *data, int n);
/// SAE J1850 CRC-8 as the IEP uses it: poly 0x1D, init 0, no final XOR,
/// MSB first.
uint8_t Crc8(const uint8_t *data, int n);
/** Find a frame in the bytes read back from the bus (the master hears its
 * own header and, for a publish frame, its own data). Looks for sync 0x55
 * followed by pid and returns the index of the first data byte, or -1.
 * A break shows up as a 0x00 before the sync and is skipped. */
int FindData(const uint8_t *buf, int n, uint8_t pid);
} // namespace Lin

#define LINPUMP_ID_CMD 0x30
#define LINPUMP_ID_STATUS 0x32
#define LINPUMP_ID_AUX1 0x33
#define LINPUMP_ID_AUX2 0x34

#define LINPUMP_OP_OFF 0xAA
#define LINPUMP_OP_RUN 0x55

#define LINPUMP_FLAG_CLEAR 0x02
#define LINPUMP_FLAG_LIMP 0x04
#define LINPUMP_FLAG_WARMUP 0x08

// TCU-side bits of the status word (OEM ctl_IEP_Status numbering [C])
#define LINPUMP_ST_LINCHK (1u << 3)   // LIN checksum error
#define LINPUMP_ST_ALIVE (1u << 4)    // alive counter stuck or jumped
#define LINPUMP_ST_NORX (1u << 7)     // no valid 0x32 for LinBusFault
#define LINPUMP_ST_CRC8 (1u << 9)     // two bad CRC-8 in a row
#define LINPUMP_ST_SWVER (1u << 13)   // SW version is not IEP85 [I: bit]
// OEM class masks [C]
#define LINPUMP_MASK_TEMP 0x03000000u // EB1 bits 0-1: temporary failure
#define LINPUMP_MASK_PERM 0xFCCF3200u // permanent (incl. EB0 0x40/0x80)
#define LINPUMP_MASK_COMM 0x0030019Bu // EB0 0x10/0x20 + bus/alive/checksum

enum LinPumpState
{
   LP_OFF = 0,  // bus silent, transceiver asleep (disabled or F11)
   LP_IDLE = 1, // schedule running, operating point AA (off)
   LP_RUN = 2,  // schedule running, operating point 55 (run)
};

/// Fault classes as a bit mask (lpClass value).
enum LinPumpClass
{
   LPC_TEMPORARY = 1,
   LPC_PERMANENT = 2,
   LPC_COMM = 4,
   LPC_BUS = 8, // KFC_IEPBUS: no valid 0x32 for LinBusFault
};

struct LinPumpConfig
{
   bool enhanced;        // LIN checksum type
   uint16_t slotMs;      // schedule slot (10 ms [I])
   uint16_t busFaultMs;  // no valid 0x32 this long -> bus fault (1800 [C])
   uint16_t diagDelayMs; // diagnosis blocked after a clear (400 [C])
};

struct LinPumpStatus
{
   uint8_t raw32[8], raw33[8], raw34[8];
   uint8_t eb0, eb1;      // pump error bytes (latest valid frame)
   uint8_t opPoint;       // 0x32 b2
   int32_t rpm;           // b3 x 32
   int32_t currentmA;     // b4 x 200
   int32_t supplymV;      // b5 x 100, 14000 until a valid 0x32
   uint8_t tempRaw;       // 0x33 b2 (scaling unverified)
   uint32_t pumpId;       // 0x33 b3..b6
   uint8_t hwVersion;     // 0x34 b2
   uint16_t swVersion;    // 0x34 b3..b4
   bool seen32, seen34;
};

struct LinSlot
{
   uint8_t id;      // frame ID (not PID)
   bool publish;    // true: master sends data[0..7] + checksum
   uint8_t data[8];
};

class LinPump
{
public:
   LinPump();
   void Configure(const LinPumpConfig &c) { cfg = c; }
   /** Call every 1 ms. active = bus on (enabled, gearbox has the pump),
    * run = operating point 55, oilTempC = sump temperature for b2.
    * Returns true when a frame slot starts now; the slot is in *slot. */
   bool Tick1ms(bool active, bool run, int32_t oilTempC, LinSlot *slot);
   /** Response for id (0x32..0x34): bytes = 8 data + LIN checksum as seen
    * on the bus. Returns false if the frame was dropped. */
   bool OnResponse(uint8_t id, const uint8_t *bytes, int n);
   /// Our own 0x30 read back wrong (bus short / collision).
   void OnTxError() { txErrors++; }
   /// Send the clear-errors flag in the next 0x30 (one frame) and reset
   /// the TCU-side fault bits. Done at start-up and on FaultClear.
   void ClearErrors();
   /// Extra flags for b1 (LINPUMP_FLAG_LIMP / _WARMUP). Not used yet.
   void SetFlags(uint8_t f) { extraFlags = f & (LINPUMP_FLAG_LIMP | LINPUMP_FLAG_WARMUP); }

   LinPumpState State() const { return state; }
   bool TransceiverAwake() const { return state != LP_OFF; }
   const LinPumpStatus &Status() const { return st; }
   /// OEM-style status word (EB0 << 16 | EB1 << 24 | own bits).
   uint32_t StatusWord() const;
   /// Fault classes (LinPumpClass bits); 0 during the post-clear delay.
   uint8_t Classes() const;
   uint16_t LinChecksumErrors() const { return linChkErrors; }
   uint16_t CrcErrors() const { return crcErrors; }
   uint16_t AliveErrors() const { return aliveErrors; }
   uint16_t TxErrors() const { return txErrors; }

   /// The 0x30 data bytes for the present state (tests and logging).
   void Command(uint8_t data[8]) const;

private:
   bool CheckAlive(int idx, uint8_t alive);

   LinPumpConfig cfg;
   LinPumpState state;
   LinPumpStatus st;
   uint32_t tSlot;       // ms into the current slot
   uint8_t slotIdx;      // 0..5
   uint8_t txAlive;      // 0..15, value of the next 0x30
   uint8_t oilRaw;
   uint8_t extraFlags;
   bool clearPending;    // send 0x02 in the next 0x30
   uint32_t tDiagBlock;  // ms left with diagnosis blocked
   uint32_t tNoValid;    // ms since the last valid 0x32
   uint32_t ownBits;
   uint8_t badCrc;       // consecutive bad CRC-8 on 0x32
   int8_t lastAlive[3];  // per response frame, -1 = none yet
   uint8_t stuck[3];
   uint16_t linChkErrors, crcErrors, aliveErrors, txErrors;
};

/* When to run the pump (330e strategy [C values, I for how they combine]):
 * input speed below LinPumpRpm (500, hysteresis LinPumpHyst 50), sump
 * LinTempMin..LinTempMax (0..125 degC) and system pressure below LinPMax
 * (20 bar). After the rule drops, the pump keeps running for LinOffDelay
 * (OEM D_t_IEP_ShutdownDelay 800 ms [I: use]).
 */
enum LinPumpReq
{
   LINREQ_OFF = 0,
   LINREQ_AUTO = 1,  // the 330e run rule
   LINREQ_FORCE = 2, // bench: run whenever the bus is on
};

struct LinRuleConfig
{
   int32_t onRpm, hystRpm;
   int32_t tempMin, tempMax;   // degC
   int32_t pMaxMbar;           // no run at or above this system pressure
   int32_t offDelayMs;
};

class LinPumpRule
{
public:
   LinPumpRule() : cfg(), on(false), speedOn(false), tOff(0) {}
   void Configure(const LinRuleConfig &c) { cfg = c; }
   /// Every 10 ms. Returns true while the pump should run.
   bool Update(uint8_t req, int32_t n_t, int32_t sumpC, int32_t pSysMbar);
   bool On() const { return on; }

private:
   LinRuleConfig cfg;
   bool on, speedOn;
   int32_t tOff;
};

#endif // LINPUMP_H
