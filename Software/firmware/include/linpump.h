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

/* LIN master for the ZF electric auxiliary oil pump (IEP85 class, label
 * "IEP85 49 20 D5, 0501.334.165, NBP 3.5 bar, 4.5 l/min, 12 V").
 *
 * Source: Damien's "ZF LIN Electric Oil Pump - Reverse Engineering
 * Summary" (bench reverse engineering, not an OEM document). Labels:
 * [D] stated in that summary, [H] my reading where the summary is silent.
 * Every [H] item is listed for Ian/Damien in README "LIN oil pump".
 *
 *  - 19200 baud, LIN 2.x [D]. Master command on ID 0x30, 8 bytes, every
 *    ~25 ms [D]. Pump responses on 0x32 (main status), 0x33, 0x34 [D].
 *  - Checksum: LIN 2.x uses the enhanced checksum (PID included) for IDs
 *    0x00..0x3B, so that is the default; classic is a parameter [H].
 *  - Command 0x30 [D]:
 *      cold start  AA 00 0A 00 00 00 73 cc   for ~900 ms
 *      phase 2     AA 00 42 00 00 00 73 cc   (byte 2 0x0A -> 0x42)
 *      run start   55 00 42 00 00 00 73 cc   until a 0x32 with byte 0 = 00
 *      run         55 00 42 00 00 00 ee cc   ee = byte 6 of the last 0x32
 *    cc = alive counter 0..15 in byte 7.
 *  - 0x32 [D]: byte 0 status (00 good, 40 communication/watchdog fault),
 *    byte 3 speed x 32 rpm, byte 4 supply x 0.1 V, byte 6 echo byte.
 *    Bytes 5/6 also carry raw current and temperature, scaling unknown.
 *    0x33/0x34 are not decoded (raw bytes only).
 *
 * Not in the summary, so NOT guessed here:
 *  - how to command a speed or pressure: byte 2 is sent exactly as in the
 *    summary (0x0A, then 0x42); no other value is ever sent [H: 0x42 x 32
 *    = 2112 rpm looks like a speed request in the 0x32 speed units, but
 *    that is unconfirmed].
 *  - how to stop the pump: the master just stops sending and the
 *    transceiver goes to sleep [H: assumes the pump stops on bus silence].
 *  - phase-2 duration and the "20 ms settle delay": parameters.
 *
 * This class is the hardware-free part (frames, checksums, schedule,
 * state machine, decode) so it runs in the host tests. stm32_tcu.cpp
 * moves the bytes over UART4 in LIN mode.
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
/** Find a frame in the bytes read back from the bus (the master hears its
 * own header and, for a publish frame, its own data). Looks for sync 0x55
 * followed by pid and returns the index of the first data byte, or -1.
 * A break shows up as a 0x00 before the sync and is skipped. */
int FindData(const uint8_t *buf, int n, uint8_t pid);
} // namespace Lin

enum LinPumpReq
{
   LINREQ_OFF = 0,
   LINREQ_AUTO = 1,  // run while line pressure is needed at low input speed
   LINREQ_FORCE = 2, // bench: run whenever enabled
};

/** Pump demand. Ian's spec: the pump "provides line pressure when the
 * input shaft isn't turning" (330E_MECHATRONIC_PINOUT.md, LIN row) and is
 * "essential at standstill in an EV" (330E_OPEN_TCU_CONTROL_SPEC.md). In
 * Auto it runs while pressureNeeded and the input speed is below onRpm,
 * and stops above onRpm + hyst (hysteresis so it does not chatter).
 * pressureNeeded is set by the caller (stm32_tcu.cpp: TCU in N, engage,
 * drive or shift; not in P, init or safe). The thresholds are TBD. */
bool LinPumpDemand(uint8_t req, bool enabled, bool pressureNeeded,
                   int32_t n_t, int32_t onRpm, int32_t hyst, bool wasOn);

#define LINPUMP_ID_CMD 0x30
#define LINPUMP_ID_STATUS 0x32
#define LINPUMP_ID_AUX1 0x33
#define LINPUMP_ID_AUX2 0x34

#define LINPUMP_ST_OK 0x00
#define LINPUMP_ST_COMMFAULT 0x40

enum LinPumpState
{
   LP_OFF = 0,  // not requested, transceiver asleep, bus silent
   LP_COLD1,    // AA 00 0A ...
   LP_COLD2,    // AA 00 42 ...
   LP_START,    // 55 00 42 00 00 00 73, waiting for status 00
   LP_RUN,      // 55 00 42 ... with the live echo
   LP_FAULT,    // no valid response: pause, then cold start again
};

struct LinPumpConfig
{
   bool enhanced;       // checksum type (LIN 2.x = enhanced)
   uint16_t cycleMs;    // command period, ms (summary: ~25)
   uint16_t rxDelayMs;  // 0x30 header -> 0x32 header, ms ("settle delay")
   uint16_t coldMs;     // cold-start phase 1 length, ms (summary: ~900)
   uint16_t phase2Ms;   // phase 2 length before run start, ms (not given)
   uint16_t timeoutMs;  // no good 0x32 this long -> LP_FAULT
   uint16_t retryMs;    // pause in LP_FAULT before a new cold start
   bool pollAux;        // also read 0x33 / 0x34 (alternating), 8 ms after
                        // the 0x32 header; needs cycleMs >= rxDelayMs + 16
};

/// Decoded 0x32 plus the raw 0x33/0x34 bytes.
struct LinPumpStatus
{
   uint8_t raw32[8];
   uint8_t raw33[8];
   uint8_t raw34[8];
   uint8_t status;     // byte 0
   int32_t rpm;        // byte 3 x 32
   int32_t supplymV;   // byte 4 x 100
   uint8_t echo;       // byte 6
   bool fresh;         // a good 0x32 arrived since the last cold start
};

/// What the master must put on the bus now.
struct LinSlot
{
   uint8_t id;         // frame ID (not PID)
   bool publish;       // true: master sends data[0..7] + checksum
   uint8_t data[8];
};

class LinPump
{
public:
   LinPump();
   void Configure(const LinPumpConfig &c) { cfg = c; }
   /** Call every 1 ms. request = pump wanted. Returns true when a frame
    * slot starts now; the slot is then in *slot. */
   bool Tick1ms(bool request, LinSlot *slot);
   /** A response was received for id (subscriber frames 0x32..0x34).
    * bytes = the 8 data bytes plus the checksum as seen on the bus.
    * Returns false (and ignores it) if the checksum is wrong. */
   bool OnResponse(uint8_t id, const uint8_t *bytes, int n);
   /// The master's own 0x30 publish failed (read-back mismatch): count it.
   void OnTxError() { txErrors++; }

   LinPumpState State() const { return state; }
   const LinPumpStatus &Status() const { return st; }
   bool TransceiverAwake() const { return state != LP_OFF; }
   /// Faults: 0x40 replies seen, checksum errors, timeouts, tx errors.
   uint16_t CommFaults() const { return commFaults; }
   uint16_t ChecksumErrors() const { return csErrors; }
   uint16_t Timeouts() const { return timeouts; }
   uint16_t TxErrors() const { return txErrors; }

   /// The 0x30 data bytes for the present state (for tests and logging).
   void Command(uint8_t data[8]) const;

private:
   void Enter(LinPumpState s);

   LinPumpConfig cfg;
   LinPumpState state;
   LinPumpStatus st;
   uint32_t tState;   // ms in this state
   uint32_t tCycle;   // ms into the schedule cycle
   uint32_t tNoGood;  // ms since the last good 0x32
   uint8_t alive;     // 0..15
   uint8_t auxToggle;
   uint16_t commFaults, csErrors, timeouts, txErrors;
};

#endif // LINPUMP_H
