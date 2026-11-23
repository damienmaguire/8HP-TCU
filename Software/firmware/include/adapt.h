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
#ifndef ADAPT_H
#define ADAPT_H

/* Clutch fill adaptation: a learned fill-pressure offset per clutch and
 * per oil-temperature class, added to the calibrated fill pressure
 * (KL_PFCG) of that clutch when it is the on-coming clutch.
 *
 * Temperature classes (the OEM cold / warm windows, CS_UN/OB_ADAKALT and
 * CS_UN/OB_ADAWARM [C]): cold 10..30 degC, warm 50..110 degC. Learning
 * only happens inside a class; the offset used between and outside the
 * classes is the nearer class's.
 *
 * One learning shift per clutch (each clutch is the on-coming one there):
 *   3->4 D, 4->5 C, 5->6 E, 6->7 A, and 6->5 (downshift) B, because B is
 * never the on-coming clutch of an upshift.
 *
 * Learning rule (deliberately slow): after a learning shift, if the
 * input speed flared above the old gear's sync speed by more than
 * AdaFlareRpm, or the shift took longer than AdaShiftMs + AdaBandMs, the
 * clutch filled late: offset + AdaStep. If it was quicker than
 * AdaShiftMs - AdaBandMs (harsh): offset - AdaStep. The offset is
 * limited to +-AdaMax. The target time and band are placeholders.
 *
 * Every entry condition must hold at the start AND the end of the shift
 * (Status() = 0 means allowed, else the ADA_BLK_* bits).
 *
 * Storage: own flash block (not the parameter block), so a parameter
 * load, save or upload never touches it; only the explicit reset command
 * clears it. Pack() / Unpack() give the stored image with a CRC. */

#include <stdint.h>

#define ADA_CLUTCHES 5
#define ADA_CLASSES 2
enum AdaClass
{
   ADA_COLD = 0,
   ADA_WARM = 1,
};

// Reasons learning is blocked (Status bits).
#define ADA_BLK_DISABLED (1u << 0)
#define ADA_BLK_TORQUE (1u << 1)
#define ADA_BLK_RPM (1u << 2)
#define ADA_BLK_OIL (1u << 3)
#define ADA_BLK_GEAR (1u << 4)
#define ADA_BLK_PEDAL (1u << 5)
#define ADA_BLK_DRIVETIME (1u << 6)
#define ADA_BLK_MODE (1u << 7)  // not Normal drive mode / Auto
#define ADA_BLK_FAULT (1u << 8)
#define ADA_BLK_CLASS (1u << 9) // oil outside both temperature classes

struct AdaConfig
{
   bool enable;                 // AdaptEnable (default off)
   int32_t tqMin, tqMax;        // Nm, 55..180
   int32_t rpmMin, rpmMax;      // engine / input rpm, 1600..3200
   int32_t oilMin, oilMax;      // degC, 55..80
   int32_t gearMin, gearMax;    // 3..7
   int32_t pedalMin;            // %, 2
   int32_t driveMinS;           // s driven since start, 180
   int32_t shiftMs, bandMs;     // target shift time and band, ms
   int32_t flareRpm;            // flare threshold, rpm
   int32_t step;                // mbar per learning shift
   int32_t maxOfs;              // mbar limit
};

struct AdaInputs
{
   uint8_t mode;     // TcuMode
   uint8_t gear;     // engaged gear
   uint8_t target;   // target gear
   int32_t m_t;      // Nm
   int32_t n_t;      // rpm
   int32_t n_mot;    // rpm
   int32_t n_ab;     // rpm
   int32_t temp;     // degC
   int32_t pedal;    // %
   int32_t drivenS;  // s
   bool normalMode;  // DriveMode Normal and no manual gear
   bool faultFree;   // no TCU fault, not in limp
};

/// Stored image: magic, version, offsets, count, CRC.
struct AdaImage
{
   uint32_t magic;
   uint16_t version;
   uint16_t shifts;
   int16_t ofs[ADA_CLUTCHES][ADA_CLASSES];
   uint32_t crc;
};

#define ADA_MAGIC 0x41444150u // "ADAP"
#define ADA_VERSION 1

class Adapt
{
public:
   Adapt();
   void Configure(const AdaConfig &c) { cfg = c; }
   /// Every 10 ms.
   void Tick10ms(const AdaInputs &in);
   /// Entry-condition bits right now (0 = learning allowed).
   uint32_t Status() const { return status; }
   /// Offset for a clutch at a temperature, mbar.
   int32_t Offset(int clutch, int32_t temp) const;
   int16_t Raw(int clutch, int cls) const { return ofs[clutch][cls]; }
   uint16_t Shifts() const { return shifts; }
   /// Learned values changed since the last Pack (save when convenient).
   bool Dirty() const { return dirty; }
   /// Explicit reset: all offsets 0.
   void Reset();
   void Pack(AdaImage *img);
   /// Load a stored image; false (and all zero) if magic / CRC are wrong.
   bool Unpack(const AdaImage &img);

   /// Temperature class for learning, -1 if outside both.
   static int Class(int32_t temp);
   /** Learning clutch for a shift from -> to, -1 if not a learning shift. */
   static int LearnClutch(int from, int to);
   static uint32_t Crc(const AdaImage &img);

private:
   uint32_t Gate(const AdaInputs &in) const;
   void Learn(int clutch, int cls, int32_t ms, int32_t flare);

   AdaConfig cfg;
   int16_t ofs[ADA_CLUTCHES][ADA_CLASSES];
   uint16_t shifts;
   uint32_t status;
   bool dirty;
   // Running shift.
   bool inShift, startOk;
   uint8_t from, to;
   int32_t shiftMs, flare;
};

#endif // ADAPT_H
