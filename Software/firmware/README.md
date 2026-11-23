# 8HP TCU firmware (STM32F107VCT6)

Open-source transmission control for the ZF 8HP (first target: BMW GA8P75HZ
in EV conversions). Built on the openinverter libraries the same way as the
ZombieVerter VCU (Stm32-vcu): libopeninv parameters, terminal, CAN map and
SDO on top of libopencm3.

> **Status: first cut, not run on hardware.** It builds and the host unit
> tests pass, but nothing has been tried on Bob's board or a gearbox yet.
> The pins follow Bob's **Stm32_TCU_V4** map (see "Pin map"), and several values
> that need a bench measurement are parameters with placeholder defaults
> (see "Parameters for unmeasured items" below). Do not connect this to a
> car until those are resolved.
>
> The firmware sends **no CAN frames** by default (`CanTxEnable = 0`,
> `LogEnable = 0`). SDO replies on CAN1 only go out when a tool asks.
> Immobiliser handling is out of scope.

The control strategy follows Ian's reverse-engineering notes of the F30
330e EGS (in the `8hp45-dump/analysis` package):
`330E_OPEN_TCU_CONTROL_SPEC.md`, `330E_SAS_FILL_RAMP.md`,
`330E_EDS_PWM.md`, `330E_MECHATRONIC_PINOUT.md`,
`330E_SHIFT_CLUTCH_FIRST_CUT.md` and the calibration export
`330e_cal_export/330e_sas_eds_cal.json`. Section numbers below (§) refer to
the control spec unless stated otherwise. [C] / [I] / [H] tags are Ian's:
confirmed in code / inferred / from public sources.

## Building

Ubuntu 22.04 or later:

```
sudo apt-get install gcc-arm-none-eabi g++ make git
git clone --recursive https://github.com/damienmaguire/8HP-TCU.git
cd 8HP-TCU/Software/firmware
make get-deps      # builds libopencm3 for STM32F1 (first time only)
make               # -> stm32_tcu, stm32_tcu.bin, stm32_tcu.hex
make Test          # host unit tests
test/test_tcu
```

If the repository was cloned without `--recursive`, run
`git submodule update --init` first (`make` does this for you if
libopencm3 has not been built).

Code::Blocks: open `tcu8hp.cbp`. The `STM32F107` target runs `make`, the
`Test` target runs `make Test`.

The image links at 0x08001000 (4 KiB bootloader in front, as on the
openinverter boards). Parameters live in the last 2 KiB flash page and the
CAN map in the page below it (libopeninv F1 layout, see `stm32_tcu.ld`).
Build output (`obj/`, `stm32_tcu*`) is ignored by git; no binaries are
committed. The CI workflow (`ci/CI-build.yml`, to be moved to
`.github/workflows/CI-build.yml` at the repository root to switch it on)
builds the firmware, runs the tests and attaches the .bin/.hex to the run.

### Calibration tables

`include/cal_330e.h` and `src/cal_330e.cpp` are generated from Ian's export
by `tools/gen_cal.py`. Only the tables the firmware uses are emitted (554
items, about 7,700 values) out of the full export. Every
table carries its A2L name and ROM address in a comment. To regenerate:

```
make cal CAL_JSON=/path/to/330e_sas_eds_cal.json            # Normal set
make cal CAL_JSON=/path/to/330e_sas_eds_cal.json CAL_SET=el # _EL set (Q7)
```

The F11 8HP45 set (`include/cal_f11.h`, `src/cal_f11.cpp`) comes from
Ian's `f11_cal_export/f11_sas_eds_cal.json` the same way:

```
make cal-f11 F11_JSON=/path/to/f11_sas_eds_cal.json CAL_JSON=/path/to/330e_sas_eds_cal.json
```

That export has the seven EDS curves (A..E, SYS, TCC) and the four fill
items per clutch (`KF_PSFCGMT_x`, `KL_TSFCG_x`, `KL_PFCG_x`, `KL_PFHYSCG_x`;
blocks every 0x594 bytes from 0x19FFF8). Those are per variant; the
quick-fill time offsets `KF_TOSFHS/RS_x` are not in the export and are
copied from the 330e (marked in the generated file). The F11 has no A2L,
so its names are ported from the 330e by address [I]. Two [I] points in
the F11 data: which of +0x524 / +0x55C is the fill pressure and which the
hysteresis (assigned by matching 330e sizes), and the second axis of the
quick-fill pressure maps, which is raw torque counts (25..84) in the
export and is used as it is until the scaling is known.

`calset.*` holds one row per gearbox (`GearboxType`): the EDS curves, the
fill items, and pointers to the shift, gear-in, line-pressure and global
tables, plus the channel 7 default, LIN pump present and the torque
frame ID. Today the F11 row shares everything except the EDS and fill
tables with the 330e (the clutch pattern tables are byte-identical in both
images [C]; F11 shift tables are not exported yet). When Ian sends the
8HP45/70/90 sets (same a81 program), each one is a new generated file, a
new row in `calset.cpp` and a new name in `GBOXTYPES`.

Units after conversion: pressure mbar, gradients mbar/s, current 0.1 mA,
time ms, torque Nm, speed rpm, temperature degC. Maps are stored
`z[iy * nx + ix]` with x = the first A2L axis. "Time before sync" cals stay
negative as in the ROM (-30 = 30 ms before sync).

## Architecture

```
            CAN1 (VCU)          speed sensors      ATF / Vbat ADC
                 |                    |                  |
                 v                    v                  v
 stm32_tcu.cpp   ----------- inputs every 10 ms ------------
                                      |
                                      v
 tcu.cpp       mode machine: INIT / PARK / NEUTRAL / ENGAGE / DRIVE / SHIFT / SAFE
               gear choice, line pressure, fault latch, torque-intervention ramp
                 |                 |                |
           engage.cpp          sas.cpp        interlock.cpp
           N->1, N->R          single-swap     max 3 applied (4 in a swap),
                               KZU/KAB phases  only the allowed elements
                 \_________________|________________/
                                   v
                    pressure per element A..E + line (mbar)
                                   |
 eds.cpp        KL_EDSKV* curves, temperature correction, slew 60 mA/10 ms,
                125 Hz dither (1 ms task)  -> current per valve (0.1 mA)
                                   |
 valveout.cpp   valve -> MAX22200 channel map (parameters), fault aggregation
                                   |
 max22200.cpp   SPI driver: HOLD DAC per channel (CDR), ON bits, faults
```

* **Integer maths only.** All lookups are clamped linear (curves) or
  bilinear (maps) interpolation with 64-bit intermediates
  (`src/caltypes.cpp`).
* **Hardware-free core.** Everything from `tcu.cpp` down to `max22200.cpp`
  has no libopencm3 or parameter dependency, so the same code runs in the
  host tests. `stm32_tcu.cpp` copies parameters into config structs and
  wires up the hardware.
* **Tasks** (Stm32Scheduler on TIM4): 1 ms dither + SPI updates + LIN
  schedule, 10 ms shifter + control + CAN, 100 ms faults, ATF, battery
  voltage. Interrupts: ADC2 at 40 kHz for the speed sensors, UART4 for LIN.

### Pin map (Stm32_TCU_V4)

From Bob's `Stm32_TCU_V4/pinmap/firmware_answers.md`; Damien approved the
header mapping. **REV** = waiting for the next board revision.

| Function | Pin(s) | Notes |
|---|---|---|
| Clock | 8 MHz HSE | -> 72 MHz |
| MAX22200 SPI1 (no remap) | SCK PA5, MISO PA6, MOSI PA7 | mode 0, 4.5 MHz |
| MAX22200 CSB / CMD / ENABLE | PA4 / PE8 / PE7 | soft CS active low; ENABLE high = run |
| MAX22200 nFAULT | PE10 | `INPUT_PU` (no external pull-up), polled every 10 ms |
| MAX22200 TRIGA / TRIGB | PE9 / PE11 | outputs held low (channels SPI-controlled) |
| MAX22200 OUT0..7 | A, B, C, D, E, LINE (SYS), PREL, TCC | all low-side CDR; RREF 15k -> IFS 1.0 A (0.5 A with HFS) |
| Park-hold magnet | PC7 | NCV8402, active high (off the MAX22200 on V4) |
| Spare low-side outputs | gp_out1 PC8, gp_out2 PC9 | NCV8402, `GpOut1Func`/`GpOut2Func` (off / on / starter lock) |
| gp_pwm | PC6, TIM3_CH1 | FAN3121 **inverting**: PC6 low = +12 V; TIM3 full remap; `GpPwmFreq`/`GpPwmDuty` |
| Speed ISS / OSS | PA0 / PA1 (ADC12_IN0 / IN1) | sampled by ADC2 at 40 kHz, edges with `SpdAdcLo`/`SpdAdcHi` hysteresis; sensor supply always on. **REV:** RC filter |
| Aux speed (n6) | none on V4 | next revision: probably PA2 (ADC12_IN2), `HW_HAS_SPD3`; `nAux` reads 0 on V4 |
| vbat | PC0 (IN10) | 0.18 divider, `VbatScale` 18330 mV full scale |
| atf_temp1 / atf_temp2 | PC1 (IN11) / PC2 (IN12) | 2.6k / 470R pull-up to 5 V, then /2; `AtfInput` picks one |
| ana_in | PC3 (IN13) | /2, 0-6.6 V (analog shifter placeholder) |
| din1, din2, park_in, notpark_in | PD4, PD5, PD6, PD7 | active high, external 1.5k pull-down, `INPUT_FLT` |
| LED | PE2 | active low |
| Terminal | USART3 PB10 / PB11 | 115200 |
| CAN1 | PA11 RX / PA12 TX | no remap |
| CAN2 | PB5 / PB6 (remap) | **REV:** no transceiver on V4; only started with `Can2Enable = 1` |
| LIN (aux oil pump) | UART4 TX PC10 / RX PC11, SLP_N PD11 | **REV:** no transceiver on V4; `LinPumpEnable = 0` by default |
| T15 (ignition sense) | not on V4 | next revision, pin TBD (`HW_HAS_T15`); `t15` reads 1 on V4 |
| 12 V shift up / down | not on V4 | next revision, pins TBD (`HW_HAS_SHIFT12V`) |
| MAX22200 #2 | not on V4 | next revision: SPI1, own CSB (pin TBD, `max_cs1`), PZYL on its OUT0 = channel 8 |
| Debug | SWD only | JTAG disabled (frees PA15, PB3, PB4) |

There is no Sol1 and no high-side supply switch on V4.

### Next board revision (placeholders)

Bob's next revision is not final, so its parts are behind `HW_REV` in
`hwdefs.h` (default 4 = V4; nothing changes for a V4 build). With
`HW_REV=5` the code for a second MAX22200 (SPI1, own CSB), the third
speed input (n6, PA2 probably), the 12 V shift up / down inputs and T15
is compiled in, and the build stops with `#error` until the TBD pins
(`MAX_CS1_*`, `SHIFT_UP_*`, `SHIFT_DN_*`, `T15_*`) are set from the final
pin map. `ChanPZYL` defaults to 8 (chip 2 OUT0) on that board and 255
(not fitted) on V4. No other pins are assumed.

### Modules

| File | What it does | Source |
|---|---|---|
| `gears.*` | Gear/clutch table, ratios, single-swap off/on elements, next step (adjacent only in v1) | §4, §5.1 |
| `interlock.*` | Hard check before any output, on the commanded coil currents: a clutch is open only below `IlOpenLow` (A/B, 60 mA) or above `IlOpenHigh` (C/D/E, 790 mA); every clutch outside the allowed set must be open; <= 3 applied and forming a gear, or <= 4 inside the from/to union during a swap; never 5 | §4, Ian QA2 |
| `sas.*` | Shift state machines with the OEM state numbers. Upshift KZU 1,2,4,9,10,13 + KAB 1,2,3,4,9,16; downshift KZU 1,2,5,9,12,18 + KAB 10..16; timeout 14 (KL_TMAXCG); power-on/coast picked at shift start from the torque sign; GLS PI on KZU or KAB with the OEM scaling and +-3 bar clamp; torque intervention KF_MEHAUPT / KF_MENACH | §5.3-5.6, `330E_SAS_FILL_RAMP.md`, Ian QA2 |
| `engage.*` | N->1 / N->R: V1/V2 quick fill, then the last clutch (per `GearboxType`) fills and closes, fast only with confirmed no-slip, torque near 0 and line pressure OK | §5.7, Ian QA2 |
| `eds.*` | Pressure -> current (KL_EDSKVA..K0), KF_KIEDSCG* temperature correction, slew limit, firmware dither | §6, `330E_EDS_PWM.md` |
| `max22200.*` | MAX22200/MAX22200A SPI driver | data sheet Rev 1 (3/25) |
| `valveout.*` | Valve-to-channel map (one chip on V4, two supported), per-channel HFS, fault mapping | |
| `speedsens.*` | Edge period -> rpm with tooth count as a parameter | §2, pinout §1 |
| `anaconv.*` | ADC edge finder with hysteresis (speed sensors), vbat, temperature-input resistance, R -> T table | V4 pin map |
| `shifter.*` | Selector and tip up/down from CAN or the V4 inputs (`ShifterType`), plus the 12 V shift up / down inputs (next board) | Damien |
| `linpump.*` | LIN master for the aux oil pump: PID, checksum, CRC-8, 6-slot schedule, 0x30 build, 0x32/33/34 decode, OEM receive checks and fault classes, run rule | `330E_LIN_OIL_PUMP.md` |
| `tcu.*` | Top-level mode machine, line pressure, faults, limp | §3, §5.5 |
| `cal_330e.*`, `cal_f11.*`, `caltypes.*` | Generated calibration tables (330e, F11 8HP45) and the lookup code | JSON exports |
| `calset.*` | Per-gearbox table set picked by `GearboxType`, channel 7 mode | `F11_VS_330E_DIFF.md` |
| `canif.*` | CAN1 frames per `EcuType` (handler table), F11 0x0B0 packer | |
| `stm32_tcu.cpp`, `hwinit.cpp`, `terminal_prj.cpp` | Firmware glue: params, SPI, ADC2 speed sampling, LIN UART, PWM, CAN, terminal | ZombieVerter |

### Solenoid driver (MAX22200)

The valves are driven by MAX22200AETJ+T octal solenoid drivers over SPI
(data sheet Rev 1, 3/25; register, bit and figure numbers are cited in
`include/max22200.h`):

* Every valve channel runs in **current-drive regulation (CDR), low side**.
  The chip regulates the **peak** current of each chopping cycle
  (`I = HOLD / 127 x IFS`). There is no shunt and no software current loop.
* `IFS = 15 kV / RREF` (HFS = 0, up to 1 A). V4 has RREF = 15k, so
  `MaxIfs` = 1000 mA. One HOLD step is then about 7.9 mA. HFS is per
  channel: `MaxHfsMask` bit n puts OUTn on half scale (500 mA, ~3.9 mA
  steps). Default 0 (no channel on half scale).
* The ceiling `IMaxCoil` (850 mA) caps every HOLD code. HOLD = 127 means
  "fully on, unregulated" (about 2.3 A into a 5 ohm coil) and is never
  written; the driver's largest code is 126.
* Start-up follows data sheet Figure 6: read STATUS (clears UVM), write
  STATUS with ACTIVE = 1 and all channels off, write the eight CFG_CHx
  registers, read STATUS again.
* A HOLD change is a one-byte write after an 8-bit command, sent only when
  the code changes. Channels switch with a one-byte write of STATUS[31:24].
* Faults (OCP, open load, HIT not reached, DPM, over-temperature, UVM,
  SPI error) are read every 100 ms and at once when nFAULT goes low, then
  cleared selectively (MAX22200A). A fault on A..E or SYS, a chip that
  reset itself (UVM) or a failed SPI echo puts the TCU in the safe state.
  Faults on K0 or POS are only reported.
* **Dither:** the MAX22200 has no dither generator, so the 125 Hz dither is
  made in firmware: the 1 ms task adds a triangle (or square) offset of
  `DitherAmp` (default 30 mA) with period `DitherPeriod` (default 8 ms) to
  the setpoint and rewrites the HOLD byte. At 7.9 mA per step a 30 mA
  dither is about +-4 codes. As in the OEM unit, dither is off near the
  50 mA and 850 mA end points.
* The command byte is built from data sheet Table 9 (address in bits 4:1).
  Analog Devices' no-OS driver puts the address in bits 6:0 instead. This
  firmware follows the data sheet; check it on the bench with a logic
  analyser before trusting it.
* V4: one chip, one chip select, channels switched over SPI (TRIGA/TRIGB
  held low). The code still handles a second chip for a later board.

### Safe state / limp

All coils at 0 A gives C+D+E applied = **6th gear** mechanically, and line
pressure at maximum (SYS is normally high) [C]. On any critical fault the
TCU drops every output at once (no slew), disables the MAX22200 channels
and reports gear 6. Faults latch. `FaultClear = 1` is accepted only at
standstill in P or N; it restarts the drivers and leaves the safe state if
the cause has gone.

Critical faults: driver fault on A..SYS, chip lost, interlock rejection,
battery below `UvLimit` (OEM 7.4 V), input/output speed ratio not matching
the engaged gear for `GearChkMs`, invalid valve map.

### CAN

What runs on CAN1 is chosen by `EcuType` (`canif.cpp`, one handler row per
type: receive IDs, decode, frames to send). Only `0 = None/VCU` exists:
the placeholder VCU frames below. OEM engine ECUs (DME/DDE) will be added
later as more rows; no OEM ECU frames are in the firmware yet.

CAN1 (vehicle/VCU) and CAN2 (logging, needs `Can2Enable` and a
transceiver, REV) both at 500 kbit/s by default. All
frame layouts below are **placeholders** until the VCU/inverter side is
agreed (Q5). Values little endian.

* RX `CanCmdId` (default 0x300): byte 0 selector (0=P 1=R 2=N 3=D, used
  when `ShifterType = CAN`), byte 1 gear request (0 = hold, 1..8, used
  when `ShiftMode = VCU`), bytes 2-3 actual motor torque Nm (signed),
  bytes 4-5 motor rpm (signed), byte 6 (optional, `ShifterType = CAN`)
  bit 0 tip up, bit 1 tip down.
  If it stops for `CanTimeout` ms the TCU posts `CANTIMEOUT`, uses
  `CanLossTq` for line pressure (default 600 Nm, the OEM substitute
  `D_m_IstVM_Default`) and follows `CanLossMode`: 1 (default, Damien's
  decision 2026-11-23) holds the gear at `LineMax`, refuses upshifts and
  only steps down as the car slows; 0 goes to neutral. Neutral/limp is
  kept for hydraulic and electrical faults (the safe state). No gear-in is
  started while the frame is missing.
* TX `CanStatId` (default 0x301, only if `CanTxEnable = 1`): gear, mode,
  target gear, fault, torque intervention request Nm (signed), ATF + 40,
  flags (bit 0 shifting, 1 park request, 2 safe state, 3 hold motor
  torque near 0 for gear-in, 4 CAN-loss gear hold, 5 coast shift). The torque intervention
  request is the hook for the inverter: how it reaches the inverter is
  open (Q5).
* TX `CanStatId + 1` (same condition): bytes 0-1 minimum motor speed the
  TCU asks for (rpm, 0 = none: 700 rpm to leave P and for 2 s after, or
  `PumpFaultRpm` with `PumpFaultAct = MotorSpeed`; the VCU should turn the
  motor with the clutches open), byte 2 bit 0 waiting to leave P, bit 1
  gear-in waiting for a pressure source, bit 2 Auto tip override, byte 3
  LIN pump fault classes, bytes 4-5 channel 7 pressure (mbar).
* TX 0x0B0, F11 engine torque intervention (`F11_VS_330E_DIFF.md` 5): only
  with `GearboxType = F11`, `F11TqTx = 1` and `CanTxEnable = 1` (all off
  by default; nothing goes to the car without Damien's say-so). BMW E2E
  [C]: byte 0 CRC-8 (poly 0x1D, init 0, XOR K = 0x65) over bytes 1-7,
  byte 1 low nibble counter 0..14. The request is packed as 12 bits in
  byte 1 high nibble + byte 2, raw = 2 x (Nm + 2047), the 330e
  `CAN_RQ_TORQ_CRSH_GRB` scale [I]; the other two 12-bit fields and bytes
  6-7 are 0. **Flag:** that scale only fits -2047..0 Nm in 12 bits, and
  the field positions are [I]; check against a live F11 log first.
* CAN2 log (only if `LogEnable = 1`), every 10 ms from `LogBaseId`
  (default 0x5F0): +0 pA pB pC pD, +1 pE pSys n_t n_ab, +2 iA iB iC iD (mA),
  +3 iE iSYS iK0 iPOS, +4 mode gear target zKZU zKAB engPhase interlock
  fault, +5 m_t tqDelta tSyn gls. The state numbers match the OEM ones so
  logs can be compared.
* libopeninv CAN map and SDO (node `SdoNodeId`, default 22) on CAN1, so
  any value can also be mapped from the web interface.

`CmdSource = Param` takes the selector, gear, torque and motor speed from
the `SelectorSet`, `GearSet`, `TorqueSet` and `MotorRpmSet` parameters for
bench work without a VCU.

## Implemented vs stubbed

Implemented:

* Gear/clutch tables, single-swap table (tested against §5.1), interlock.
* SAS upshift and downshift phase machines with OEM state numbers, fill,
  ramps, GLS, sync detection, timeout, torque intervention and its ramp-out.
* Coast shifts (zug_schub_sas): picked at runtime from the sign of `m_t`
  at the start of the shift, using the negative-torque points of the same
  maps (there are no separate coast ROM sets). The GLS moves to the other
  clutch: KZU for power-on upshifts and coast downshifts, KAB for coast
  upshifts and power-on downshifts (`KP/KI_GLSKAB`, `KL_TVGLSKABCG`).
* N->1 / N->R engage with the last clutch per gearbox variant, neutral
  with B pre-filled.
* Line pressure: `max(KF_PSYS_<gear>(m_t, n_mot), P_SYSMIN<xy>)` while
  shifting, `P_SYSMIN_D/R` while engaging, `KF_PSYSN` in N/P, plus
  `LineOffset`, capped at `LineMax`.
* Pressure -> current with temperature correction, slew limit, firmware
  dither.
* MAX22200 driver, configurable valve map, fault handling, limp mode.
* Speed sensors (ADC2 sampling with hysteresis, V4), ATF resistance and
  table, battery voltage.
* Web-interface dropdowns: `GearboxType`, `ShifterType`, `ShiftMode`
  (see "Setup parameters").
* LIN aux oil pump master (UART4), off by default; see "LIN oil pump".
* CAN1 command/status placeholders, CAN2 logging, CAN map, SDO, terminal.
* `ShiftMode`: Auto (simple input-speed thresholds, default), Manual
  (driver tips with rpm limits and lugging protection, or the
  `ManualGear` dropdown), VCU (gear from the CAN frame). 12 V shift up /
  down inputs (next board) and Auto tip override (`TipOverride`).
* Dual gearbox support (`GearboxType`): 8HP75 330e and 8HP45 F11 tables
  (see "Calibration tables"), gear-in last clutch per variant (F11: C for
  1st, D for R; 330e: B with the A+C / A+D pre-fill), channel 7 mode
  (`Ch7Mode`), LIN pump forced off on the F11, F11 0x0B0 torque frame
  (TX off).
* Park lock sequence (Ian 11 Q9d / 12.1): leaving P waits for the motor
  at `ParkRelRpm` (700) or a measured 17 bar, asks the VCU for that speed
  and holds it `ParkRelHold` (2 s) after; P -> N `ParkPNDelay` (150 ms);
  N -> P 1000 / 500 / 300 / 150 ms at -30 / -15 / 0 / 40 degC (from a
  drive range `KL_TVPSEIN`); with `ParkCtrl = 1` PZYL at `PzylCurrent`
  (550 mA + cold offset up to 190 mA) while released and the park magnet
  for `ParkMagMin`..`ParkMagMax` (100 ms .. 1 s) after a release.
  `ParkCtrl` is 0 by default (outputs stay off) until the bench confirms
  the PZYL channel and the sequence.
* Gear-in source rule (Ian 11 Q9d [I]): N -> D/R only after the LIN pump
  has delivered for `GearInPumpMs` (1 s) or with the motor at
  `GearInMotRpm` (100 rpm), and with line pressure at least `GearInMinP`
  (8 bar, `P_SYSMINIEP`; the OEM `P_SYSMIN_D/R` 9 bar is higher and
  wins). **Bench note:** with no pump and the motor at 0 rpm there is no
  gear-in (and no leaving P below 700 rpm); set `MotorRpmSet` with
  `CmdSource = Param`, or `GearInPumpMs = GearInMotRpm = 0` / `ParkRelRpm =
  0` to switch the waits off.
* Pump fault reaction `PumpFaultAct` (Ian 11 Q9a): 0 = report (default),
  1 = ask the VCU for `PumpFaultRpm` (600) in P/N with the clutches open,
  2 = no new gear-in while the fault is there.

Stubbed / not done yet:

* **Park lock outputs**: written (see above) but off by default
  (`ParkCtrl = 0`); PZYL has no V4 channel by default (`ChanPZYL = 255`);
  the magnet / PZYL order is [I].
* **Electric oil pump (IEP) over LIN**: done to Ian's OEM read (see "LIN
  oil pump"), but untested (no transceiver on V4).
* **n6 cross-check**: the third speed input is read on the next board
  (`nAux`), but the OEM check needs the per-gear `D_k_n6` factors (only
  some are known: 1.5, -2.214, 7.475, not tied to gears), so it is TBD.
  Shifting uses n_t / n_ab with the ratio check (`GearChk*`) and the
  "never more than 3 applied clutches" interlock, as Ian suggests.
* **TCC lock-up (F11)**: a simple lock above `TccOnRpm` / open below
  `TccOffRpm` in a steady gear >= `TccMinGear`, ramped at `TccRamp` to
  `TccPress`, never during a shift or gear-in. All values TBD; the OEM
  WK control was not ported.
* **F11 POS / Sol1**: unconfirmed. No POSV / Sol1 / park strings in the F11
  image; an 8th curve (0x19FE14) may be POS [I]. `PosCurrent` and
  `ChanPOS` are plain parameters; nothing F11-specific is assumed.
* **Analog shifter mapping**: placeholder (see "Setup parameters").
* **Inverter torque interface** (Q5): only the request value in the status
  frame.
* Skip shifts and the OEM routing tables (0x51418 / 0x51350): v1 only does
  adjacent shifts.
* `TAB_IKABENTLEER` (0x234c78), the shift-type switch for an off-going
  current ramp, is all zeros in the 330e calibration (`CS_IKABENTLEER`
  -40 degC), so that ramp is never active. It is not implemented; revisit
  if another calibration sets it.
* The SP/SPORT/_EL sets (only Normal is generated; `CAL_SET=el` exists).
* Fill adaptation (Q8), POS valve logic (fixed `PosCurrent`; the OEM
  switch-over is 420 mA), K0 (held at `K0Press`, default 0 = open).

## Parameters for unmeasured items

Nothing that still needs a measurement is hard-coded. These are the
parameters to set once the value is known:

| Item | Parameter(s) | Default (placeholder) | Open question |
|---|---|---|---|
| Solenoid -> MAX22200 channel | `ChanA` .. `ChanPZYL` (chip x 8 + channel, 255 = not fitted) | A..E = 0..4, SYS 5, POS 6 (PREL, assumed), K0/TCC 7, PZYL 255 (V4) / 8 (next board) | Bob, Damien bench |
| MAX22200 full scale / half scale | `MaxIfs`, `MaxHfsMask` | 1000 mA (RREF 15k), no channel on HFS | Bob |
| Chopping (PWM) frequency | `MaxFreq`, `MaxFreqDiv` | 100 kHz / 4 = 25 kHz | Q10 |
| Dither amplitude / period / shape | `DitherAmp`, `DitherPeriod`, `DitherShape` | 30 mA, 8 ms, triangle | Q10 |
| Speed sensor teeth | `TeethIn`, `TeethOut`, `TeethAux` (no pin) | 60 (OEM firmware value) | Q3 |
| Speed timeout / minimum | `SpdTimeout`, `SpdMinRpm` | 100 ms, 30 rpm | Q3 |
| Speed sensor ADC thresholds | `SpdAdcLo`, `SpdAdcHi` (read `spdInLo/Hi`, `spdOutLo/Hi` to set them) | 900 / 1200 mV (for 0.7 / 1.4 V) | bench, REV RC filter |
| ATF sensor | `AtfSource`, `AtfFixed`, `AtfInput`, `AtfR1..6`, `AtfT1..6`, `AtfRMin/Max` | fixed 40 degC; generic NTC table (70R at 140 degC .. 26k at -30 degC) | Q4, Damien (which input) |
| Battery voltage scaling | `VbatScale` | 18330 mV full scale (V4 0.18 divider) | Bob |
| Analog shifter | `ShAnaR`, `ShAnaN`, `ShAnaMin`, `ShAnaMax` | 1500 / 3000 / 200 / 6000 mV | shifter not chosen |
| Manual mode limits | `ManMaxRpm`, `ManMinRpm` | 6000 / 1000 rpm | bench |
| Spare outputs | `GpOut1Func`, `GpOut2Func`, `GpPwmFreq`, `GpPwmDuty` | off, 200 Hz, 0 % | use not decided |
| Aux pump LIN | `LinPumpEnable`, `LinPumpReq`, `LinPumpRpm`, `LinPumpHyst`, `LinTempMin/Max`, `LinColdMin`, `LinColdMs`, `LinPMax`, `LinPumpP`, `LinOffDelay`, `LinChecksum`, `LinSlotMs`, `LinBusFault`, `LinArmMs`, `LinSymMax`, `LinDelivRpm`, `LinRxCheck`, `ChgPlugSrc`, `ChgPlugSet` | off; Auto: on < 500 / off > 550 rpm, 0..125 degC (-15 degC for 5 s), < 20 bar, 800 ms; enhanced; 10 ms slot; 1.8 s; not-delivering check 1000 / 1500 ms below 100 rpm; replies checked; no charge plug | Ian `330E_LIN_OIL_PUMP.md` |
| Inverter torque CAN | `CanCmdId`, `CanStatId`, `CanTxEnable`, `TqInterv`, `TqRamp` | placeholder frames, TX off | Q5, Q11 |
| Applied clutch pressure | `ClutchApplyP` | 16500 mbar (valve fully open) | bench |
| Engage fast close | `EngageCloseGrad` (20-50 bar/s), `EngageSyncMs` (50-100 ms), `EngageTqZero` | 30 bar/s, 80 ms, 10 Nm | Ian QA2, bench |
| Engage fill/ramp | `EngageFillEq`, `EngageRamp` | 50 ms, 300 ms | bench |
| Gearbox type (tables, gear-in last clutch, ch7 default, LIN pump) | `GearboxType` | 0 = 8HP75 330e | Ian QA2, F11 diff |
| Channel 7 (K0 / TCC) | `Ch7Mode`, `K0Press`, `TccPress`, `TccOnRpm`, `TccOffRpm`, `TccMinGear`, `TccRamp` | variant default (F11 TCC, 330e K0); K0 0 mbar (open); TCC 6000 mbar, 1500 / 1200 rpm, from 2nd, 200 mbar/10 ms (all TBD) | bench |
| Park release | `ParkRelRpm`, `ParkRelP`, `ParkRelHold`, `ParkPNDelay` | 700 rpm, 17000 mbar, 2000 ms, 150 ms | Ian 11 Q9d [C] |
| Park outputs | `ParkCtrl`, `PzylCurrent`, `ParkMagMin`, `ParkMagMax` | off, 550 mA, 100 ms, 1000 ms | Ian 12.1, bench |
| Gear-in pressure source | `GearInMinP`, `GearInPumpMs`, `GearInMotRpm` | 8000 mbar, 1000 ms, 100 rpm | Ian 11 Q9d [I] |
| Pump fault reaction | `PumpFaultAct`, `PumpFaultRpm` | report, 600 rpm | Ian 11 Q9a [I] |
| Manual gear / tips | `ManualGear`, `TipOverride`, `Tip12Deb` | Selector, 0 (tips ignored in Auto), 30 ms | Damien |
| CAN1 partner | `EcuType` | 0 = None/VCU | OEM ECUs later |
| F11 torque frame | `F11TqTx` | off | Ian F11 diff, live log |
| Interlock open thresholds | `IlOpenLow`, `IlOpenHigh` | 60 mA, 790 mA | Ian QA2 |
| CAN loss behaviour | `CanLossMode`, `CanLossTq` | 1 = hold gear, 600 Nm | Damien (decided 2026-11-23) |

### Setup parameters (web interface dropdowns)

| Parameter | Values | Default | What it does |
|---|---|---|---|
| `GearboxType` (id 9, was `Variant`) | 0 = 8HP75_330e (GA8P75HZ), 1 = 8HP45_F11 | 0 | Selects the per-gearbox data: EDS curves and fill items (`calset.cpp`), the gear-in role table (Ian QA2 Q1), the channel 7 default, whether the LIN pump exists (F11: bus forced off) and the F11 0x0B0 frame. |
| `Ch7Mode` | 0 = Variant, 1 = TCC, 2 = K0, 3 = ForceOpen | 0 | Channel 7 (K0 / TCC slot): F11 lock-up, 330e K0 (held at `K0Press`, 0 = open), or always open on either gearbox. `ch7Act` shows the result. |
| `ManualGear` | 0 = Selector, 1 = P, 2 = R, 3 = N, 4..11 = 1st..8th | 0 | Manual mode only: picks P / R / N or a gear in D instead of the selector. Gear-in is still to 1st (or R) through the engage logic, then the TCU steps one gear at a time, each step inside `ManMinRpm` / `ManMaxRpm` (it waits rather than force a step), with the engage output-speed limit, `TopGear`, the CAN-loss rules and the interlock unchanged. Tips are ignored while a gear is picked. |
| `EcuType` | 0 = None/VCU | 0 | Placeholder: which CAN1 protocol (`canif.cpp`). OEM engine ECUs will be added later. |
| `ShifterType` | 0 = CAN, 1 = Analog | 0 | CAN: selector in byte 0 and tips in byte 6 of the `CanCmdId` frame (placeholder). Analog **placeholder mapping**: `park_in` high = P; `notpark_in` high = R/N/D from `ana_in` (below `ShAnaR` = R, below `ShAnaN` = N, else D); both or neither high, or `ana_in` outside `ShAnaMin..Max` = N plus a `SHIFTER` fault; `din1` = tip up, `din2` = tip down. Inputs must be stable for 20 ms. |
| `ShiftMode` (replaces `GearSelMode`) | 0 = Auto, 1 = Manual, 2 = VCU | 0 | Auto: TCU shifts on `AutoUpRpm` / `AutoDownRpm`. Manual: one gear per tip; a tip down is refused if the lower gear would put the input above `ManMaxRpm`, a tip up if the higher gear would put it below `ManMinRpm`; the lugging downshift (`AutoDownRpm`) still happens on its own; interlock, `TopGear` and CAN-loss rules are unchanged. VCU: the gear request in byte 1 of the CAN frame (the old default). Tips (shifter or the 12 V up / down inputs, debounced over `Tip12Deb`, one gear per press) act in Manual, and in Auto only with `TipOverride` > 0: a tip shifts one gear and Auto stays off for that many ms. Never in VCU mode. |

## Open questions

### For Ian

From the control spec §7:

* **Q1** Lead-frame pinout and solenoid <-> pin (`330E_MECHATRONIC_PINOUT.md` §4).
* **Q2** VFS index <-> valve (only to correlate OEM logs).
* **Q3** Speed-sensor teeth: 60 in the firmware vs 28/40 public.
* **Q4** ATF sensor R vs T.
* **Q5** Inverter torque-reduction interface and latency.
* **Q6** IEP LIN frames: answered in `330E_LIN_OIL_PUMP.md` (see "LIN oil
  pump"); remaining items are listed there.
* **Q7** `_EL` vs Normal sets for an EV (`make cal CAL_SET=el` is ready).
* **Q8** Fill adaptation.
* **Q9** Exact OEM transition conditions in some states (`FUN_0010831c`).
* **Q10** CG208 PWM frequency and dither amplitude.
* **Q11** Motor-to-gearbox torque sign and ratio for `m_t`.

Raised while writing this firmware. Ian answered these in
`330E_SAM_QA2.md` (23 Nov 2026); the firmware was checked against it item
by item. Labels as in his notes: [C] confirmed in code or cal/A2L, [I]
inferred, [H] hypothesis.

* **Gear-in last clutch (QA2 Q1)** - resolved [C]. Roles {last, V1, V2}:
  330e (tables 0x4C214 / 0x4C21E / 0x4C228) 1st {B, A, C}, R {B, A, D};
  F11 (0x5BA08 / 0x5BA12 / 0x5BA1C) 1st {C, A, B}, R {D, A, B}.
  `GearboxType` parameter (was `Variant`), default 0 = 8HP75 330e. V2 starts `KL_TVKZUCGV2_n` (300 ->
  50 ms) after V1. Since Damien's OK the F11 EDS and fill tables are per
  variant too (see "Calibration tables").
* **Interlock (Q2)** - resolved [C]. On commanded coil currents: open =
  A/B below 60 mA, C/D/E above 790 mA (`IEDS_STOER_VUNPLAUSI*`). Every
  clutch outside from-gear + to-gear must be open; 4 non-open allowed
  during a swap (OEM monitor state 4), never 5.
* **GLS units, start and delays (Q3)** - resolved [C]. KP 1 count = 0.02
  mbar per rpm/s, KI 1 count = 0.02/64 mbar per rpm/s per 10 ms, output
  x50/64, clamp +-3 bar. Phase machine after `FUN_0010f12e`: wait state
  left when the owning phase starts (KZU ramp 2, or the KAB shift-pressure
  phase) or by the start criterion once KZU ramp 1 runs (`KL_NDGLSPOSHS`
  350 rpm / `KL_NDGLSNEGHS` -160..-300 rpm on upshifts, `ND_GLSRS` 150 rpm
  on 2->1); then the delay `KL_TVGLSKZUCG` (180-200 ms) or
  `KL_TVGLSKABCG` (250/150/50 ms) on a timer held at 0 until that phase
  starts. The thresholds come from the cal, not parameters. [I] The OEM
  power-upshift condition on the main torque intervention is not
  required, and the RS KZU "ramp 2" is taken as `KZU_HOLD_SYNC`.
* **Downshift KAB hold (Q4)** - implemented as Ian's reading, **[I]**:
  KAB fully applied for `KF_TVKABMTNMOTRS21`, hold `KF_PKAB21 +
  KL_POHKAB21` (0.4-1.0 bar) until `KF_TOHKABMTNT21` before the end of
  the KZU quick fill, then `KL_TRAHKAB21` (100 ms) x `F_TRAHKAB21` down to
  `KF_PKAB21` x `F_PRAHKAB21`. **Open (Ian Q4):** the code that adds
  `KF_PKAB21` and `KL_POHKAB21` is not traced; the candidate
  `FUN_0010d270` is not yet tied to the KAB (vs KZU) object. Using
  `KF_TOHKABMTNT21` as the hold end is also [I].
* **Upshift takeover (Q5)** - no separate takeover ramp: on-coming
  `KF_PGOKZUMTNT12`, off-going `KL_PGOMTKAB12`. Ramp 3 starts
  `KL_TSYNMTRAKZU12` before sync with `KL_PGRASYN12`, `T_SYN12` (300 ms)
  in sync, then `KL_PORASYN12` to PMAX (450 ms). `T_RAMAX12` (10 ms) is
  the timeout ramp only. **Open (Ian Q5):** "no takeover map" comes from
  an A2L name/description search, not a code trace [C for the absence in
  the A2L, I for the behaviour]. `KL_PGOMTKAB12` is a gradient *offset*;
  its base gradient is not identified and is taken as 0 [H].
  `KL_FPGRASYN12` (n_ab-gradient factor) is not applied.
* **Gear-in close (Q6)** - resolved. OEM [C]: `KL_PGRASYN_n` (1-15 bar/s)
  to sync, `T_SYN_n` (200 ms) in sync, `KL_PORASYN_n` to PMAX (~1.5 s
  after sync on N->1). Fast close [I, Ian's opinion]: `EngageCloseGrad`
  (30 bar/s, 20-50) only with no slip for `EngageSyncMs` on valid
  sensors, `|m_t| <= EngageTqZero` (TCU asks for 0 Nm until clamped,
  status bit 3) and adequate line pressure.
* **Coast (Q7)** - resolved [C]: no separate coast ROM sets; runtime
  zug/schub from the torque sign, negative-torque points of the same maps,
  GLS on KAB for coast upshifts / power downshifts and on KZU otherwise
  (sequence table @0x58998). `TAB_IKABENTLEER` is all zeros (not used).
  Needs checking against an OEM coast-shift log.
* **CAN loss (Q8)** - resolved, **Damien's decision 2026-11-23**:
  `CanLossMode` default 1 = hold gear at `LineMax`, no upshifts, step down
  as the car slows; neutral only for hydraulic/electrical faults.
  `CanLossTq` default 600 Nm = `D_m_IstVM_Default` [C]. **Open (Ian Q8):**
  the OEM fault-to-reaction table (KFC -> KMN) was not found, so the
  exact OEM reaction to `KFCx_ENGSIGTRQ` is [H]/[I].
* Not implemented from Q1: the gear-in GLS on the power-flow clutch
  (`SWI_GLSGE`, `KF_WNGTNDSYNCGGE_n`, `T_VGLSKZU_n`, `ND_GLSGE_n`) and the
  gear-in ramp lead `T_SYNRAKZU_n`.

### LIN oil pump

The driver follows Ian's read of the OEM driver, `330E_LIN_OIL_PUMP.md`
(330e `CDD_IEP_Control`), which replaces the guesses from Damien's bench
summary (no timed cold-start phases, no echo: byte 6 is a CRC). Labels are
Ian's [C]/[I]/[H].

What is implemented:

* 19200 baud, TCU master, 8-byte frames. Schedule 0x30, 0x32, 0x30, 0x33,
  0x30, 0x34 on `LinSlotMs` slots (10 ms [I]), so 0x30 every 20 ms. PIDs
  F0 / 32 / 73 / B4 [C].
* 0x30 [C]: b0 0xAA off / 0x55 run (0xCC / 0x33 unused); b1 flags (0x02
  clear errors for exactly one frame, at bus start and on `FaultClear`,
  then 400 ms without diagnosis; 0x04 limp and 0x08 warm-up defined but
  not requested yet); b2 oil temperature + 50; b3 after-run 0; b4-5 0; b6
  CRC-8 over b0 (poly 0x1D, init 0, no XOR: AA -> 73, 55 -> B7); b7 alive
  counter 0-15.
* 0x32 [C]: EB0/EB1, operating point, speed x 32 rpm, current 0.2 A/bit,
  voltage 0.1 V/bit (14.0 V until a valid frame), CRC-8 over b2, alive.
  0x33 (temperature raw, 32-bit ID) and 0x34 (HW / SW version, IEP85
  check) are decoded raw.
* Receive checks as the OEM [C]: LIN checksum; 0x32 CRC-8 (2 bad in a row
  = fault); alive counter repeated twice or jumping by more than 2 = fault,
  frame dropped; no valid 0x32 for `LinBusFault` (1.8 s) = bus fault. Fault
  classes use the OEM `ctl_IEP_Status` masks (EB0 0x40/0x80 = permanent
  level-2 safety fault, 0x10/0x20 = pump comms fault).
* Run rule (330e) [C values, I for how they combine]: input speed below
  `LinPumpRpm` 500 rpm (hysteresis `LinPumpHyst` 50), sump `LinTempMin` ..
  `LinTempMax` (0..125 degC), commanded line pressure below `LinPMax`
  (20 bar); keeps running `LinOffDelay` (800 ms, `D_t_IEP_ShutdownDelay`
  [I: use]) after the rule drops. `LinPumpP` (4 bar, `KF_PIEP_SYS`) is the
  pressure the OEM assumes from the pump; it is shown (`lpPress`) but not
  used by the line-pressure model yet.
* `GearboxType = F11`: the bus stays off (the F11 8HP45 has no LIN pump
  [C]). `LinPumpEnable` stays 0 by default (no transceiver on V4).
* From Ian's section 11 (23 Nov 2026): 0xAA -> 0x55 directly, no settle
  phase [C]; status ignored for 1 s after the bus starts [I] and 400 ms
  after a clear [C]; "not delivering" armed `LinArmMs` (1000 ms,
  `KL_TVERZSYMIEP`) after the run command and set when the pump has
  reported less than `LinDelivRpm` until `LinSymMax` (1500 ms,
  `KL_TMAXSYMIEP`); on < 500 / off > 550 rpm, not above 20 bar, 0..125
  degC or down to -15 degC for 5 s at most; P does not switch it off;
  charging plug in -> LIN asleep (`ChgPlugSrc` = Param + `ChgPlugSet`
  for now; the input is TBD); no ECU limit on the pump voltage (it flags
  its own, EB1 bit 2), diagnosis blanked below `UvLimit` (7.4 V); alive
  counter low nibble only, starts at 0 at power-on, not reset when the
  bus restarts [C/I]; the 0x30 never depends on the replies, and
  `LinRxCheck = 0` stops the replies raising faults.
* The document itself is not copied into the repository.

Answered by Ian (closed), section 11: start-up (no settle), pump fault
reaction (motor speed through the mechanical pump, `PumpFaultAct`), run
thresholds, cold allowance, charging plug, voltage, alive counter, replies
optional, gear-in and park release pressure. Earlier: setpoint (none; operating point only), byte 0
meaning, byte 2 (oil temperature, not a phase), byte 6 (CRC, not an
echo), the cold-start / phase-2 timing (not in the firmware), the "20 ms
settle delay" (not found; the slot timing gives 20 ms between 0x30s),
0x32 byte layout, meaning of 0x40 (level-2 safety, not comms), 0x33/0x34
contents, alive counter rules, when the pump runs, F11 (no pump).

Still open:

1. **LIN channel / checksum.** Which SH725x LIN/SCI channel the OEM uses
   was not found, nor whether its hardware does the checksum; checksum
   type is [H: enhanced] (`LinChecksum`, classic selectable).
2. **b4 / b5 scaling** (current 0.2 A/bit, voltage 0.1 V/bit) is from the
   A2L; check on the bench against a meter.
3. **0x33 pump temperature scaling**: the OEM curve looks odd
   (raw 12..199 -> 120..150 degC), so `lpTempRaw` is shown raw only.
4. **Slot time** 10 ms is [I].
5. Go-to-sleep handling (no LIN sleep frame found; SLEEP is sent as 0xAA),
   and the meaning of EB0 bits 6/7 inside the pump.
6. **Fault reaction** - answered (Q9a): the OEM raises the motor speed so
   the mechanical pump carries the oil supply. `PumpFaultAct` (default 0 =
   report only; 1 = ask the VCU for `PumpFaultRpm`; 2 = no new gear-in).
   The schedule always keeps running and `FaultClear` sends one clear
   frame. The motor speed request only goes out with `CanTxEnable`.
7. **Gear-in waiting for the pump** - answered (Q9d): minimum pressure, no
   fixed wait in the OEM; the "pump 1 s or motor turning" rule is Ian's
   recommendation [I] (`GearInPumpMs`, `GearInMotRpm`).
8. Warm-up (b1 0x08: below 16 degC, ends at 60 degC, < 8 km/h, <= 8 bar,
   pump SW >= 0x0705) and limp (0x04) requests are not generated yet.
9. LIN hardware on the next board (transceiver part, SLP_N on PD11).
10. Charging plug input: pin TBD (`ChgPlugSrc = Input` reads "not
    plugged" until it exists).
11. Air-ingestion guard (30 s on a gradient / 500 s at speed) and the
    car-wash cut-off are not done.

### Valves and speed sensors (Ian section 12) - bench items for Damien

* Valve map [I]: for an EV drop K0, leaving 8 proportional valves A..E,
  SYS, POS, PZYL. POS = position valve (switch-over 420 mA). PZYL = park
  cylinder valve = the old PREL (550 mA + cold offset) [I mapping]. The
  park magnet is the on/off NCV8402 on PC7 and is needed with PZYL.
  **Conflict to settle on the bench:** Bob's V4 header calls OUT6 "PREL"
  and the firmware maps POS there (`ChanPOS = 6`); if PREL is PZYL, OUT6
  is PZYL and POS needs another channel (OUT7 on a 330e EV with K0
  dropped, or the second chip). Defaults are unchanged until confirmed.
* OEM OUT7 = Sol1, on/off, 0.95 A peak / 0.4 A hold; identity unproven.
  Not used and not mapped. (This is the OEM ECU's OUT7, not V4 OUT7.)
* F11: 7 proportional (A..E, SYS, TCC) plus an 8th curve that is probably
  POS [I]; no K0, park, POS or Sol1 strings.
* Speed: n_ab, n_t and n6 on both gearboxes, 60 ppr, 30 rpm minimum (the
  `Teeth*` and `SpdMinRpm` defaults). n6 is a safety cross-check only.
* Open for the bench: channel -> pin -> valve map (drive one channel at a
  time), Sol1 identity, OEM OUT10 and OUT12-14, which connector input
  n6 is on.

### For Bob

Resolved by the V4 pin map (RREF, one chip and CS, pins, channel map,
park magnet off the MAX22200, no Sol1 / high-side switch, dividers,
crystal, terminal, speed sensor interface). Still open:

* **PREL**: which hydraulic valve is OUT6? The firmware maps POS there
  (`ChanPOS = 6`) as an assumption; Ian now reads PREL as PZYL (park
  cylinder) [I], see "Valves and speed sensors". On the next board PZYL
  is chip 2 OUT0 (channel 8).
* **TCC on OUT7**: on the 330e this is K0 (held open), on the F11 the
  converter lock-up. Same channel for both variants?
* Revision items: speed sensor RC filter (thresholds to re-check), CAN2
  transceiver on PB5/PB6, LIN transceiver. Placeholders are in the code
  for the second MAX22200 CSB, the third speed input (PA2?), the 12 V
  shift up / down inputs and T15 (`HW_REV 5`, pins TBD; see "Next board
  revision").
* `spd_in_ana` / `spd_out_ana` are sampled by ADC2 only, not by the
  ADC1 scan, so one channel is never converted by both ADCs at once.
* Starter lock on gp_out1/2: which polarity does the vehicle want? The
  firmware drives the output *on* in P/N.
* Which temperature input (TempIn1 2.6k or TempIn2 470R) the 8HP ATF
  sensor uses, and its type (Damien).

## Licence

GPL v3 or later, like libopeninv and the ZombieVerter. See the headers in
each file.
