# 8HP TCU firmware (STM32F107VCT6)

Open-source transmission control for the ZF 8HP (first target: BMW GA8P75HZ
in EV conversions). Built on the openinverter libraries the same way as the
ZombieVerter VCU (Stm32-vcu): libopeninv parameters, terminal, CAN map and
SDO on top of libopencm3.

> **Status: first cut, not run on hardware.** It builds and the host unit
> tests pass, but nothing has been tried on Bob's board or a gearbox yet.
> Every pin is a placeholder (see `include/digio_prj.h`), and several values
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
by `tools/gen_cal.py`. Only the tables the firmware uses are emitted (450
items, about 7,200 values, ~39 KB of flash), not all 6,650 values. Every
table carries its A2L name and ROM address in a comment. To regenerate:

```
make cal CAL_JSON=/path/to/330e_sas_eds_cal.json            # Normal set
make cal CAL_JSON=/path/to/330e_sas_eds_cal.json CAL_SET=el # _EL set (Q7)
```

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
* **Tasks** (Stm32Scheduler on TIM4): 1 ms dither + SPI updates, 10 ms
  control + CAN, 100 ms faults, ATF, battery voltage.

### Modules

| File | What it does | Source |
|---|---|---|
| `gears.*` | Gear/clutch table, ratios, single-swap off/on elements, next step (adjacent only in v1) | §4, §5.1 |
| `interlock.*` | Hard check before any output, on the commanded coil currents: a clutch is open only below `IlOpenLow` (A/B, 60 mA) or above `IlOpenHigh` (C/D/E, 790 mA); every clutch outside the allowed set must be open; <= 3 applied and forming a gear, or <= 4 inside the from/to union during a swap; never 5 | §4, Ian QA2 |
| `sas.*` | Shift state machines with the OEM state numbers. Upshift KZU 1,2,4,9,10,13 + KAB 1,2,3,4,9,16; downshift KZU 1,2,5,9,12,18 + KAB 10..16; timeout 14 (KL_TMAXCG); power-on/coast picked at shift start from the torque sign; GLS PI on KZU or KAB with the OEM scaling and +-3 bar clamp; torque intervention KF_MEHAUPT / KF_MENACH | §5.3-5.6, `330E_SAS_FILL_RAMP.md`, Ian QA2 |
| `engage.*` | N->1 / N->R: V1/V2 quick fill, then the last clutch (per `Variant`) fills and closes, fast only with confirmed no-slip, torque near 0 and line pressure OK | §5.7, Ian QA2 |
| `eds.*` | Pressure -> current (KL_EDSKVA..K0), KF_KIEDSCG* temperature correction, slew limit, firmware dither | §6, `330E_EDS_PWM.md` |
| `max22200.*` | MAX22200/MAX22200A SPI driver | data sheet Rev 1 (3/25) |
| `valveout.*` | Valve-to-channel map, two chips, fault mapping | |
| `speedsens.*` | Edge period -> rpm with tooth count as a parameter; ATF table lookup | §2, pinout §1 |
| `tcu.*` | Top-level mode machine, line pressure, faults, limp | §3, §5.5 |
| `cal_330e.*`, `caltypes.*` | Generated calibration tables and the lookup code | JSON export |
| `stm32_tcu.cpp`, `hwinit.cpp`, `terminal_prj.cpp` | Firmware glue: params, SPI, EXTI, CAN, terminal | ZombieVerter |

### Solenoid driver (MAX22200)

The valves are driven by MAX22200AETJ+T octal solenoid drivers over SPI
(data sheet Rev 1, 3/25; register, bit and figure numbers are cited in
`include/max22200.h`):

* Every valve channel runs in **current-drive regulation (CDR), low side**.
  The chip regulates the **peak** current of each chopping cycle
  (`I = HOLD / 127 x IFS`). There is no shunt and no software current loop.
* `IFS = 15 kV / RREF` (HFS = 0, up to 1 A) is set by Bob's RREF resistor,
  so it is the parameter `MaxIfs` (default 1000 mA). One HOLD step is then
  about 7.9 mA. `MaxHfs = 1` halves full scale for finer steps.
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
  Faults on K0, POS, PZYL, Sol1 or the park magnet are only reported.
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
* Assumes one chip select per chip (no daisy chain) and channels switched
  over SPI (TRIGA/TRIGB unused).

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

CAN1 (vehicle/VCU) and CAN2 (logging) both at 500 kbit/s by default. All
frame layouts below are **placeholders** until the VCU/inverter side is
agreed (Q5). Values little endian.

* RX `CanCmdId` (default 0x300): byte 0 selector (0=P 1=R 2=N 3=D),
  byte 1 gear request (0 = hold, 1..8, used when `GearSelMode = VCU`),
  bytes 2-3 actual motor torque Nm (signed), bytes 4-5 motor rpm (signed).
  If it stops for `CanTimeout` ms the TCU uses `CanLossTq` for line
  pressure and follows `CanLossMode`: 0 (default) goes to neutral; 1 holds
  the gear at `LineMax`, refuses upshifts and only steps down as the car
  slows. No gear-in is started while the frame is missing.
* TX `CanStatId` (default 0x301, only if `CanTxEnable = 1`): gear, mode,
  target gear, fault, torque intervention request Nm (signed), ATF + 40,
  flags (bit 0 shifting, 1 park request, 2 safe state, 3 hold motor
  torque near 0 for gear-in, 4 CAN-loss gear hold, 5 coast shift). The torque intervention
  request is the hook for the inverter: how it reaches the inverter is
  open (Q5).
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
* Speed sensors (EXTI + free-running timer), ATF table, battery voltage.
* CAN1 command/status placeholders, CAN2 logging, CAN map, SDO, terminal.
* Simple internal shift schedule (`GearSelMode = Auto`, input-speed
  thresholds) for bench tests; the normal mode takes the gear from the VCU.

Stubbed / not done yet:

* **Park lock** (§5.8): PZYL, park magnet and Sol1 outputs exist and are
  mapped, but are held at 0 A. `parkRequest` is computed. The release
  sequence (17 bar line, cylinder, magnet pulses <= 1 s) is not written.
* **Electric oil pump (IEP) over LIN** (Q6): not implemented. In an EV
  there is no line pressure at standstill without it.
* **Inverter torque interface** (Q5): only the request value in the status
  frame.
* Skip shifts and the OEM routing tables (0x51418 / 0x51350): v1 only does
  adjacent shifts.
* `TAB_IKABENTLEER` (0x234c78), the shift-type switch for an off-going
  current ramp, is all zeros in the 330e calibration (`CS_IKABENTLEER`
  -40 degC), so that ramp is never active. It is not implemented; revisit
  if another calibration sets it.
* The SP/SPORT/_EL sets (only Normal is generated; `CAL_SET=el` exists).
* Fill adaptation (Q8), POS valve logic (fixed `PosCurrent`), K0 (held
  open at 50 mA).

## Parameters for unmeasured items

Nothing that still needs a measurement is hard-coded. These are the
parameters to set once the value is known:

| Item | Parameter(s) | Default (placeholder) | Open question |
|---|---|---|---|
| Pinout (all I/O) | `include/digio_prj.h`, `include/anain_prj.h`, `include/hwdefs.h` (SPI, timer) | placeholder pins, marked TBD | Bob |
| Solenoid -> MAX22200 channel | `ChanA` .. `ChanPARK` (chip x 8 + channel, 255 = not fitted) | 0..10 | Bob, Q1 |
| Number of MAX22200s | `MaxChips` | 2 | Bob |
| MAX22200 full scale (RREF) | `MaxIfs`, `MaxHfs` | 1000 mA, off | Bob |
| Chopping (PWM) frequency | `MaxFreq`, `MaxFreqDiv` | 100 kHz / 4 = 25 kHz | Q10 |
| Dither amplitude / period / shape | `DitherAmp`, `DitherPeriod`, `DitherShape` | 30 mA, 8 ms, triangle | Q10 |
| HIT phase for on/off coils | `HitCurrent`, `HitTime` | off | Bob, bench |
| Speed sensor teeth | `TeethIn`, `TeethOut`, `TeethAux` | 60 (OEM firmware value) | Q3 |
| Speed timeout / minimum | `SpdTimeout`, `SpdMinRpm` | 100 ms, 30 rpm | Q3 |
| ATF sensor curve | `AtfSource`, `AtfFixed`, `AtfAdc1..6`, `AtfT1..6`, `AtfAdcMin/Max` | fixed 40 degC; table is a guess | Q4 |
| Battery voltage scaling | `VbatScale` | 23100 mV full scale | Bob |
| Aux pump LIN frames | not implemented (pin `lin_nslp` placeholder) | | Q6 |
| Inverter torque CAN | `CanCmdId`, `CanStatId`, `CanTxEnable`, `TqInterv`, `TqRamp` | placeholder frames, TX off | Q5, Q11 |
| Applied clutch pressure | `ClutchApplyP` | 16500 mbar (valve fully open) | bench |
| Engage fast close | `EngageCloseGrad` (20-50 bar/s), `EngageSyncMs` (50-100 ms), `EngageTqZero`, `EngagePostSync` | 30 bar/s, 80 ms, 10 Nm, 200 ms | Ian QA2, bench |
| Engage fill/ramp | `EngageFillEq`, `EngageRamp` | 50 ms, 300 ms | bench |
| Gearbox variant (last clutch on gear-in) | `Variant` | 0 = 330e | Ian QA2 |
| Interlock open thresholds | `IlOpenLow`, `IlOpenHigh` | 60 mA, 790 mA | Ian QA2 |
| KAB GLS early start slip | `GlsKabSlipUp`, `GlsKabSlipDn` | 350 rpm, 150 rpm | Ian QA2 |
| CAN loss behaviour | `CanLossMode` | 0 = neutral | **Damien** |

## Open questions

### For Ian

From the control spec §7:

* **Q1** Lead-frame pinout and solenoid <-> pin (`330E_MECHATRONIC_PINOUT.md` §4).
* **Q2** VFS index <-> valve (only to correlate OEM logs).
* **Q3** Speed-sensor teeth: 60 in the firmware vs 28/40 public.
* **Q4** ATF sensor R vs T.
* **Q5** Inverter torque-reduction interface and latency.
* **Q6** IEP LIN frames, and whether the input-shaft pump alone is enough.
* **Q7** `_EL` vs Normal sets for an EV (`make cal CAL_SET=el` is ready).
* **Q8** Fill adaptation.
* **Q9** Exact OEM transition conditions in some states (`FUN_0010831c`).
* **Q10** CG208 PWM frequency and dither amplitude.
* **Q11** Motor-to-gearbox torque sign and ratio for `m_t`.

Raised while writing this firmware. Ian answered these in his QA2
write-up (round 2, 23 Nov 2026); what the firmware now does:

* **Gear-in last clutch** - resolved. Per variant: 330e (table 0x4C214)
  closes B last for both N->1 and N->R with A+C (1st) / A+D (R)
  pre-filled; F11 8HP (table 0x5BA08) closes C last for 1st and D for R.
  `Variant` parameter, default 0 = 330e to match the calibration in use.
  Only this table is variant-specific; Ian's wider F11 vs 330e comparison
  waits for Damien's OK.
* **Interlock** - resolved. Done on commanded coil currents: open = A/B
  below 60 mA, C/D/E above 790 mA (normally high). Every clutch outside
  from-gear + to-gear must be open; 4 applied is allowed during a swap,
  never 5.
* **GLS units and start** - resolved. KP 1 count = 0.02 mbar per rpm/s,
  KI 1 count = 0.02/64 mbar per rpm/s per 10 ms, output x50/64, clamped
  +-3 bar. KZU start delay `KL_TVGLSKZUCG` (180-200 ms) from ramp 2. KAB
  start delay `KL_TVGLSKABCG` (250/150/50 ms) from the KAB shift-pressure
  phase, or earlier once the slip reaches 350 rpm (up) / 150 rpm (down).
* **Downshift KAB hold** - resolved, unchanged: `KF_PKAB + KL_POHKAB`
  (0.4-1.0 bar), then a `KL_TRAHKAB` (100 ms) ramp to `KF_PKAB`. Upshifts
  use the absolute `KL_PHKABMT` against torque.
* **Upshift takeover gradient** - resolved. No separate gradient:
  on-coming `KF_PGOKZUMTNT`, off-going `KL_PGOMTKAB`. After sync
  `KL_PGRASYN` for `T_SYN` (300 ms), then a ramp to max over `T_RAMAX`.
  The old third phase (`KL_PORASYN`) has been removed.
* **Gear-in gradients** - resolved. Fast close (`EngageCloseGrad`,
  default 30 bar/s, 20-50) only with no slip confirmed for `EngageSyncMs`
  on valid speed sensors, `|m_t| <= EngageTqZero` (the TCU asks for 0 Nm
  until clamped: status flag bit 3) and adequate line pressure. Otherwise
  the OEM 1-15 bar/s gradients, and "in gear" only after 200 ms with no
  slip.
* **Coast** - resolved, implemented as above. Needs checking against an
  OEM coast-shift log.
* **CAN loss** - open, **for Damien**. `CanLossMode` 0 = neutral (current
  default), 1 = hold gear with high line pressure and no upshifts. Ian
  recommends 1, with neutral only for hydraulic/electrical faults (those
  already go to the safe state).
* **Still open:** Ian's QA2 notes file (`330E_SAM_QA2.md`) was not on the
  shared box when this was written, so these changes follow the summary
  passed on with his answers and the raw `qa2/` Ghidra extracts. Worth a
  read-through against his full write-up.

### For Bob

* RREF value (sets `MaxIfs`), and HFS: are any valves better on half scale?
* One chip select per MAX22200 or a daisy chain? Shared CMD? (The driver
  assumes separate CS and a shared CMD.)
* Pins for CS0/CS1, CMD, ENABLE, nFAULT (with pull-up), and TRIGA/TRIGB
  tied low.
* **Which valves are not on a MAX22200?** The firmware assumes all 11
  outputs (A, B, C, D, E, SYS, K0, POS, PZYL, Sol1, park magnet) are on two
  chips. In particular: is the park-lock magnet (OUT11) on a MAX22200 (CDR
  is low side only), is Sol1, is there a high-side supply switch for the
  solenoid bank (OEM PSply) and is there a starter-lock / P-N output? Any
  valve that is not on a MAX22200 is set to 255 in its `Chan` parameter
  and needs its own driver code.
* Two MAX22200s (16 channels for 11 valves) or one plus discrete drivers?
* Current sensing: none is needed now that the MAX22200 regulates current.
* Speed sensor interface: supply (switched? `sens_supply` pin), signal
  conditioning, and which timer/EXTI pins.
* LIN transceiver for the oil pump, and its sleep pin.
* ATF sensor divider and battery voltage divider values.
* Crystal (the firmware assumes 8 MHz -> 72 MHz) and the terminal UART
  pins.

## Licence

GPL v3 or later, like libopeninv and the ZombieVerter. See the headers in
each file.
