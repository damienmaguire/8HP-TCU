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
| Aux speed | none | `TeethAux` kept as a parameter only, `nAux` reads 0 |
| vbat | PC0 (IN10) | 0.18 divider, `VbatScale` 18330 mV full scale |
| atf_temp1 / atf_temp2 | PC1 (IN11) / PC2 (IN12) | 2.6k / 470R pull-up to 5 V, then /2; `AtfInput` picks one |
| ana_in | PC3 (IN13) | /2, 0-6.6 V (analog shifter placeholder) |
| din1, din2, park_in, notpark_in | PD4, PD5, PD6, PD7 | active high, external 1.5k pull-down, `INPUT_FLT` |
| LED | PE2 | active low |
| Terminal | USART3 PB10 / PB11 | 115200 |
| CAN1 | PA11 RX / PA12 TX | no remap |
| CAN2 | PB5 / PB6 (remap) | **REV:** no transceiver on V4; only started with `Can2Enable = 1` |
| LIN (aux oil pump) | UART4 TX PC10 / RX PC11, SLP_N PD11 | **REV:** no transceiver on V4; `LinPumpEnable = 0` by default |
| T15 (ignition sense) | not assigned | **REV** |
| Debug | SWD only | JTAG disabled (frees PA15, PB3, PB4) |

There is no Sol1 and no high-side supply switch on V4.

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
| `shifter.*` | Selector and tip up/down from CAN or the V4 inputs (`ShifterType`) | Damien |
| `linpump.*` | LIN master for the aux oil pump: PID, checksum, CRC-8, 6-slot schedule, 0x30 build, 0x32/33/34 decode, OEM receive checks and fault classes, run rule | `330E_LIN_OIL_PUMP.md` |
| `tcu.*` | Top-level mode machine, line pressure, faults, limp | §3, §5.5 |
| `cal_330e.*`, `caltypes.*` | Generated calibration tables and the lookup code | JSON export |
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
  (driver tips with rpm limits and lugging protection), VCU (gear from
  the CAN frame).

Stubbed / not done yet:

* **Park lock** (§5.8): the park-hold magnet (PC7) is held off except for
  the `ParkHoldTest` bench switch; PZYL has no V4 channel by default
  (`ChanPZYL = 255`). `parkRequest` is computed. The release sequence
  (17 bar line, cylinder, magnet pulses <= 1 s) is not written.
* **Electric oil pump (IEP) over LIN**: done to Ian's OEM read (see "LIN
  oil pump"), but untested (no transceiver on V4). Fault reaction is
  report-only.
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
* Fill adaptation (Q8), POS valve logic (fixed `PosCurrent`), K0 (held
  open at 50 mA).

## Parameters for unmeasured items

Nothing that still needs a measurement is hard-coded. These are the
parameters to set once the value is known:

| Item | Parameter(s) | Default (placeholder) | Open question |
|---|---|---|---|
| Solenoid -> MAX22200 channel | `ChanA` .. `ChanPZYL` (0..7, 255 = not fitted) | A..E = 0..4, SYS 5, POS 6 (PREL, assumed), K0 7 (TCC), PZYL 255 | Bob, Q1 |
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
| Aux pump LIN | `LinPumpEnable`, `LinPumpReq`, `LinPumpRpm`, `LinPumpHyst`, `LinTempMin/Max`, `LinPMax`, `LinPumpP`, `LinOffDelay`, `LinChecksum`, `LinSlotMs`, `LinBusFault` | off; Auto: < 500 rpm (+50), 0..125 degC, < 20 bar, 800 ms; enhanced; 10 ms slot; 1.8 s | Ian `330E_LIN_OIL_PUMP.md` |
| Inverter torque CAN | `CanCmdId`, `CanStatId`, `CanTxEnable`, `TqInterv`, `TqRamp` | placeholder frames, TX off | Q5, Q11 |
| Applied clutch pressure | `ClutchApplyP` | 16500 mbar (valve fully open) | bench |
| Engage fast close | `EngageCloseGrad` (20-50 bar/s), `EngageSyncMs` (50-100 ms), `EngageTqZero` | 30 bar/s, 80 ms, 10 Nm | Ian QA2, bench |
| Engage fill/ramp | `EngageFillEq`, `EngageRamp` | 50 ms, 300 ms | bench |
| Gearbox type (last clutch on gear-in) | `GearboxType` | 0 = 8HP75 330e | Ian QA2 |
| Interlock open thresholds | `IlOpenLow`, `IlOpenHigh` | 60 mA, 790 mA | Ian QA2 |
| CAN loss behaviour | `CanLossMode`, `CanLossTq` | 1 = hold gear, 600 Nm | Damien (decided 2026-11-23) |

### Setup parameters (web interface dropdowns)

| Parameter | Values | Default | What it does |
|---|---|---|---|
| `GearboxType` (id 9, was `Variant`) | 0 = 8HP75_330e (GA8P75HZ), 1 = 8HP45_F11 | 0 | Selects the per-gearbox data. Today that is only the gear-in role table (Ian QA2 Q1); the calibration set is the 330e one for both until Ian's F11 comparison is cleared. |
| `ShifterType` | 0 = CAN, 1 = Analog | 0 | CAN: selector in byte 0 and tips in byte 6 of the `CanCmdId` frame (placeholder). Analog **placeholder mapping**: `park_in` high = P; `notpark_in` high = R/N/D from `ana_in` (below `ShAnaR` = R, below `ShAnaN` = N, else D); both or neither high, or `ana_in` outside `ShAnaMin..Max` = N plus a `SHIFTER` fault; `din1` = tip up, `din2` = tip down. Inputs must be stable for 20 ms. |
| `ShiftMode` (replaces `GearSelMode`) | 0 = Auto, 1 = Manual, 2 = VCU | 0 | Auto: TCU shifts on `AutoUpRpm` / `AutoDownRpm`. Manual: one gear per tip; a tip down is refused if the lower gear would put the input above `ManMaxRpm`, a tip up if the higher gear would put it below `ManMinRpm`; the lugging downshift (`AutoDownRpm`) still happens on its own; interlock, `TopGear` and CAN-loss rules are unchanged. VCU: the gear request in byte 1 of the CAN frame (the old default). |

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
  50 ms) after V1. Only this table is variant-specific; Ian's wider F11 vs
  330e comparison waits for Damien's OK.
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
* The document itself is not copied into the repository.

Answered by Ian (closed): setpoint (none; operating point only), byte 0
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
6. **Fault reaction (safety, no OEM answer yet):** retry and report only.
   On any pump fault class the firmware posts `LINPUMP`, shows `lpClass`,
   keeps the schedule running and changes nothing in the TCU (no neutral,
   no engage block). `FaultClear` sends one clear frame.
7. **Gear-in waiting for the pump (safety, no OEM answer yet):** not done;
   the engage "line pressure OK" input only checks the commanded
   pressure.
8. Warm-up (b1 0x08: below 16 degC, ends at 60 degC, < 8 km/h, <= 8 bar,
   pump SW >= 0x0705) and limp (0x04) requests are not generated yet.
9. LIN hardware on the next board (transceiver part, SLP_N on PD11).

### For Bob

Resolved by the V4 pin map (RREF, one chip and CS, pins, channel map,
park magnet off the MAX22200, no Sol1 / high-side switch, dividers,
crystal, terminal, speed sensor interface). Still open:

* **PREL**: which hydraulic valve is OUT6? The firmware maps POS there
  (`ChanPOS = 6`) as an assumption; POS and PZYL are both at 0 mA today.
  PZYL (park cylinder valve) has no V4 channel.
* **TCC on OUT7**: on the 330e this is K0 (held open), on the F11 the
  converter lock-up. Same channel for both variants?
* Revision items: speed sensor RC filter (thresholds to re-check), CAN2
  transceiver on PB5/PB6, LIN transceiver, T15 / ignition sense.
* `spd_in_ana` / `spd_out_ana` are sampled by ADC2 only, not by the
  ADC1 scan, so one channel is never converted by both ADCs at once.
* Starter lock on gp_out1/2: which polarity does the vehicle want? The
  firmware drives the output *on* in P/N.
* Which temperature input (TempIn1 2.6k or TempIn2 470R) the 8HP ATF
  sensor uses, and its type (Damien).

## Licence

GPL v3 or later, like libopeninv and the ZombieVerter. See the headers in
each file.
