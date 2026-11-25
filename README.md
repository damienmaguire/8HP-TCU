# 8HP-TCU

Open-source transmission controller for the ZF 8HP automatic gearbox, for EV conversions and custom builds. The board uses an STM32F107VCT6. The software is built the same way as the OpenInverter ZombieVerter.

Forum: https://openinverter.org/forum/

<img width="1624" height="928" alt="Stm32_TCU_V5" src="https://github.com/user-attachments/assets/684fc972-fac7-4564-ae98-45893f91e95d" />



Connector housing, header, plug and pins are the same as ZombieVerter VCU.

## Gearboxes

8HP45 / 8HP70 / 8HP75 / 8HP90. None has been run on a gearbox.

| Model | Came from | Where it stands |
|---|---|---|
| 8HP45 | BMW F11, N47 engine. No LIN pump; start-stop uses a pressure accumulator. The F11 v058 calibration is the target and has its own shift maps. An older a81 v095 calibration for the same box is data only. | Target, plus one extra calibration. Untried. |
| 8HP70 | BMW N57 and N63 engines. a81 v095 calibrations. Shift maps stay the 330e ones. | Calibration data only. Untried. |
| 8HP75 | BMW 330e, GA8P75HZ. Hybrid, with a K0 clutch and an electric LIN oil pump. Calibration Q030. | Target. Untried. |
| 8HP90 | BMW N74, and the RR4 with the N74. a81 v095 calibrations. Shift maps stay the 330e ones. | Calibration data only. Untried. |

`GearboxType` names, in order: `8HP75_330e`, `8HP45_F11`, `8HP45_F11N47_v095`, `8HP70_N57_v095`, `8HP70_N63_v095`, `8HP90_N74_v095`, `8HP90_RR4N74_v095`. On the data-only rows the size figures are not used by the control yet.

## Shifters

`ShifterType` picks the lever. The CAN levers are ported from the ZombieVerter (`Stm32-vcu`, branch `BMW-F11-Dev`, `src/shifters/`). Host tests cover them. None has been tried on hardware. Messages the TCU sends stay off until both `LeverTxEnable` and `CanTxEnable` are on.

| Setting | Lever | Interface | Where it stands |
|---|---|---|---|
| 0 `BMW_GWS` | BMW F-series GWS, the monostable lever used on the F30 and the F11. One setting covers both. Reads `0x197`. When sending is on, it sends the lever display on `0x3FD`. From ZombieVerter `F30_Lever`. | CAN | In the firmware. Host tests include an F11 capture. Not tried on a car. |
| 1 `Analog` | No part chosen. Park and not-park switches, an analogue voltage for R / N / D, and two tip switches. | Board pins | Placeholder. Untried. |
| 2 `BMW_E65` | BMW E65 lever. Reads `0x192` (park button, full forward, full back). It sends nothing back to the lever. | CAN | Ported. Host tests only. Untried on hardware. |
| 3 `JLR_G1` | JLR rotary knob, first generation. Reads `0x312`, sends `0x3F3`. | CAN | Ported. Host tests only. Untried on hardware. |
| 4 `JLR_G2` | JLR rotary knob, second generation. Reads `0x0E0`, sends `0x02C`. | CAN | Ported. Host tests only. Untried on hardware. |
| 5 `PKP2300` | Blink Marine PKP-2300-SI keypad, CANopen node `0x15`. Reads `0x195`, sends the lamp message `0x215`. | CAN | Ported. The regen, heater and charge display from the VCU port are not included. Untried on hardware. |
| 6 `VCU_Frame` | No lever. The vehicle controller puts P / R / N / D, and tip up / tip down, in its command frame. | CAN | Placeholder frame. Untried. |

The 12 V shift-up and shift-down inputs are extra tip buttons. They are not a gear selector, and they work alongside whichever lever you pick.

## Status

In development, and not tried on hardware. The firmware is in this repository, with host tests, and it has not been run on a board or on a gearbox. Leave it off a car until that has happened.

Written so far, still untested on the hardware:

- Solenoid and line-pressure control, and the shift maps.
- Gear levers, in the table above. For the BMW GWS, the TCU can send the lever display message `0x3FD`. That sending stays off until you enable it.
- Park lock. The outputs stay off until you enable them, and the wiring still needs a check on the bench.
- LIN oil pump, for the 330e only. Off until you enable it.

What each part does, the settings, and what is still open: [Software/firmware/README.md](Software/firmware/README.md).

## What is in the repo

- `Hardware/` — V2 board drawings, photos and pinouts, and the adapter manufacturing files (`TCU_Adapter_BoardV1.zip`). V4 STM32F107VCT6 board added today.
-  Decided to change the proportional valve drivers on the V4 board from the MAX22200 to the Infineon TLE92466ED and go for a V5.
- `TCM_Pinout.pdf` — pinout of the GA8P75HZ gearbox connector.
- `Software/firmware/` — STM32 firmware, Makefile, Code::Blocks project (`tcu8hp.cbp`) and host tests. Two submodules: [libopeninv](https://github.com/damienmaguire/libopeninv) and [libopencm3](https://github.com/jsphuebner/libopencm3).
- `CAN/` — F11 8HP CAN notes: message IDs, a CAN database (`.dbc`), and a map of the messages a controller has to handle.

## Build

<img width="2304" height="4096" alt="8hp_sw1" src="https://github.com/user-attachments/assets/072ac935-9eab-4df6-b3f7-c79d3e288967" />


Ubuntu 22.04 or later. The build writes a `.bin` and a `.hex`. The image is linked at `0x08001000`, with room for a 4 KiB bootloader in front, the same layout as other OpenInverter boards. There is no flash program in this repo.

```
sudo apt-get install gcc-arm-none-eabi g++ make git
git clone --recursive https://github.com/damienmaguire/8HP-TCU.git
cd 8HP-TCU/Software/firmware
make get-deps
make              # V4 board: stm32_tcu.bin and stm32_tcu.hex
make HW_REV=5     # V4 rev B: stm32_tcu_revb.bin and stm32_tcu_revb.hex
make Test && test/test_tcu
```

`make get-deps` fetches the submodules and builds libopencm3 the first time. In Code::Blocks, open `tcu8hp.cbp`. A workflow file sits at `Software/firmware/ci/CI-build.yml` and is not enabled yet. To turn it on, move it to `.github/workflows/`.

## Contributing

Pull requests are welcome. Say what you actually ran: the host tests, a bench check, or neither.

## Licence

GNU GPL v3 or later. See [LICENSE](LICENSE).

Copyright Damien Maguire. The CAN gear-lever code is ported from his ZombieVerter VCU (`Stm32-vcu`, branch `BMW-F11-Dev`, `src/shifters/`), also GPL v3. The V2 board artwork is credited on the board to D. Maguire / J. Huebner (evbmw.com / openinverter.org).
