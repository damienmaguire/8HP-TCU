# 8HP-TCU

Open-source transmission controller for the ZF 8HP automatic gearbox, for EV conversions and custom builds. The board uses an STM32F107VCT6. The software is built the same way as the OpenInverter ZombieVerter.

Forum: https://openinverter.org/forum/

<img width="640" alt="TCU board" src="https://github.com/user-attachments/assets/2486fc5e-2951-46a5-b27e-60c1235bf647" />

Connector housing, header, plug and pins are the same as ZombieVerter VCU.

## Gearboxes

Two gearboxes are the ones this is written for:

- **BMW 330e, GA8P75HZ (8HP75).** Hybrid box. It has an electric oil pump on LIN, used when nothing is turning the mechanical pump.
- **BMW F11 8HP45.** Driven by the engine. It has no LIN pump. Start-stop holds pressure in an accumulator instead.

`GearboxType` selects which one the firmware uses. A few other 8HP size rows are included as calibration data only. They have not been tried.

## Status

In development, and not tried on hardware. The firmware is in this repository, with host tests, and it has not been run on a board or on a gearbox. Leave it off a car until that has happened.

Written so far, still untested on the hardware:

- Solenoid and line-pressure control, and the shift maps.
- CAN gear lever (BMW GWS). With sending turned on, the TCU sends the lever display message `0x3FD`. Sending stays off until you enable it.
- Park lock. The outputs stay off until you enable them, and the wiring still needs a check on the bench.
- LIN oil pump, for the 330e only. Off until you enable it.

What each part does, the settings, and what is still open: [Software/firmware/README.md](Software/firmware/README.md).

## What is in the repo

- `Hardware/` — V2 board drawings, photos and pinouts, and the adapter manufacturing files (`TCU_Adapter_BoardV1.zip`). The firmware pin map is for a later V4 board, and a rev B of that board. Those drawings are not in this folder.
- `TCM_Pinout.pdf` — pinout of the GA8P75HZ gearbox connector.
- `Software/firmware/` — STM32 firmware, Makefile, Code::Blocks project (`tcu8hp.cbp`) and host tests. Two submodules: [libopeninv](https://github.com/damienmaguire/libopeninv) and [libopencm3](https://github.com/jsphuebner/libopencm3).
- `CAN/` — F11 8HP CAN notes: message IDs, a CAN database (`.dbc`), and a map of the messages a controller has to handle.

## Build

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
