# F11 8HP45 — CAN Map for a Standalone CANTcu-style Controller

**Car:** owner's BMW F11 520d (2011), ZF 8HP45 / ITCU2. EV-conversion donor.
**Goal:** what a Damien-Maguire-style standalone transmission controller (CANTcu / 8HP-TCU /
ZombieVerter family) must **transmit**, must **receive**, and can **ignore** to drive this gearbox.
**Sources:** F11 SavvyCAN CAN logs (PT-CAN + PT-CAN2). FlexRay is out of scope — the EGS is
CAN-only. SavvyCAN timestamps wrap every 1 s; periods below use a rebuilt continuous timeline.
**Companion CSV:** [cantcu_can_ids.csv](cantcu_can_ids.csv) (one row per signal).
**Confidence:** **confirmed** / **analysis** / **hypothesis**.

**Capture caveat:** the logs used here are P / R / D creep and warm-up only. A multi-gear
drive log (gears 1–4 +, S/manual, one kickdown) will fill in torque-reduction requests,
actual-vs-target gear, shift-in-progress and clutch-lockup — planned, but the donor engine
is near end-of-life so that log comes late.

Buses: **PT-CAN2 = bus 2**, **PT-CAN = bus 3**.
E2E on protected frames: **CRC8, poly 0x1D, init 0x00, final XOR = per-ID key K**;
alive counter = byte 1 low nibble, 0…14.

---

## Periods (median, after timeline fix)

| ID | Bus(es) | Period | DLC | Role |
|---|---|---|---|---|
| `0x1AF` | PT-CAN2 | 20 ms | 8 | EGS TX — turbine / output speed + gear |
| `0x3FD` | both | 100 ms | 5 | EGS TX — gear display |
| `0x39A` | both | 1 s | 6 | EGS TX — gear code |
| `0x518` | both | ~642 ms | 8 | EGS TX — network management (node `0x18`) |
| `0x598` | both | 1 s + burst | 8 | EGS TX — service channel + EWS challenge |
| `0x0A0` / `0x0A6` / `0x0A7` / `0x0B0` | PT-CAN2 | 10 ms | 8/8/7/8 | EGS-candidate high-rate status |
| `0x0A5` | both | 10 / 20 ms | 8 | RX — engine torque / speed (DDE) |
| `0x0D9` | both | 10 / 20 ms | 8 | RX — pedal (DDE) |
| `0x12F` | both | 100 ms | 8 | RX — terminal / ignition |
| `0x1A1` | both | 20 ms | 5 | RX — vehicle speed |
| `0x197` | both | 30 ms | 4 | RX — GWS lever |
| `0x3F9` | both | 1 s | 8 | RX — DDE drivetrain-2 (gear echo) |
| `0x397` | both | 60 ms | 3 | RX — DDE OBD diagnosis |

---

## (a) Must transmit

- **`0x1AF` (PT-CAN2, 20 ms, E2E K=`0x43`)** — the core EGS status frame. **confirmed**
  - byte 0 = CRC8, byte 1 low-nibble = counter
  - byte 2 = gear / range status (`0xFF`=P, `0x08`=R, `0x09`=D)
  - **turbine / input speed = LE16(byte3:byte4) − 2000 rpm** (r = 1.000 vs DDE FlexRay)
  - **output-shaft speed = LE16(byte5:byte6) − 2000** (~24 raw / km/h; r = 0.998 vs `0x1A1`)
  - byte 7 = engaged range (`0x01`=P, `0x06`=R, `0x09`=D)
- **`0x3FD` (both, 100 ms, E2E K=`0x70`)** — gear display to KOMBI / GWS.
  byte 2 = `0x20` P / `0x40` R / `0x60` N / `0x80` D. **confirmed**
- **`0x39A` (both, 1 s)** — gear code, byte 3 = 3 P / 2 R / 5 D. **confirmed**
- **`0x598` (both)** — 1 Hz `13 ff 01 18 …` keep-alive while KL15 on; at KL15-on also the
  EWS4 challenge burst on PT-CAN2 (see the immo note). **confirmed**
- **`0x518` (both, ~642 ms)** — BMW network-management, node byte 7 = `0x18`. The EGS kept
  transmitting control frames after NM stopped, so this is a "be-a-good-citizen" frame, not
  gating. **confirmed**
- **EGS-candidate 10 ms group — `0x0A0` (K=`0xCC`), `0x0A6` (K=`0xC5`), `0x0A7` (K=`0x23`),
  `0x0B0` (K=`0x65`):** PT-CAN2 only, all gear-dependent. **analysis** (ownership unproven —
  needs an EGS-alone bench log). **`0x0A7` byte 2 is the ATF / oil-temperature candidate** —
  rises monotonically over the warm-up (r ≈ +0.84), roughly °C = raw − 40. **hypothesis**
- **Not yet located** (logs are P/R/D creep only): actual-vs-target gear, shift-in-progress /
  clutch-lockup, and the **torque-reduction request EGS → DDE** during shifts. Flagged
  hypothesis in the CSV; a gears-1–4 + kickdown drive closes them.

---

## (b) Must receive

- **`0x0A5` (DDE, E2E K=`0x6A`)** — engine torque + speed.
  **engine speed = LE16(byte5:byte6) × 0.25 rpm** (r = +0.979 vs turbine). **confirmed**
  Torque in bits 16…27 (relative, ×0.5 − 1023.5 %). **analysis**
- **`0x0D9` (DDE, E2E K=`0xD7`)** — accelerator pedal, bits 16…27 / 40 → 0–100 %. **analysis**
- **`0x12F` (BDC / CAS / FEM, 100 ms, E2E K=`0xB1`)** — terminal / ignition.
  byte 1 `0x86` (KL15 off) → `0x8A` (on); the EGS EWS burst follows this edge by ~0.16 s.
  **Required** — this is the wake / ignition gate. **confirmed**
- **`0x1A1` (DSC / ICM, 20 ms, E2E K=`0x0F`)** — vehicle speed (r = 0.998 vs `0x1AF` output).
- **`0x197` (GWS, 30 ms, E2E K=`0x53`)** — gear-selector lever. Lever moves ~100 ms before
  the EGS changes gear. **Required** (or the controller supplies its own lever source).
- **`0x3F9` (DDE, 1 s)** — drivetrain-2, byte 6 gear code echoed.
- **`0x397` (DDE, 60 ms)** — OBD engine-diagnosis frame named as an EGS input by DTC CF1511.
- **`0x08F` / `0x0DC` (DDE)** — further DDE frames the EGS likely consumes; content not decoded.
- **Brakes / chassis:** `0x0EF` (brake-torque sum), `0x173` (brake status), `0x254` (wheel
  speeds) — shift quality / hill logic.
- **Optional comfort:** `0x328` time, `0x2CA` ambient temp, `0x3A0` vehicle condition.

**Minimum practical emulation set** to run the 8HP standalone (hypothesis to test on a bench):
`0x0A5`, `0x0D9`, `0x08F`, `0x3F9`, `0x397` (+ `0x0DC`) from the DDE with valid E2E;
`0x12F` terminal; `0x1A1` (+ `0x254`) speed; `0x0EF` / `0x173` brake; `0x197` GWS (or real GWS);
and the EWS4 answer to the EGS `0x598` challenge.

---

## (c) Can ignore

- **`0x518` NM** — not required to move gears (EGS kept running after NM stopped).
- **`0x335`** fuel-pump status (EKPS, not EGS).
- **`0x1B3`** FlexRay gateway copy (not EGS).
- **`0x618` / `0x6F0`** UDS diagnostics (node `0x18`) — not needed to run.
- **`0x199` / `0x19A` / `0x19F` / `0x302`** ICM accel / yaw / steer — not needed for basic shifting.
- **`0x297`** seat-belt / occupancy (ACSM).

---

## Vs the prior F11 EGS CAN map

Same car / logs as [egs_8hp_map.md](egs_8hp_map.md) / [egs_8hp_ids.csv](egs_8hp_ids.csv), so the
core TX / RX catalogue matches. This pass re-verified periods on a rebuilt timeline and
independently re-confirmed the `0x1AF` speed / gear and `0x0A5` rpm decodes.

**New here:**
- Independent engine-speed confirmation in **`0x0A5` bytes 5:6 LE × 0.25** (r = 0.979).
- An **ATF-temperature candidate on `0x0A7` byte 2** (r ≈ 0.84 over warm-up) — not pinned before.
- A few extra PT-CAN2-only periodic IDs (`0x137`, `0x145`, `0x1BA`) as EGS-candidates; undecoded.

This is a standard 8HP45 (N47 diesel). No 8HP45-specific CAN control-frame differences vs the
prior F11 map were observed.

---

## Open loops

- Torque-reduction request (EGS → DDE), actual / target gear, shift-in-progress & clutch-lockup —
  need a multi-gear drive log.
- ATF-temperature encoding (`0x0A7` byte 2): confirm factor / offset against a UDS temperature read.
- EGS-candidate 10 ms group ownership (`0x0A0` / `0x0A6` / `0x0A7` / `0x0B0` / …): confirm with an
  EGS-alone bench log on both buses.
