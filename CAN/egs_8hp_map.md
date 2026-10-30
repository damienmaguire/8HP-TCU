# 8HP EGS on CAN: map from the ptcan1 logs (2026-10-30, Europe/Dublin)

**Sources:** captures/ptcan1 (ignonoff, 3krev, r10md10m drive). Bus 2 = PT-CAN2, bus 3 = PT-CAN, bus 0/1 = FlexRay (vehicle/DDE). Contrast: bench_f10 (F10 DDE alone; this was the **PT-CAN side**: it has 0x0DC, which the car only shows on bus 3).

**Drive windows:** P 9–16 s, R 17.5–32.5 s, D 34–42.4 s. Times are in seconds from file start.

**Labels:** CONFIRMED = shown in our data. LIKELY = strong data, owner not proven. HINT = public sources only. The ID table is in egs_8hp_ids.csv.

## 1. Topology
- The EGS is on both PT-CAN and PT-CAN2. Damien's wiring shows this; we take it as given.
  - Data agrees: the EGS NM frame 0x518 (node 0x18) appears on both buses.
  - 0x598 (service channel 0x580+0x18) also appears on both buses.
- Diagnostics run on PT-CAN only. 0x618 responses and the tester's 0x6F0 flow control for 0x18 are seen only on bus 3. CONFIRMED.
- The GWS also transmits on each bus on its own. Its 0x55E NM payload differs per bus (d4..5 = `11 47` on bus 2, `19 61` on bus 3), while 0x197 is identical on both. CONFIRMED.
- No EGS frame appears on FlexRay as a gateway copy:
  - The CAN→FR content match found no EGS ID.
  - The P/R/D byte scan of every vehicle-FR PDU found no field with the EGS gear codes.
  - EGS data reaches FlexRay only inside the **DDE's own PDUs**:
    - FR 0x26: turbine speed = 0x1AF d3..4.
    - FR 0xE6 b7 and 0x28 c3 b11: gear code = 0x39A d3 / 0x3F9 d6.
  - CONFIRMED.
  - Consequence: without the DDE, nothing on FlexRay tells the car the gear. Only the EGS CAN frames (and 0x3FD to the GWS/KOMBI) do.

## 2. EGS transmit (per bus)

| ID | PT-CAN2 (b2) | PT-CAN (b3) | Rate / len | E2E (CRC8 0x1D, init 0, xor K) | Validated content | Status |
|---|---|---|---|---|---|---|
| 0x1AF | yes | **no** | 20 ms / 8 | K 0x43, b1..b7, ctr d1 lo 0..14 | d3..4−2000 turbine rpm (r 1.000 vs FR 0x26). d5..6−2000 output speed, ~24 raw per km/h (r 0.998). d7 01/06/0B and d2 FF/08/0A for P/R/D | CONFIRMED EGS |
| 0x3FD | yes | yes (identical, same ms) | 100 ms / 5 | K 0x70, b1..b4, ctr 0..14 | d2 display: 0x20 P, 0x40 R, 0x60 N, 0x80 D | CONFIRMED (and a DTC hint) |
| 0x39A | yes | yes (identical) | 1 s / 6 | none | d3 gear code 3/2/5 = P/R/D. d4 e2/e0/e0 | LIKELY EGS |
| 0x518 | yes | yes | 642 ms / 8 | none | NM, d7 = 0x18 | CONFIRMED |
| 0x598 | yes | yes | 1 s while KL15 on / 8 | none | `13 ff 01 18 ff…`, plus the EWS burst (§5, PT-CAN2 only) | CONFIRMED sender |
| 0x618 | – | yes | on request | – | UDS | CONFIRMED |
| 0x0A0 / 0A6 / 0A7 / 0B0 | yes | no | 10 ms / 8,8,7,8 | K CC / C5 / 23 (b1..b6) / 65, ctr | Gear-dependent bytes (see the csv). 0x0B0 b3..4 vs turbine r −0.91 | Owner unproven: EGS or another PT-CAN2 node |
| 0x0F8, 0x2B1, 0x145 | yes | no | 20 / 100 / 10 ms | 0x145 K 0x0C | 0x0F8 = rpm×1.6 | Owner unproven |

**Not EGS:**
- 0x335 is the fuel-pump status from the EKPS (DTC hint; bus 2 only, like EKPS NM 0x517).
- 0x1B3 is a FlexRay→PT-CAN copy of vehicle FR 0x61 (100 % content match, +9 ms).

**Missing EGS outputs.** Shifts and torque intervention were not isolated. The drive only covered P/R/D at creep (no gear 2+, no kickdown), so these were never exercised:
- actual vs target gear in D;
- the torque-reduction request to the DDE;
- oil temperature.

**Timing note.** 0x1AF and 0x3FD run from file start (KL R, before KL15) until log end. The EGS kept transmitting after its NM 0x518 stopped at 79.5 s. So the NM stop is not EGS sleep.

## 3. What the EGS can consume (inputs present on its buses)
Passive logs cannot prove what the EGS consumes. They only show what is present and who sends it. Public EGS DTC texts are the only "consumed" evidence, and they are HINTs.

| Source | IDs (bus, rate, E2E) | Consumed? |
|---|---|---|
| DDE | 0x0A5 rpm/torque (b2 100 Hz, b3 50 Hz, K 6A). 0x0D9 pedal (b2 100 Hz, b3 50 Hz, K D7). 0x08F (K 3C). 0x0DC (b3, K 0F). 0x3F9 (1 s, both). 0x397 (60 ms, both) | HINT: EGS DTC CF1511 names 0x397 from DME/DDE; CF1601 names 0x0A5 (shown for a DCT receiver). The F10 bench confirms the DDE sends all six |
| DSC / ICM (FlexRay→PT-CAN copies) | 0x1A1 speed (both, 20 ms, K 0F). 0x254 wheels, 0x0EF brake torque (K CA), 0x173 brake (K 5E), 0x199/19A/19F/302 accel/yaw/steer (PT-CAN only) | HINT: EGS DTCs CF16C1/CF16D1 (DSC) and CF18D1/CF18E1 (ICM) name group messages, not IDs |
| GWS | 0x197 lever (both, 30 ms, K 53, b1..b3) | HINT CF17B1, plus data: the lever moves about 100 ms before the gear changes |
| BDC/CAS/FEM | 0x12F terminal (both, 100 ms, K B1). 0x3A0 (both, 5 s) | Data: the EGS EWS burst follows the 0x12F KL15 edge by 0.16 s |
| ACSM | 0x297 seat belt/occupancy (PT-CAN, 200 ms) | HINT: EGS DTC CF2301 |
| KOMBI | 0x328 time, 0x2CA ambient (both). 0x393, 0x5E0 (PT-CAN) | not shown |

Practical reading: to run the 8HP standalone, the minimum plausible set to emulate is:
- DDE: 0x0A5, 0x0D9, 0x08F, 0x3F9, 0x397 (+0x0DC/0x145?), with valid E2E (K as above).
- 0x12F terminal.
- 0x1A1 and 0x254 speeds.
- 0x0EF / 0x173 brake.
- 0x197 GWS (or a real GWS).
- The EWS answer (§5).

This list is a hypothesis to test, not proven.

## 4. E2E (all CRC8 poly 0x1D, init 0, final xor K; counter = d1 low nibble, 0..14)
EGS: 0x1AF K 0x43 (b1..b7), 0x3FD K 0x70 (b1..b4).
Inputs: 0x0A5 6A, 0x0D9 D7, 0x08F 3C, 0x0DC 0F, 0x1A1 0F, 0x0EF CA, 0x173 5E, 0x12F B1, 0x197 53.
Unproven PT-CAN2 IDs: 0x0A0 CC, 0x0A6 C5, 0x0A7 23 (b1..b6), 0x0B0 65, 0x145 0C.

## 5. EWS / immobiliser: evidence vs hearsay

### Evidence (all 3 logs, CONFIRMED)
At ignition-on we see two paired exchanges on the BMW service channel (ID = 0x580 + sender address). Each uses ISO-TP-style segmentation: byte 0 is an address-like prefix, then FF `10 len` and CFs `21`/`22`.

1. **DDE ↔ CAS/FEM on PT-CAN (bus 3)**
   - 0x592 (DDE, addr 0x12; the F10 DDE bench also sends 0x592) sends a 17-byte message: `80 10 11 | 10 + 16 random bytes`.
   - About 20 ms later, 0x5C0 (addr 0x40 = CAS/FEM by the address pattern) sends a 7-byte message: `82 10 07 | 21 + 6 random bytes`.
   - Then follow `40 22 00 15` (0x592), `48 00 f5 25 05 00 f0 25` (0x5C0) and `40 xx 00 29`.
2. **EGS ↔ DDE on PT-CAN2 (bus 2) only**, about 120–160 ms later:
   - 0x598 (EGS) sends a 17-byte message: `82 10 11 | 10 + 16 random bytes`.
   - 0–18 ms after its last frame, 0x592 (DDE) on **bus 2** sends a 7-byte message: `85 10 07 | 21 + 6 random bytes`.
   - This is the only time 0x592 appears on PT-CAN2.

| Log | KL15 edge (0x12F d2 86→8A/29) | DDE→CAS 17 B | CAS 7 B | EGS 17 B (b2) | DDE 7 B (b2) |
|---|---|---|---|---|---|
| ignonoff | 4.295 | 4.335 | 4.357 | 4.496 | 4.534 |
| 3krev | – | 4.612 | 4.633 | 4.754 | 4.774 |
| drive | – | 7.381 | 7.403 | 7.521 | 7.541 |

- All random bytes differ in every file, consistent with a per-wake challenge/response.
- No frame of either exchange appears on FlexRay; a byte-substring search of bus 0/1 found none. No gateway copy appears on PT-CAN either.
- ignonoff 34.44 s: the DDE↔CAS pair repeats with KL15 off (0x12F d2 = 0x86). The EGS pair does **not** repeat. So the EGS exchange is tied to KL15-on; the DDE↔CAS one is not only tied to it.
- From the data alone we can't tell which side is the challenger. Byte counts suggest it (16 B out, 6 B back), but that is inference.

### Hearsay / public hints (not verified)
- On F/G-series cars, the EGS holds an ISN shared with the DME/DDE and CAS/FEM/BDC. A mismatched used EGS lets the engine start but won't leave P or engage gears. A virgin EGS learns the ISN at first ign-on. Sources: microtronik/AutoHex, ARMHEX, BMW-Explorer, automodulelab.
- Our data fits "the EGS authenticates against the DDE on PT-CAN2", but the payload meaning, key and algorithm are unknown.

### Implication for using the 8HP elsewhere (inference)
- Expect the EGS to need a correct answer to its 0x598 burst on PT-CAN2. Without it, it may refuse to engage gears.
- Option: carry the donor DDE along with a virgin/ISN-reset EGS. Replaying a captured answer is unlikely to work because the challenge is random.
- Test before relying on any of this.

## 6. Public hints used (pointers only)
- openinverter GWS wiki: 0x3FD/0x197 formats and K values. These match our data.
- projectgus GWS DTCs: 0x3FD from the EGS on PT-CAN and PT-CAN2; GWS inputs 0x12F/0x202/0x328/0x3A0.
- Public EGS DTC list (F10 535i, Reddit r/BmwTech): CF1511 0x397 (DDE), CF17B1 0x197 (GWS), CF2301 0x297 (ACSM), DSC/ICM group faults.
- Other DTC lists: 0x335 = fuel pump status from EKP; 0x1A1/0x199/0x19A/0x19F/0x302 from ICM; 0x0EF from DSC.
- motor-talk: GWS redundant on PT-CAN/PT-CAN2.
- Apex IOcan: separate PT-CAN / PT-CAN2 8HP presets.

## 7. Tests that would close the gaps

| # | Test | What it answers |
|---|---|---|
| 1 | EGS-alone bench (12 V, KL15 wire, both buses logged) | The true EGS TX list per bus, including whether 0x0A0/0A6/0A7/0B0/0F8/2B1/145 are EGS. Shows what it sends with no DDE answer to 0x598 |
| 2 | DDE-alone bench with **both** buses | The DDE's PT-CAN2 TX set (the existing bench was PT-CAN side only) |
| 3 | Car, DDE PT-CAN2 pair disconnected | Does the EGS still engage gears without the 0x592 answer? Which DTCs are set? |
| 4 | GWS unplugged | Which EGS messages and states depend on 0x197 |
| 5 | Drive above creep through gears 1–4+, manual/S mode up/down shifts, one kickdown | Actual vs target gear, shift-in-progress, torque intervention fields (EGS→DDE and DDE→EGS) |
| 6 | 10+ minutes warm-up log | Oil temperature field |
| 7 | Read EGS DTCs/freeze frames via 0x6F1→0x18 on PT-CAN after tests 3/4 | The EGS's own named list of missing messages (= its true input set) |
| 8 | Two ign-on cycles back-to-back, plus one with the key absent | Whether the EGS challenge repeats and what the EGS does on failure |
