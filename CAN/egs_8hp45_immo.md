# F11 8HP45 EGS — EWS4 Immobiliser Confirmation

**Car:** owner's BMW F11 520d (2011), ZF 8HP45 / ITCU2 mechatronic — EV-conversion donor.
**Scope:** documentation only. No key recovery, no response computation, no unlock tooling.
No VIN or secret-key bytes are included in this file.
**Companion:** [CANTCU_CAN_MAP.md](CANTCU_CAN_MAP.md) (standalone-controller CAN map).
**Confidence:** **confirmed** (observed here) / **analysis** (derived) / **hypothesis** (inference).

---

## Verdict

| Question | Answer | Level |
|---|---|---|
| Does this 8HP45 use the family EWS4 challenge/response? | **Yes** | confirmed on-wire; analysis on construction |
| Transport | Extended-address ISO-TP on `0x580 + node`, **no flow-control** | confirmed |
| Challenge / response shape | `0x10`/`0x11` + 16-byte nonce → `0x21` + 6 bytes | confirmed |
| Construction | `response = first6( MD5( nonce[16] ‖ K[16] ) )` | analysis (family cross-check — see below) |
| Where the secret lives | Module EEPROM (EWS4 state + key bank) | confirmed region exists |
| What a standalone controller must do | **Answer the EGS challenge** (play the DDE responder role on PT-CAN2) | confirmed roles |

---

## What's in this dump

- **`maps.bin` (2 MB)** — a **calibration / DST data partition**, not a CPU program image.
  The low 1.5 MB and most of the high end are erased (0xFF); only ~384 KB holds data.
  There are **no reset vectors and no executable code**, so the MCU architecture can't be
  identified from this file alone. Identity strings confirm ZF **8HPXY / ITCU2_3_X_2048**,
  build `Z0BA5800_J580A`, SWE `#EGS__GKEB23…F10-N47D20O1-EUR` — N47 2.0d, F10/F11 platform.
- **`int_eeprom.bin` (128 KB)** — the serial/data EEPROM, present here (it was absent from the
  earlier bench 530e flash set). Four 32 KB banks with A/B redundancy. This is where the
  EWS4 immobiliser state and the shared key live.
- **`.mmf`** — the tool's encrypted full-backup container. Opaque; the two bins above are its
  decrypted products.

The flash-level crypto core (MD5 + Diffie-Hellman modexp) lives in the **program flash**, which
is not in this file set. That construction was previously pinned on a sibling Bosch EGS
(GA8P75HZ / Renesas SH-2A) — see [egs_dde_immo_firmware_findings.md](egs_dde_immo_firmware_findings.md).

---

## On-wire handshake (this car)

Two legs per KL15-on, from the F11 CAN captures (SavvyCAN timestamps wrap every 1 s — periods
below use a rebuilt continuous timeline):

| Leg | Challenger → Responder | Bus | Challenge | Response | Latency |
|---|---|---|---|---|---|
| L1 | DDE `0x592` → CAS/FEM `0x5C0` | PT-CAN | `0x10`/`0x11` + 16-byte nonce | `0x21` + 6 bytes | ~20–22 ms |
| L2 | EGS `0x598` → DDE `0x592` | **PT-CAN2** | `0x10` + 16-byte nonce | `0x21` + 6 bytes | ~0–20 ms |

- Extended-address ISO-TP: a fixed address-extension byte per leg, then the ISO-TP PCI —
  **no flow-control frames**.
- L2 starts ~140–160 ms after L1. A KL15-off re-arm repeats **only L1** (opcode `0x11`);
  the EGS leg is tied to KL15-on.
- Every nonce and every response across the captured cycles is unique — **no replay**.
  A captured answer will not satisfy a fresh nonce.

Raw frame catalogue: [immo_handshake_frames.csv](immo_handshake_frames.csv),
[egs_dde_immo_handshake.md](egs_dde_immo_handshake.md).

---

## Construction (family evidence)

Pinned independently on the sibling EGS flash (and cross-confirmed on the F11 DDE):

```
response = first6( MD5( nonce[16] ‖ K[16] ) )
```

- Static-key leg: `K` = EEPROM-resident 16-byte secret.
- Session-key leg: Diffie-Hellman modexp first, then `K = MD5( g^x mod p )`, then the same
  `first6(MD5(nonce‖K))` for the on-wire value.
- Truncation is simply digest bytes `[0..5]` (RFC-1321 little-endian finaliser).

That matches everything seen on this car (opcodes, lengths, timing, fresh nonce). Because
`maps.bin` has no code, the MD5/modexp core was **not re-derived from this 8HP45 image** —
it is carried as family evidence. The matching shape plus the same EWS4 EEPROM state block
make the match well-supported for this unit, but analysis-level rather than
confirmed-from-this-flash.

A2L labels on the family confirm the subsystem: `EepromEws4Immediate.f_ews4_locked` /
`fsc_status`, and `st_crypto_fctn_call` described as **"DH-exp + MD5"**.

---

## What a standalone controller must do

- The EGS is a **challenger** on PT-CAN2 (leg L2). At KL15-on it sends `0x598 … 0x10 + nonce`
  and expects a correct `0x592 … 0x21 + 6` answer from the "DDE" peer within ~20 ms.
- A standalone build that keeps the donor EGS must therefore **replicate the DDE responder
  role on PT-CAN2** — receive the nonce and return `first6(MD5(nonce‖K))`. That needs the
  shared key `K`, which lives in the paired modules' EEPROMs and is **owner/bench-side only**.
- Leg L1 (DDE→CAS/FEM) is a separate engine/body authentication the gearbox does not sit on.
  A gearbox-only build does not need to satisfy L1 to move gears.
- If the matched donor DDE is kept alongside the EGS, the controller need not touch the EWS4
  leg at all — it just drives the control/status frames in the CAN map.

**Deliberately not done here:** no extraction of `K`, the DH parameters or the ISN; no
`MD5(nonce‖K)` computation; no unlock method. Those stay on the owner's bench with the
owner's own modules.

---

## Vs the earlier bench dump

| | This 8HP45 (F11 donor) | Bench 530e 8HP set |
|---|---|---|
| EEPROM | present (128 KB, 4-bank) | absent |
| Program flash / code | not in this dump (`maps.bin` = cal/DST only) | present (full analysis) |
| Gearbox | standard 8HP45, N47 diesel, ITCU2 | hybrid GA8P75HZ |
| On-wire EWS4 | identical shape | same family |

---

## Open loops

- Binding physical EEPROM offsets to A2L labels for *this* unit needs this unit's own A2L
  (or its program flash).
- Code-level confirmation of `first6(MD5(nonce‖K))` for *this* 8HP45 needs its program flash.
- Virgin-relearn / which peer "owns" `K` for a standalone responder is untested (bench/car).
