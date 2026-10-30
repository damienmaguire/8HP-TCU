# EGS/DDE EWS4 Immobiliser — Firmware Findings Summary

**Project:** Damien Maguire open-source 8HP-TCU EV conversion.
**Date:** 2026-10-30 (times Europe/Dublin).
**Scope:** document the EWS4 immobiliser challenge–response so a custom open-source TCU
can satisfy the handshake on **owned hardware**. Documentation/analysis only — no unlock
tool, no recovered secret key, and no vehicle-identifying data in this file.
**Confidence words:** *confirmed / evidence / analysis / hypothesis / candidate*.

This summary draws together today's reverse-engineering of two owned modules and the
on-CAN captures. The detailed working notes, disassembly artefacts and per-claim evidence
live outside the repo; this file is the readable overview.

---

## What's new

Previously the handshake was characterised only from CAN captures: the transport was
understood, but the function turning a challenge into a response was unknown. With firmware
dumps of **both** ends now in hand, the algorithm itself is identified and cross-confirmed:

> **`response = first6( MD5( nonce[16] ‖ K[16] ) )`**

i.e. the 16-byte challenge nonce concatenated with a 16-byte key `K`, MD5-hashed, and the
**first 6 bytes** of the digest returned as the authenticator. This was pinned independently
in the ZF 8HP EGS firmware (Renesas SH-2A) and in the F11 DDE firmware (Infineon TriCore) —
two different CPUs, two different codebases, the same construction. That agreement is strong
evidence the characterisation is correct.

---

## Subjects

| Module | Part | MCU | Notes |
|---|---|---|---|
| EGS (transmission) | Bosch EGS, ZF 8HP | Renesas **SH725xx** (SuperH SH-2A, big-endian) | FLASH + A2L analysed; EEPROM/data-flash was **not** in the dump. |
| DDE (engine) | Bosch **EDC17C41** | Infineon **TC179x** (TriCore, little-endian) | 4 MB flash **and** 128 KB EEPROM both present. |

**Correction to the earlier CAN notes:** the DDE is **EDC17C41**, not EDC17C50
(flash strings `...EDC17C41-4.57`, `32/1/EDC17_C41/11/P_735`, MCU tag `.../1797`,
ECU designation `DME__DDE721b`). Worth bearing in mind for which public tool/ISN docs apply.

---

## The handshake on the wire (confirmed)

Extended-address ISO-TP on the `0x580 + node` service IDs — **not** stock UDS SecurityAccess
(no flow-control frames):

- Node `0x12` = DDE (`0x592`), `0x18` = EGS (`0x598`), `0x40` = CAS/FEM (`0x5C0`).
- Byte `d0` = a fixed address-extension per leg; `d1` = ISO-TP PCI.
- **Challenge:** opcode `0x10` (or `0x11` on a re-arm) + **16-byte nonce**.
- **Response:** opcode `0x21` + **6 bytes** (a 48-bit authenticator).

Two legs per KL15-on, ~140–160 ms apart:
- **L1** DDE→CAS (challenge `0x592` ext `0x80`, response `0x5C0` ext `0x82`).
- **L2** EGS→DDE (challenge `0x598` ext `0x82`, response `0x592` ext `0x85`).

Challenge→response latency is ~20 ms. Across the captured cycles every nonce and response is
unique (no replays), consistent with a fresh random nonce each cycle. `0x618`/`0x6F0` carry
ordinary UDS ReadDataByIdentifier, not handshake traffic.

---

## The algorithm (identified)

The A2L labels the EGS immobiliser subsystem explicitly as EWS4 using *"DH-exp + MD5"*, and
the disassembly bears that out. Two authenticator forms share one construction:

- **Static-key leg:** `K` = a stored 16-byte secret key →
  `response = first6( MD5( nonce ‖ staticKey ) )`.
- **Session-key leg:** a Diffie-Hellman style modular exponentiation runs **first**; its
  ~1024-bit result is MD5-hashed to derive a session key, and that session key is then used
  as `K`: `sessionKey = MD5( g^x mod p )`, then `response = first6( MD5( nonce ‖ sessionKey ) )`.

Details that matter for a re-implementation:
- MD5 input is a single contiguous 32-byte buffer: nonce first, key second, natural byte order.
- The 48-bit response is simply `digest[0..5]` — the first 6 bytes, no permutation. Because
  the MD5 finaliser serialises RFC-1321 little-endian words, that is word A (4 bytes, LSB
  first) followed by the low 2 bytes of word B.
- On the EGS the primitive is the **same crypto core used for RSA-1024/MD5 flash-signature
  checking**, reached at runtime through an indirect dispatch (an ops table beside the MD5
  constant tables), not a separate copy. The DDE uses its own MD5 the same way.
- MD5 itself is confirmed byte-for-byte in both images (init words, the 64-entry sine table
  and the shift table all present).

The RSA-1024 signing that protects **flashing** the modules is a separate mechanism from this
runtime immobiliser handshake; they are not the same path.

---

## What a working TCU still needs (bench side)

The *algorithm* is documented. To actually produce valid responses on owned hardware, two
pieces of per-vehicle data are required, and both live in the module EEPROM / data-flash:

1. **The 16-byte secret key `K`.** The firmware reads it from EEPROM into a RAM key bank at
   runtime. In the DDE EEPROM a high-entropy block is the strongest candidate source, but the
   exact EEPROM→RAM mapping is not a linear copy — it goes through the EDC17 EEPROM-manager
   (logical blocks with CRCs), so the precise offset binding is not yet pinned.
2. **The Diffie-Hellman parameters** (modulus / generator / private exponent) for the
   session-key leg. The DH leg is clearly present in both images; the parameters were not
   isolated as clean flash constants and most likely also reside in EEPROM.

These are the natural next steps, and they belong on the owner's own bench with the owner's
own modules — which is also where end-to-end validation (replaying a captured nonce and
checking the computed response) should happen.

---

## Evidence vs hypothesis

| Claim | Status |
|---|---|
| ISO-TP transport, service IDs, leg structure, timing | **Confirmed** (CAN captures) |
| Challenge = `0x10/0x11`+16, response = `0x21`+6 | **Confirmed** |
| `response = first6(MD5(nonce ‖ K))`; MD5 primitive present | **Evidence** (both firmwares) |
| Session-key leg = modexp-then-MD5 (DH-exp + MD5) | **Evidence** |
| Same construction on EGS and DDE | **Evidence** (independent) |
| Exact EEPROM offset of `K`; DH parameter values | **Hypothesis / not yet pinned** (EEPROM-manager indirection; DH params likely EEPROM-resident) |

---

## Scope and ethics

This work covers the owner's own vehicle and modules for a legitimate EV conversion. It
documents the handshake and where the key material sits; it deliberately does **not** recover
or publish any secret key, reproduce live responses, or provide a means of unlocking modules
that are not the owner's. No vehicle identifier or key material is included in this file.
