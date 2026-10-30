# F11 DDE ↔ CAS/FEM ↔ EGS immobiliser (ISN/EWS) handshake

**Author / date:** Fred, 2026-10-30 (Europe/Dublin).

**Scope:** Damien Maguire's BMW F11 520d (DDE EDC17C50 + ZF 8HP). Both engine and gearbox are being removed for the EV conversion; this note characterises the per-ignition-on immobiliser exchange **from his own captures only**. No live system was attacked.

**Data used:**
- `captures/ptcan1/F11_4log_ignonoff.csv` — KL15 on (no crank), then off; **two** DDE↔CAS exchanges (ign-on + a second after ~30 s while still awake).
- `captures/ptcan1/F11_4log_3krev.csv` — one full ign-on + crank.
- `captures/ptcan1/F11_4log_r10md10m.csv` — one full ign-on + drive.
- `captures/bench_f10/f10_dde_bench_ptcan.csv` — F10 DDE alone on a bench bus: only a periodic status frame on 0x592, **no** challenge/response (no CAS, no EGS).
- Frame-level extraction: `analysis/immo_handshake_frames.csv`.

**Labels used throughout:**
- **EVIDENCE** — measured in the logs above.
- **HYPOTHESIS** — consistent with the evidence and with public BMW EWS material, but not proven by us.
- **PUBLIC** — from third-party documentation; cited at the end. Not verified by us.

---

## 1. Topology and frame IDs (EVIDENCE)

BMW service / diagnostic channels use `0x580 + node_address`:

| CAN ID | Node | Role in our logs |
|---|---|---|
| 0x592 | 0x12 = DDE | DDE service TX. Carries the DDE→CAS challenge (PT-CAN) and the DDE→EGS response (PT-CAN2). |
| 0x5C0 | 0x40 | Body / access module (CAS family; on F11 this is FEM/BDC — HYPOTHESIS from PUBLIC F-series docs). Carries the CAS→DDE response. PT-CAN only. |
| 0x598 | 0x18 = EGS | EGS service TX. Carries the EGS→DDE challenge on **PT-CAN2 only**. Also carries a 1 Hz status `13 FF 01 18` on **both** buses while KL15 is on. |

Other 0x592 / 0x5C0 / 0x598 traffic exists (status, NM-adjacent), but it is **not** part of the seed/key exchange. See §7.

At every KL15-on we see **two** challenge–response legs, always in this order:

```
t≈0   (relative to the first challenge FF)
  PT-CAN  : DDE 0x592  --17 B challenge-->  CAS/FEM 0x5C0
  ~20–22 ms later
  PT-CAN  : CAS/FEM 0x5C0 --7 B response-->  DDE 0x592

  ~120–160 ms after the first challenge
  PT-CAN2 : EGS 0x598  --17 B challenge-->  DDE 0x592
  ~0–18 ms later
  PT-CAN2 : DDE 0x592  --7 B response-->  EGS 0x598
```

Measured latencies (challenge FF → response FF), Europe/Dublin wall clock relative to file start:

| Log | DDE→CAS challenge | CAS→DDE response | Δ ms | EGS→DDE challenge | DDE→EGS response | Δ ms | Gap CAS→EGS ms |
|---|---|---|---|---|---|---|---|
| ignonoff (1st) | 4.335 | 4.357 | 22 | 4.496 | 4.534 | 38 | 139 |
| ignonoff (2nd) | 34.441 | 34.462 | 21 | — | — | — | — (no EGS leg) |
| 3krev | 4.612 | 4.633 | 21 | 4.754 | 4.774 | 20 | 121 |
| drive | 7.381 | 7.403 | 22 | 7.521 | 7.541 | 20 | 118 |

The second ignonoff exchange happens ~30 s after the first, while the car is still awake (no crank; ignition was never taken to "start"). Only the DDE↔CAS leg fires; the EGS does not re-challenge. EVIDENCE.

Terminal state (0x12F) at the first challenge: d2 has just moved through 0x29 / 0x3A (KL15 rising). Crank (rpm > 300) starts ~0.8–1.0 s **after** both legs complete. EVIDENCE.

---

## 2. Transport: extended-address ISO-TP (EVIDENCE)

This is **not** a plain 8-byte CAN PDU. Every challenge and response is an ISO-TP multi-frame message with an **address-extension** byte in d0:

```
d0 = address extension (routing / peer identifier)
d1 = PCI
     0x1L = First Frame; length = ((L & 0xF) << 8) | d2
     0x2N = Consecutive Frame, sequence N
d2… = payload (FF: length then data; CF: data)
```

| Leg | Sender ID | d0 (addr ext) | FF PCI | Declared length | # frames | Response peer |
|---|---|---|---|---|---|---|
| DDE → CAS | 0x592 | **0x80** | 0x10 | 17 | 3 (FF + CF1 + CF2) | CAS replies on 0x5C0 with d0=0x82 |
| CAS → DDE | 0x5C0 | **0x82** | 0x10 | 7 | 2 (FF + CF1) | — |
| EGS → DDE | 0x598 | **0x82** | 0x10 | 17 | 3 | DDE replies on 0x592 with d0=0x85 |
| DDE → EGS | 0x592 | **0x85** | 0x10 | 7 | 2 | — |

**No Flow Control frame (PCI 0x3x) is observed** between the FF and the CFs of these exchanges. The sender streams CF1 and CF2 within 1–20 ms of the FF. EVIDENCE. (Other traffic on 0x5A9 / 0x6F0 does use FC elsewhere in the logs; this particular channel does not.)

Address-extension values are fixed across all wakes. EVIDENCE.

### Reassembled application payload

After stripping ISO-TP, the message is:

| Direction | Length | Byte 0 (opcode) | Bytes 1… |
|---|---|---|---|
| Challenge (DDE→CAS or EGS→DDE) | 17 | **0x10** (first wake) or **0x11** (ignonoff 2nd DDE→CAS only) | **16 bytes**, all varying |
| Response (CAS→DDE or DDE→EGS) | 7 | **0x21** (constant) | **6 bytes**, all varying |

So the earlier shorthand "17-byte random / 7-byte random" was counting the application payload including the opcode. The cryptographic material is **16 B challenge + 6 B response**. EVIDENCE.

### Is this UDS / KWP?

**HYPOTHESIS (strong):** this is BMW's proprietary immobiliser channel on the service ID range, **not** UDS SecurityAccess (0x27).

Reasons against 0x27:
- The first application byte is 0x10 / 0x11 / 0x21, not 0x27.
- Lengths (17 / 7) and the 16-byte / 6-byte split do not match typical UDS seed/key framing (SID + level + seed).
- There is no diagnostic session control (0x10 UDS) preceding the exchange on these IDs.

Reasons it still looks like a seed/key protocol:
- Fixed opcode + long random challenge + short response.
- Matches the publicly documented EWS4 challenge–response shape (PUBLIC, §6).

We have **not** identified a SID/sub-function beyond the opcodes 0x10/0x11 (challenge) and 0x21 (response). EVIDENCE for the opcodes; HYPOTHESIS for "not UDS 0x27".

---

## 3. Challenge → response pairs (EVIDENCE)

### 3.1 PT-CAN: DDE challenger → CAS/FEM responder

| Log | Wake | Challenge opcode + 16 B | Response opcode + 6 B |
|---|---|---|---|
| ignonoff | 1 | `10` `60 53 62 4e a7 03 7d 6c 25 7e 56 62 87 17 34 9e` | `21` `76 df 83 0b 77 28` |
| ignonoff | 2 | `11` `1f 8c 91 2a ab ea 4b a8 c1 1b 83 1c ed a8 b2 51` | `21` `d6 75 d9 9d 2f ba` |
| 3krev | 1 | `10` `3a 4b 88 8d 02 61 d0 d2 67 87 cb 6a 48 6c cb 87` | `21` `e0 f7 8f 28 1f 4e` |
| drive | 1 | `10` `75 11 41 aa 61 fb ef e3 9d 21 64 c1 a9 c1 58 c4` | `21` `c7 d5 80 fc ba c3` |

### 3.2 PT-CAN2: EGS challenger → DDE responder

| Log | Wake | Challenge opcode + 16 B | Response opcode + 6 B |
|---|---|---|---|
| ignonoff | 1 | `10` `94 e4 11 3b 4e 7f ec 60 1d 4f 68 07 83 99 41 3c` | `21` `6e 9d 1d 92 50 7d` |
| 3krev | 1 | `10` `c9 87 9a cc 91 36 ae fb 26 8b 02 f0 29 ce 61 01` | `21` `9c 32 1d ef 59 bb` |
| drive | 1 | `10` `31 a3 83 4d f0 fd 73 fa f3 4b 09 f8 40 80 10 c0` | `21` `e4 04 1e ee f8 2b` |

**No challenge or response repeats** across these 7 exchanges (4 CAS-leg + 3 EGS-leg). EVIDENCE.

---

## 4. Structure analysis (EVIDENCE, with sample-size caveats)

With only 3–4 pairs per leg, entropy estimates are upper-bounded by log₂(n) (~1.6–2.0 bits). They can only tell us "looks unique so far", not "cryptographically random".

| Field | Distinct across wakes | Constant? | Notes |
|---|---|---|---|
| Challenge opcode | 0x10 (all first wakes); 0x11 (ignonoff 2nd only) | mostly | 0x11 may mean "retry / re-auth while awake" — HYPOTHESIS |
| Challenge bytes 1..16 | all distinct in every wake | **no** | No fixed prefix, no fixed suffix, no zero bytes observed |
| Response opcode | 0x21 every time | **yes** | |
| Response bytes 1..6 | all distinct | **no** | One coincidence: DDE→EGS byte 3 is `1d` in two of three pairs (ignonoff, 3krev). Almost certainly chance with n=3. |

Cross-checks that returned nothing (EVIDENCE of absence in this sample):
- No challenge equals any other challenge (either leg, any log).
- No response equals any other response.
- No challenge is a prefix/suffix of its response, or vice versa.
- No response is a truncation of its challenge (first/last 6 bytes, or any 6-byte window).
- XOR of challenge[i] with response[j] shows no constant across pairs.
- Challenge bytes 1..16 of the DDE→CAS leg are independent of the EGS→DDE challenge in the same wake (no shared bytes in position).

**Byte-order / block hints:** none visible. A 16-byte challenge is the natural AES block size, and a 6-byte response is a plausible truncated MAC — but that is **PUBLIC analogy / HYPOTHESIS**, not evidence from these bytes.

**Can the response be a pure function of the challenge?**
In principle yes (that is how challenge–response works: response = f(challenge, shared_secret)). With 3–4 pairs we **cannot** test candidate functions. Requirement for a meaningful attack on f alone: dozens to hundreds of independent pairs **plus** the shared secret (ISN/SK), or firmware that reveals f. See §8.

---

## 5. Roles: who challenges whom, who holds the secret (EVIDENCE + HYPOTHESIS)

| Leg | Challenger | Responder | Bus | Shared secret (HYPOTHESIS from PUBLIC) |
|---|---|---|---|---|
| 1 | **DDE** | CAS/FEM | PT-CAN | DDE ↔ CAS/FEM long ISN / SK (128-bit) |
| 2 | **EGS** | DDE | PT-CAN2 | EGS ↔ DME/DDE long ISN (same vehicle ISN family) |

**DDE role (EVIDENCE):**
- On PT-CAN the DDE is the **challenger** (sends 16 B, checks 6 B from CAS).
- On PT-CAN2 the DDE is the **responder** (receives 16 B from EGS, returns 6 B).

So the DDE sits in the middle of the immobiliser chain. That matches PUBLIC F-series practice: the body module and the DME/DDE share an ISN, and the 8HP EGS is also paired to that vehicle ISN; the DME/DDE is the module that bridges the two.

**Who holds the master secret?**
- PUBLIC: F-series long ISN (SK) is 16 bytes, stored in CAS/FEM/BDC, DME/DDE, and (for 8HP) EGS.
- HYPOTHESIS consistent with our roles: all three modules hold a copy of the vehicle ISN. The DDE can both verify CAS and answer EGS because it has the key. There is no evidence in these logs of a fourth party.

**Bench (EVIDENCE):** the F10 DDE alone sends only `26 85 F2 00 FF…` once on 0x592 — a status frame, not a challenge. Without a CAS peer and without an EGS peer, neither leg runs. Confirms that the exchange is peer-triggered / peer-dependent, not a DDE-only spontaneous burst of seed material.

**Second ignonoff wake (EVIDENCE):** DDE re-challenges CAS with opcode 0x11; EGS does not re-challenge. So the EGS leg is "once per KL15 session" in this sample, while the DDE↔CAS leg can fire again while awake (PUBLIC E-series CAS docs describe cyclic challenge until the engine runs — consistent, not proven for F11).

---

## 6. Public research (PUBLIC; not verified by us)

### Documented behaviour
- BMW EWS4 (CAS3+/CAS4 and F-series FEM/BDC) uses a **challenge–response** between DME/DDE and the body module after Terminal 15 ON. The DME generates a random number; both sides compute a response from it and a shared secret; start is enabled if they match. Source: BMW "Car Access Systems" training material (CAS3 / CAS4 sections).
- F-series long ISN / SK is **16 bytes (128 bits)**. Autohex / BMW-Explorer documentation lists EDC17C50 among supported DDE units for OBD or bench ISN read/write.
- 8HP EGS (E/F/G) stores an ISN and must be **virginised / reset**, then coded, to adapt to a new vehicle. Tools: Autohex ISN manager option 3, BMW-Explorer "Reset EWS to EGS 8HP", Abrites BN011/BN012, ISTA coding after reset. After virginise + code, PUBLIC Abrites notes say repeated ignition cycles (order of 5–8, ~1 min each) complete adaptation.

### What is **not** publicly documented (as far as these searches go)
- The exact algorithm that turns (challenge, ISN) → response. Training docs say only "the same calculation algorithm".
- Whether it is AES-based CBC-MAC / CMAC, a proprietary MAC, or something else. **AES is a common guess** because the challenge is 16 bytes and the SK is 16 bytes, but no public source we found states that for the DME↔CAS or DME↔EGS CAN exchange.
- Older EWS3 used a short (2-byte) ISN and a different scheme; F11 EDC17C50 + 8HP is in the long-ISN / EWS4-family generation (PUBLIC tool docs), so EWS3 algorithms are unlikely to apply.

### Tools people use (context only — not instructions)
- ISTA / E-Sys: coding after EGS ISN reset.
- Autohex, Abrites, BMW-Explorer: OBD or bench ISN read / EGS EWS reset / virginise.
- Bench flash / dump of DDE or EGS firmware: used by the same tool vendors to extract or write ISN when OBD is locked.

---

## 7. Non-handshake traffic on the same IDs (EVIDENCE)

Do not confuse these with the seed/key exchange:

| ID | Bus | Pattern | Role |
|---|---|---|---|
| 0x592 | PT-CAN | `26 85 F5/F6/FA …` every ~5 s | DDE status / alive on the service channel |
| 0x592 | PT-CAN | `30 01 …` / `32 01 …` once near engine start/stop | DDE event frames (fixed across logs) |
| 0x592 | PT-CAN | `40 22 00 14/15` | DDE short status after the handshake |
| 0x5C0 | PT-CAN | `40 …`, `48 00 F5 25 …` | CAS/FEM short frames (not ISO-TP multi-frame) |
| 0x598 | both | `13 FF 01 18 FF…` at 1 Hz while KL15 on | EGS service-channel status (node 0x18); appears on both buses after the handshake |

---

## 8. What is blocked without more data

| Goal | Why it is blocked now | What would unblock it |
|---|---|---|
| Recover the algorithm f(challenge, ISN) → response | 3–4 pairs, ISN unknown | Read ISN from DDE and EGS (§9); collect ≥50–100 independent pairs; or obtain firmware that implements f |
| Confirm AES / CMAC / other | No ciphertext structure visible | Same as above; or literature from a firmware RE paper |
| Test whether response = truncate(MAC(ISN, challenge)) | Need ISN + many pairs | ISN dump + replay of recorded challenges into a candidate implementation |
| See if a challenge ever repeats | 7 challenges, all unique | Capture ≥100 ign-on cycles; birthday bound for 128-bit is astronomical, so repeats would imply a weak RNG |
| Static-seed test (same challenge → same response?) | Never saw a repeated challenge | Inject a recorded challenge (on a bench, owned hardware) and watch the response |
| Confirm FEM vs BDC as 0x5C0 | Only the node address 0x40 | Module list from ISTA / wiring |

Cracking the MAC from pairs alone, without the ISN and without firmware, is not realistic for a 128-bit secret.

---

## 9. Data-collection plan (for Damien's own car / own ECUs)

**A. More ign-on captures (cheap, high value for statistics)**
1. Log PT-CAN + PT-CAN2 from before KL15 through ~2 s after, for **≥30** clean ign-on cycles (50–100 preferred). Engine need not crank.
2. Keep the same SavvyCAN setup (buses 2 and 3). One file per cycle, or one long file with clear KL15 gaps.
3. Deliverable: a CSV of (log, leg, challenge_16, response_6) — same columns as §3. Enough to (a) confirm no repeats, (b) run simple statistical tests, (c) later verify a candidate f once the ISN is known.

**B. Read the ISN / SK (own modules)**
1. From the **DDE EDC17C50**: OBD ISN read if the tool supports this build; otherwise bench / factory-mode read (PUBLIC: Autohex, Abrites, BMW-Explorer list EDC17C50).
2. From the **8HP EGS**: OBD or bench ISN read / EWS info (same tool families).
3. From **FEM/BDC**: ISN read (same). Compare all three — PUBLIC says they should match for a paired car.
4. Store the ISN offline; do not publish it.

**C. Static-seed / replay test (bench, owned hardware only)**
1. Bench the DDE alone with a CAN peer that can send a recorded EGS challenge (0x598, d0=0x82, 17 B ISO-TP) and receive 0x592 d0=0x85.
2. Replay the same challenge twice: if the response is identical, f is deterministic in (challenge, ISN) with no per-boot salt. If it differs, there is extra state.
3. Same test for the CAS leg by peer-simulating 0x5C0 (harder: the DDE is the challenger, so the peer must answer).

**D. Firmware**
1. A DDE or EGS dump that contains the MAC routine would settle the algorithm. Tooling for that is the same ISN-bench ecosystem; treat it as a separate, careful step.

---

## 10. EV conversion options (HYPOTHESIS guided by EVIDENCE + PUBLIC)

Both engine and gearbox are leaving this car.

| Kept in the F11? | Handshake impact | Practical option |
|---|---|---|
| Neither DDE nor EGS | Both legs are moot. Nothing on PT-CAN/PT-CAN2 will challenge or respond for immobiliser. | No emulator needed for this exchange. Other modules (FEM, GWS, …) may still expect NM / status; that is separate from ISN. |
| DDE kept (e.g. for FlexRay gateway behaviour) without EGS | DDE→CAS leg still runs; EGS→DDE leg will not (no 0x598 challenger). | CAS/FEM will still answer the DDE. Whether the DDE then enables whatever it gates is unknown without testing. |
| EGS reused standalone in another project car | EGS will send 0x598 challenges on PT-CAN2 and expect a 6-byte response on 0x592 within ~20 ms. | **Someone must answer.** Options (PUBLIC tooling context): (1) virginise the EGS and adapt it to the new car's DME/DDE ISN (ISTA/Autohex/Abrites/BMW-Explorer path); (2) keep a DDE (or emulator) that holds the matching ISN and implements f; (3) run an emulator that only ACKs at the ISO-TP layer — likely **not** enough if the EGS checks the MAC. |
| EGS kept in the F11 without DDE | Same as above: 0x598 will go unanswered unless something else responds. | Prefer virginise + adapt, or a small responder ECU with the ISN. |

**Recommendation for the F11 EV path where both are removed:** treat the immobiliser handshake as moot; do not spend effort emulating 0x592/0x598/0x5C0 unless a retained module still faults on their absence (watch for FEM / EGS DTCs after removal).

**Recommendation if the 8HP moves to another car:** plan on a proper EGS ISN reset + coding to that car, rather than reverse-engineering f. The algorithm work is only needed if you want a DDE emulator that satisfies an **unmodified** EGS that still holds this car's ISN.

---

## 11. Summary

- **Transport:** extended-address ISO-TP on 0x592 / 0x5C0 / 0x598; no FC; opcodes 0x10/0x11 (challenge) and 0x21 (response). EVIDENCE.
- **Crypto material:** 16-byte challenge, 6-byte response. All unique in our sample. EVIDENCE.
- **Roles:** DDE challenges CAS on PT-CAN; EGS challenges DDE on PT-CAN2. DDE holds the middle. EVIDENCE.
- **Algorithm:** unknown. AES-sized challenge + 128-bit ISN is suggestive but unproven. PUBLIC + HYPOTHESIS.
- **Blocked:** recovering f without ISN dumps and many more pairs (or firmware).
- **EV path:** if both DDE and EGS leave, the handshake is moot; if the EGS is reused elsewhere, reset/adapt it or supply a matching responder.

CSV of every multi-frame handshake frame: `analysis/immo_handshake_frames.csv`.
