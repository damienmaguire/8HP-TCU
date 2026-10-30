# F11 8HP45 / EGS CAN findings

Source car: BMW F11 520d (VIN from notes; DDE PN 8513250). Captures: FlexRay MITM + PT-CAN + PT-CAN2, 2026-10-30.

## Files

| File | Contents |
|---|---|
| `egs_8hp_map.md` | What the EGS sends and can listen to, immobiliser handshake overview, confidence levels |
| `egs_8hp_ids.csv` | ID / bus / period / signals table |
| `f11_ptcan2_egs_dde.dbc` | PT-CAN2 DBC: DDE TX + EGS `0x1AF` / `0x3FD` |
| `egs_dde_immo_handshake.md` | Full immobiliser handshake structure (added when analysis finishes) |
| `immo_handshake_frames.csv` | Extracted challenge/response frames |

## Conventions

- Bus 2 = PT-CAN2, bus 3 = PT-CAN (SavvyCAN numbering from our captures).
- E2E: CRC8 poly `0x1D`, init 0, xorout 0, XOR a per-message key K; counter in the low nibble of byte 1 (0..14).
- Confidence is marked in each file. Speculative items are labelled candidate or left out.

## Immobiliser note

At every ignition-on the EGS does a random challenge/response with the DDE on PT-CAN2 (`0x598` / `0x592`). Replay is unlikely to work. The handshake write-up covers structure and what further data would be needed to reverse the algorithm.
