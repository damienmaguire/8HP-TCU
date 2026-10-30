# F11 8HP / DDE CAN notes (Fred → Damien)

Drop into https://github.com/damienmaguire/8HP-TCU/tree/main/CAN (or merge alongside what is already there).

**Car:** BMW F11 520d, DDE EDC17C50 + ZF 8HP. Captures are Damien's own. Times Europe/Dublin.

| File | Contents |
|---|---|
| `egs_8hp_map.md` | EGS topology, TX IDs, gear/turbine/output signals, EWS burst pointer, what is not EGS. |
| `egs_8hp_ids.csv` | One row per ID: buses, period, E2E K, validated signals, confidence. |
| `egs_dde_immo_handshake.md` | Full immobiliser (ISN/EWS) handshake write-up: ISO-TP structure, pairs, entropy, public research, data-collection plan, EV-conversion options. Evidence vs hypothesis marked. |
| `immo_handshake_frames.csv` | Every multi-frame handshake CAN frame from the three ptcan1 logs (timestamps, raw hex, reassembled message, opcodes). |
| `f11_ptcan2_egs_dde.dbc` | Minimal PT-CAN2 DBC: confirmed DDE messages + EGS 0x1AF / 0x3FD. Loads in cantools 45.x with `strict=True`. |

Confidence words in the docs: **confirmed / high / medium / low / candidate / undecoded / hint**. Pure speculation is left out.
