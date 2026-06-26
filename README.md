# exciteBot

Building an **exact native C++ physics model of Excitebike (NES)** — a from-scratch
re-implementation of the game's bike physics that replaces the NES emulator inside a JaffarPlus-style
search.

**Why:** the goal is to beat the lap-1 reference TAS (finishes at frame 1468, `Bike Pos X > 6339`) by ≥1
frame. An emulator-based search keeps *tying* the TAS — it's ~92% emulation-bound and its ~2 KB state
saturates the search DB, so it can't go deep enough to convert the remaining sub-pixel slack into a full
frame. A native model is **10–100× faster** with a **tiny state**, giving the search the depth it needs.

**The hard requirement is fidelity.** The model must reproduce the real game exactly, or any "beat" it finds
is a phantom. Two disciplines enforce this:
1. The model must reproduce **`tas.ram`** (the reference TAS's per-frame RAM) **byte-for-byte** before any
   search is run.
2. Any model-found beat must be **replayed on the real emulator** (`reference/`) before it's believed.

## Contents

| Path | What it is |
|---|---|
| `excitebike_physics_model_handoff.md` | **Start here.** The full reference: reverse-engineered physics (addresses + table bytes), variable map, the two un-traced arithmetic pieces, the track-layout blocker + how to attack it, and the build plan. |
| `excitebike_FF.commented.asm` | (copyrighted, not included in repository) Annotated NES bank-FF disassembly (all findings inline). |
| `tas.ram` | **The validation harness.** 1469 frames × 2048 bytes = the reference TAS's raw NES RAM at every frame. Ground truth for fidelity checks. |
| `memory_disassembly-findings.md` | Dense reverse-engineering notes (companion to the handoff doc). |
| `reference/` | Self-contained kit to reproduce/verify the reference TAS on the real emulator. See `REPRODUCE_REFERENCE.md`. |
| `REPRODUCE_REFERENCE.md` | How to use `jaffar-player` to replay the reference and (re)generate `tas.ram`. |

## Reward / win (must match the old project exactly)
- **Reward = `Bike Pos X` ONLY** (no momentum/speed/other metric).
- `Bike Pos X = blockTransitions*256 + RAM[0x50] + RAM[0x394]/256` (sub-pixel).
- **Win = `Bike Pos X > 6339.0`.** Reference TAS reaches it at **frame 1468**.
