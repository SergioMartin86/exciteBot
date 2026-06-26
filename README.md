# exciteBot

Building an **exact native C++ physics model of Excitebike (NES)** — a from-scratch
re-implementation of the game's bike physics that replaces the NES emulator inside a JaffarPlus-style
search.

**Why:** the goal is to beat the full-race reference TAS (4 loops; the race completes at **frame 2261**, when
`Race Over Flag (0x52)` flips 0→1, final `Bike Pos X = 12157.328`) by ≥1 frame. An emulator-based search keeps
*tying* the TAS — it's ~92% emulation-bound and its ~2 KB state
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
| `tas.ram` | **The validation harness.** 2263 frames × 2048 bytes (4,634,624 B) = the full-race reference TAS's raw NES RAM at every frame. Ground truth for fidelity checks. (gitignored — regenerate via `REPRODUCE_REFERENCE.md`.) |
| `memory_disassembly-findings.md` | Dense reverse-engineering notes (companion to the handoff doc). |
| `reference/` | Self-contained kit to reproduce/verify the reference TAS on the real emulator. See `REPRODUCE_REFERENCE.md`. |
| `REPRODUCE_REFERENCE.md` | How to use `jaffar-player` to replay the reference and (re)generate `tas.ram`. |
| `source/engine.hpp` | The native C++ physics engine (`excitebike::Engine`) — the emulator replacement. |
| `source/player.cpp` | `exciteBike-player`: replays a `.sol` movie through the engine, mirroring `jaffar-player` (`--dumpRam`, `--printFinalState`). |
| `extern/jaffarCommon` | Git submodule — provides the JSON / string / file tooling `jaffar-player` uses. |

## Building

[meson](https://mesonbuild.com) + [ninja](https://ninja-build.org), C++20. The `jaffarCommon` submodule
must be present (`git submodule update --init`).

```bash
meson setup build
ninja -C build
```

This produces `build/source/exciteBike-player`. Replay the reference movie through the **native engine**
(seed frame 0 from `tas.ram`, since the boot/countdown isn't modeled yet):

```bash
./build/source/exciteBike-player reference/race01.jaffar reference/race01.sol \
    --printFinalState --initialRam tas.ram --dumpRam native.ram
# then diff native.ram against tas.ram to find the first divergence (the fidelity gate, handoff §9.4)
```

> **Status:** scaffold. `engine.hpp` implements the validated pieces (input→`0x5C` decode, the B/A speed
> caps, air-velX-constant, engine temperature, posX/block integration); the terrain profile (§8),
> downhill over-cap (§4g), and takeoff/arc/landing (§4c–§4e) are stubbed `// TODO`. The engine therefore
> does **not** yet reproduce `tas.ram` past the flat cruise — that is the next milestone.

## Reward / win
- **Reward = `Bike Pos X` ONLY** (no momentum/speed/other metric).
- `Bike Pos X = blockTransitions*256 + RAM[0x50] + RAM[0x394]/256` (sub-pixel).
- **Win = race completed = `Race Over Flag (RAM[0x52]) > 0`.** The reference TAS reaches it at **frame 2261**
  (final `Bike Pos X = 12157.328`). The full race is **4 loops** (`Current Loop` 0x3A4: 0→3,
  `Loops Remaining` 0x57). Beating it = trip the Race Over flag in **fewer than 2261 frames**.
- *(Historical note: `Bike Pos X > 6339` @ frame 1468 was the old lap/loop-1 milestone, now superseded by the
  full-race finish.)*
