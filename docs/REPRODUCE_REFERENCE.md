# Reproducing the reference TAS with `jaffar-player` (instructions for the new-session Claude)

This explains how to replay the lap-1 reference TAS on the **real NES emulator** and how to (re)generate the
`tas.ram` validation harness. **All commands below are verified working** (run on this machine, 2026-06-26).

The kit in `reference/` is **self-contained** — it bundles the correct prebuilt player binary, the ROM, the
config, the raw input movie, and the boot sequence. You do **not** need the old jaffarPlus repo to run it.

```
reference/
  jaffar-player              # CORRECT prebuilt binary (raw-input replay; do not substitute — see GOTCHAS)
  race01a.jaffar             # JaffarPlus config (QuickerNES; win = Bike Pos X > 6339)
  race01a.tas.sol            # the reference movie: 1467 lines of RAW per-frame NES inputs (e.g. |..|......B.|)
  race01a.initial.sol        # boot/countdown sequence (REQUIRED by the config; runs before the movie)
  Excitebike (JU) [!].nes    # ROM (SHA1 2E9897846E54A4A9865E87DE7517C6710BDEC255)
```

## 1. Reproduce the reference (sanity check: should WIN at frame 1468)

Run **from inside `reference/`** (the config uses relative paths for the ROM and boot sequence):

```bash
cd reference
./jaffar-player race01a.jaffar race01a.tas.sol \
    --printFinalState --disableRender --unattended --exitOnEnd 2>&1 \
  | sed 's/\x1b\[[0-9;]*[A-Za-z]//g' \
  | grep -aoE '(Current Step:[ 0-9]*[0-9]{4}|Bike Pos X:[ 0-9]*[0-9]+\.[0-9]+ \([0-9 ,]+\))' | tail -2
```

**Expected output (the pass condition):**
```
Current Step:1468
Bike Pos X:6339.203 (195, 52)
```
`6339.203 > 6339` at step **1468** = the reference win. `(195, 52)` are `RAM[0x50]=195`, `RAM[0x394]=52`.

> The `sed` strips the player's ncurses escape codes (this binary renders an interactive display even with
> `--unattended`; `--exitOnEnd` makes it run to the end and quit). The replay itself is correct; the `sed`
> just cleans the captured text.

## 2. (Re)generate the `tas.ram` validation harness

`tas.ram` is already provided in the repo root, but to regenerate or verify it:

```bash
cd reference
./jaffar-player race01a.jaffar race01a.tas.sol \
    --dumpRam /path/to/out_tas.ram --disableRender --unattended --exitOnEnd
# Verify it matches the bundled harness:
cmp out_tas.ram ../tas.ram && echo "identical"
```

`--dumpRam` writes the NES low RAM (`$0000–$07FF`, 2048 bytes) **once per frame**, concatenated:
**1469 frames × 2048 bytes = 3,008,512 bytes**. Frame `f`, byte `addr` → offset `f*2048 + addr`.
This is the file your native model must reproduce byte-for-byte.

## 3. How to validate your native model against the real game

1. Read the raw inputs from `reference/race01a.tas.sol` (one NES input per line, frames 0..1466; the bike is
   already past the boot countdown — `race01a.initial.sol` handles that, and `tas.ram` frame 0 is the first
   post-boot frame).
2. Feed those inputs into your model and dump your model's RAM (or just the modeled variables) per frame.
3. Assert your per-frame values equal `tas.ram` at the addresses in the handoff doc's variable map
   (velX16 `0x90/0x94`, posX `0x50`+`0x394`, airMode `0xB0`, angle `0xAC`, slope `0xD4`, posY `0x8C`,
   temp `0x3B6`, …). Iterate until **all 1469 frames match exactly**. Only then start searching.

## 4. GOTCHAS (these cost time in the old project — read them)

- **Use the bundled `reference/jaffar-player` binary as-is.** It was built from the *raw-input-mode*
  `exciteBike.hpp`. Other old build dirs (e.g. `build-eb-baseline`) embed a *commit-token-decoding* version
  of the game logic that **silently ignores the raw `.sol` inputs and diverges** — replaying the same movie
  there ends at `Bike Pos X ≈ 3226`, not 6339. If you ever rebuild the player from the old jaffarPlus repo,
  make sure `race01a.jaffar` carries **no** `"Commit And Hold"` / `"Airborne Per Frame Angle"` flags (absent
  → the game applies inputs raw → faithful replay).
- **`race01a.tas.sol` is RAW inputs**, not commit tokens (no `S`/Start markers). It only replays faithfully
  through the raw-mode game logic.
- **`race01a.initial.sol` is required** — it's the boot/countdown the config plays before the movie. Keep it
  next to the config.
- The player needs standard system libs (`libSDL2`, `libncurses`, `libstdc++`, …); all resolve on this
  machine. `--disableRender` avoids needing a display.
- The ROM path in `race01a.jaffar` is **relative** (`Excitebike (JU) [!].nes`), so always run from
  `reference/`.

## 5. Useful `jaffar-player` flags (reference)

`./jaffar-player <config.jaffar> <solution.sol> [flags]`
- `--printFinalState` — print the full game state each step (incl. `Bike Pos X`).
- `--dumpRam <path>` — write per-frame NES RAM (the harness generator).
- `--dumpHashes <path>` — write per-frame state hashes.
- `--disableRender` — no SDL window.
- `--unattended` — don't wait for interactive keypresses.
- `--exitOnEnd` — quit at the last step (otherwise it idles at the end).
- `--saveStateStep N --saveStateFile <path>` — dump a savestate at step N (useful for seeding).
