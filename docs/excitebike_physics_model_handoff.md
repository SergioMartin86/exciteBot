# Excitebike (NES) — Native Physics Model: Session Handoff

**Purpose of this file.** Everything reverse-engineered about Excitebike's physics, plus the assets and
plan needed to build a **native C++ physics model** of the game that replaces the NES emulator inside a
JaffarPlus-style search. Written for a fresh session in a NEW repository. Source session reverse-engineered
the game from its disassembly + a frame-by-frame TAS RAM dump.

---

## 1. Mission & rationale

**Goal:** beat the Excitebike **lap-1 reference TAS** (race 1, "race01a") by ≥1 frame, OR rigorously
characterize the limit. Reward must be **EXCLUSIVELY the bike's posX** (no momentum/speed/other metric).

**Why a native model.** The current approach runs a reward-guided BFS state-DB search over the **QuickerNES
emulator**. That emulator is ~92% emulation-bound (6502 interpretation), and the per-state cost + ~2 KB
state make the search saturate its DB and only TIE the TAS (it tracks the TAS exactly, +0.000122, but never
pulls a full frame ahead). A native physics model would be **10–100× faster** and shrink the state from
~2 KB to a handful of bytes → the search could go vastly deeper/wider and actually crack the 1-frame
sub-pixel beat the emulator-bound search keeps tying.

**The one hard requirement: FIDELITY.** Any discrepancy between the model and the real game produces a
"beat" that won't replay on a real NES (a phantom). This is removed by two disciplines:
1. The model must reproduce the **full 1469-frame TAS RAM dump byte-for-byte** when fed the TAS inputs.
2. Any model-found beat must be **replayed on the real emulator** before it's believed.

---

## 2. Key assets & file locations (in the OLD repo `/home/jaffar/jaffarPlus`)

Copy these into the new repo / new session:

- **Annotated disassembly (PRIMARY reference):**
  `examples/nes/exciteBike/excitebike_FF.commented.asm`
  — a copy of the cyneprepou4uk `NES-Games-Disassembly` bank_FF.asm, with all findings commented INLINE
  plus a header summary block (lines ~1–61). This is the single most important artifact.
- **TAS RAM dump (THE validation harness):**
  `/tmp/claude-1000/-home-jaffar-jaffarPlus/058daf0b-6f0a-4413-bf8d-c3d0edd29644/scratchpad/tas.ram`
  — **1469 frames × 2048 bytes/frame** = raw NES low-RAM ($0000–$07FF) snapshot at every frame of the
  reference TAS. **MOVE THIS into the new repo — it is the ground truth.** (If lost, regenerate via the
  player's `--dumpRam`-style tooling replaying `race01a.tas.sol`.)
- **The reference solution / movie:** `examples/nes/exciteBike/race01a.tas.sol` (+ `race01a.jaffar` config).
- **The current emulator-based JaffarPlus game logic (for reference on integration + win condition):**
  `games/nes/exciteBike.hpp`.
- **ROM (gitignored, do NOT commit):** `examples/nes/exciteBike/Excitebike (JU) [!].nes`.
- **Memory notes** (in `/home/jaffar/.claude/projects/-home-jaffar-jaffarPlus/memory/`):
  `excitebike-disassembly-findings.md`, `excitebike-tas-beat.md`, `excitebike-lram-state-trim.md`,
  `excitebike-disassembly-findings.md` is the densest.

**TAS RAM dump format:** flat binary, frame `f` occupies bytes `[f*2048 : (f+1)*2048]`, and byte `addr`
within a frame is at `d[f*2048 + addr]`. All the variables below are single bytes at those addresses.

---

## 3. Variable map (player = object index 0; "obj" arrays are 4-wide: var V for obj i is at base+i)

| Addr | Name | Notes |
|------|------|-------|
| 0x004C | game cycle / player index | `CPX ram_004C` gates player physics |
| 0x004E | curr block X (nametable) | toggles 0/1; used with scroll for absolute X |
| 0x004F | race_started_flag | |
| 0x0050 | scroll_X = posX low pixel | |
| 0x0052 | track_finished_flag | also the "Race Over" flag; set at $DF7B and $EABE |
| 0x005C | THROTTLE/buttons | bit7(0x80)=A(slow), bit6(0x40)=B(turbo/fast); **low 2 bits index the airborne descent table** |
| 0x0060 | per-frame horizontal scroll increment | derived from velX; feeds scroll_X |
| 0x0070 | posZ1 (height) | |
| 0x008C | posY (lane / on-screen height during jump) | player vertical position |
| 0x0090 | velX low byte | |
| 0x0094 | velX high byte | **velX16 = 0x94*256 + 0x90** |
| 0x0098 | crash/wipeout flag | set to 0xFF by landing-angle safety check at $DC69 |
| 0x00AC | bike angle | player controls ONLY in the air; on ground it is terrain-forced |
| 0x00B0 | airMode | 0=ground, 2=airborne, 1=wobble/recovery. `LSR`→Y for tables |
| 0x00B8 | posZ2 (airborne Z subpixel) | the breadth driver in the emulator search |
| 0x00BC | climbRemaining ($BC) | >0 while climbing, hits 0 at apex, stays 0 through descent |
| 0x00D4 | slope | terrain slope index, from tbl_E6AD[angle-2] on the ground |
| 0x00DC | velZ-ish (vertical bounce) | EOR #$FE toggling at $E9DF |
| 0x0270/0x0274 | velY1/velY2 | |
| 0x0378/0x037C | launch velZ (16-bit) | set at takeoff = velX16 + 175 |
| 0x0380/0x0384 | airborne accumulators ("Flight" bytes) | 0x380 init 0x0F at takeoff |
| 0x0388 | launch type / jump strength | 0 or 1 at ramp; 2 after a wobble landing (halves next launch) |
| 0x0394 | posX subpixel | accumulates velX/256 per frame |
| 0x03B6 | engine temp | |

**Absolute posX (the reward):**
`posX = blockXTransitions*256 + scroll_X(0x50) + posX2(0x394)/256`, where `blockXTransitions` = a running
count of 256-px blocks passed (NOT in RAM — maintained externally; increments when 0x004E toggles).
**Win condition: posX > 6339.0** (sub-pixel). TAS reaches it at **frame 1468**.

---

## 4. The reverse-engineered physics (with addresses + exact table bytes)

### 4a. Ground speed model — `sub_CE29` (accel) / `sub_CE58` (friction), called from $CDAC/$CDBA
- `velX16 += tbl_C0BC[Y]` (16-bit add into 0x90/0x94), then **HARD-CLAMP** `velX16` to
  `(tbl_C0D1[Y]<<8 | tbl_C0CE[Y])` if it exceeds — i.e. forced down to the cap (at $CE4D).
- Friction `sub_CE58`: `velX16 -= tbl_C0C1[Y]`.
- **Y by throttle:** A(bit7)→Y0; B(bit6)→Y1; track_finished→Y2; airborne→friction-only (NO accel).
- **CRITICAL:** the accel routine (and its clamp) runs **only when an A/B button is held**. With NO button
  (coast), no Y is selected, the clamp never runs, and **gravity can push velX past the cap freely**.

**Exact tables** (Y index 0=A, 1=B, 2=finished; bytes are hex):
- `tbl_C0BC` accel = `[18(24)=A, 3F(63)=B, 28(40)=finished, 20(32)=air?, 28(40)]`
- `tbl_C0C1` friction = `[38(56), 0C(12), 00, 3C(60), 1C(28), C0(192), 7F(127)]`
- `tbl_C0CE` cap-lo = `[20, 40, 7F]`  ; `tbl_C0D1` cap-hi = `[03, 03, 01]`
  → caps: A = 0x0320 = **800**, B = 0x0340 = **832**, finished = 0x017F = **383**.
- `tbl_C0CC` (friction-floor compare) = `[01, B0, 20, 40, 7F, 03, 03, 01]` (overlaps next table; re-dump
  exact length from the asm).
- **(Re-dump exact per-table lengths from the asm — they are contiguous in ROM so the dumps above run long.)**

### 4b. Engine temperature — $E36B–$E3C0
- equilibrium temp `tbl_D8FB[Y]` = `[08(8)=coast, 20(32)=B, 11(17)=A]`
- approach rate `tbl_D8F7[Y]` = `[3F(63)=coast, 0F(15)=B, 07(7)=A]` (added to subcounter 0x3B5; carry steps
  temp 0x3B6).
- **OVERHEAT STALL when temp ≥ 0x20 = 32** (sets stall obj 0x3E0 + timer 0x3C). B's equilibrium == the stall
  threshold → holding B drives temp TO stall (self-limiting). Coast equilib 8 cools fast.
- The TAS never overheats: it coasts during jumps (throttle inert in air, and air-coast cools toward 8), so
  jumps provide the cooling that lets B be held on the ground. **Temperature is NOT a lever (self-managed).**

### 4c. Takeoff / launch — `sub_DD38` (from ramp handler `ofs_002_E934` → `sub_DD06`)
- Sets `airMode = 2`, `0x380 = 0x0F`.
- **Launch velZ (0x378/0x37C) = velX16 + 0x00AF (175).** Faster entry → higher/longer arc.
- HALVED (`LSR/ROR`) if `0x388 == 2` — only after a wobble landing ($DC81 sets 0x388=2). Normal ramp sets
  `0x388` = 0 (fast) or 1 (slow, at $E94A). Min-speed gate: `velX_hi == 0` → no launch.

### 4d. Airborne arc integration — `sub_DD6F` (from $DD29)
- `0x380/0x384 += 0x38C` (16-bit), where **`0x38C = tbl_D868[0x5C & 3]`** = `[34(52), 34(52), 18(24), 34(52)]`.
  → the **low 2 bits of the input byte (the in-air LEAN) pick the per-frame vertical-accel term**; index 2 (24)
  is the odd one out (slower term). Then height `0x8C` is updated using launch velZ (0x37C) and the accumulator.
- **AIR velX is CONSTANT** (player air-friction ≈ 0): a jump holds its takeoff speed exactly
  (832→832 … 1544→1544). **VALIDATED: 733/735 = 99.7% of airborne frames have velX[f+1]==velX[f].**

### 4e. Landing safety / wobble — `sub_DC1A` landing resolve ($DC51–$DC6D)
- Safe landing iff `tbl_D86C[Y] ≤ angle(0xAC) < tbl_D87C[Y]`, with `Y = slope(0xD4) + (8 if velX_hi<2 else 0)`
  (fast band Y=slope 0–7, slow band Y=slope+8 8–15, slow band slightly WIDER).
  - `tbl_D86C` (lo, 16 entries) = `03 02 03 02 09 06 08 0F | 03 02 02 02 08 05 07 0F`
  - `tbl_D87C` (hi, 16 entries) = `0C 09 0A 07 0C 0C 0C 00 | 0C 0A 0B 08 0C 0C 0C ..`
  - slope 7 fast window `[0F,00)` = EMPTY = forced crash.
- Outside window → crash flag 0x98 = 0xFF ($DC69).
- **PERFECT landing** (angle == `tbl_D88B[slope]` = `06 03 04 02 0B 08 09`) → routed to 0x368, **skips the
  wobble** (airMode→1 at $DC7B) and **keeps speed**. The TAS lands perfectly EVERY jump (airMode==1 occurs
  **0 times** in the dump). So the wobble penalty is never incurred in the optimal route.

### 4f. Ground terrain-following — $E7A9–$E7C8
- Each ground frame reads the track-data nibble (`AND #$0F`) → **FORCES** bike angle `0xAC = nibble`, and
  slope `0xD4 = tbl_E6AD[angle-2]` where `tbl_E6AD = 03 01 02 02 00 05 05 06 04 04`.
- ⇒ on the ground the player does NOT control the angle (terrain does); the player controls the angle ONLY
  in the air. LATENT OOB: nibble 0/1 → angle-2 underflows → OOB table read, but the TAS angles are always
  2..11 (flat=6), never 0/1, so race01 never triggers it.

### 4g. The downhill / over-cap speed (the 1088→1544 escalation)
- velX past the 832 flat cap is **purely slope (track-geometry) determined and input-independent**: the TAS
  COASTS downhills (so the clamp doesn't run) and gravity builds velX to each slope's terminal velocity
  (observed plateaus: 832 → 1088 → 1288 → 1344 → **1544**, the steepest, held 273 frames into the finish).
- **NOT yet fully traced:** the exact code/arithmetic that ADDS to velX (0x90/0x94) on downhills beyond the
  clamp. This + the exact arc fixed-point (4d) are the two pieces of arithmetic the model still needs nailed.
  Likely tied to slope (0xD4) and/or 0x60 (the scroll increment). **First reverse-engineering task.**

---

## 5. The macro-structure of the race (from the TAS RAM dump)

- **Airborne 50.2%** of the race (737/1469 frames), **9 jumps**, airborne run lengths
  `[77, 56, 74, 78, 61, 58, 63, 143, 127]`.
- velX timeline: **pinned 832 on the FLAT** (f0–~1063, hard cap, 1006 frames at exactly 832), then a
  **DOWNHILL** accelerates 832→1088→1344→**1544** (f1063–1196), held to the finish.
- posX advances **exactly velX/256 per frame** (verified: constant 3.25 px + 64 sub-pixel/frame at velX 832).
  ⇒ **during a jump, posX is a deterministic function of (entry posX, velX, frames-aloft) and carries NO
  search information** — a jump's only posX-relevant output is its LANDING (frame + re-entry point). This is
  the central insight: airborne breadth is pure waste; the model should treat each jump as one transition.

---

## 6. Where a beat can come from (conclusions — keep an open mind, [[never-call-tas-optimal]])

- **No throttle/speed/temperature slack:** flat = hard-capped 832 (held), downhill = coast-to-terminal,
  air = maintains speed; jumps are forced and speed-neutral at the cap. Temperature self-manages via jump
  cadence. All proven.
- **Finish trigger has no coarsening:** win = `posX > 6339` (sub-pixel), reward = posX (sub-pixel), both posX
  bytes hashed. Sub-pixel phase is fully exposed.
- **RNG is irrelevant:** the only RNG (0x1B LFSR, `ROR ram_001B` at $D33C) drives only opponent lanes, and the
  player's lane band is kept clear (opponents parked off-track), so RNG cannot touch posX.
- **The ONLY remaining lever is sub-pixel PHASE** at slope-boundary crossings — reaching the downhill /
  high-speed sections a fraction earlier via accumulated sub-pixel. The emulator search probes this and TIES
  at frame 1468. A faster/deeper native search is the best shot at converting it to a full frame.
- Untested but very likely dead (the dynamics argue against them): lap/finish early-trigger slack; a higher
  downhill terminal velocity; recovering a wobble (TAS never wobbles).

---

## 7. JaffarPlus integration notes (how the OLD emulator-based game is wired — for reference)

- `games/nes/exciteBike.hpp`: reward = `_bikePosX` (posX, sub-pixel). Win rule in `race01a.jaffar`:
  `Bike Pos X > 6339.0` → Trigger Win. Fail rule on `Crash Flag (0x98) > 0` forces crash-free play.
- It already has a **commit-and-hold token scheme** + a partially-built **macro-airborne** flag
  (`Macro Airborne`, currently a no-op stub) — the native model is the "all the way" version of that idea
  (simulate jumps in native physics instead of via the emulator).
- The native model would expose the SAME interface a JaffarPlus `Game` needs: a tiny state struct
  (posX hi/lo/sub, velX16, posY, airMode, angle, slope, temp, blockX, climbRemaining, the few flight
  accumulators), `advanceState(input)`, `serialize/deserialize`, `getReward()=posX`, `isWin()=posX>6339`,
  `isFail()=crashed`. State could be ~16–32 bytes vs ~2 KB.

---

## 8. THE BLOCKER & how to attack it: the track layout

The physics needs the **terrain profile** — the slope/ramp/downhill at each X (and lane). The ground code
reads it via a jump table indexed by terrain type:
- `sub_E794` ($E794): `ASL; TAY; LDA tbl_E6B7,Y` → an indirect `JMP (tbl_E6B7+Y)` = a per-terrain-type
  handler dispatch (the `ofs_002_*` handlers, e.g. `ofs_002_E934_06` = the ramp/jump type-06 handler).
- The **track LAYOUT** (the sequence of terrain types per X for race 1) is level data in ROM, read via the
  scroll position. **Locating + dumping this is the make-or-break first task.**

**Two extraction paths:**
1. **From ROM:** trace the track-data pointer (the level-data read driven by scroll_X / block X) and dump the
   byte stream for race 1. Fully general (gives off-path lanes too).
2. **From the TAS RAM dump (shortcut):** the dump already contains slope (0x D4), angle (0xAC), airMode (0xB0)
   per frame ALONG the optimal path. Since we proved the route is geometry-forced with no lane slack, the
   **on-path terrain profile (slope vs X) extracted from the dump may be sufficient** to model the forced
   route and search the sub-pixel/timing micro-variations. Start here — it's immediate and needs no ROM
   tracing — and fall back to (1) if off-path exploration turns out to matter.

---

## 9. Suggested build plan for the new session

1. **Bring assets over:** the annotated asm, the `tas.ram` dump, `race01a.tas.sol`, this file.
2. **Extract the terrain profile** (Section 8 — start from the RAM dump).
3. **Implement the native model in C++**: ground speed (4a) + temperature (4b, optional — only as a B-hold
   gate) + takeoff (4c) + arc (4d) + landing (4e) + ground-follow (4f) + downhill over-cap (4g). Nail the two
   un-traced arithmetic pieces (4d exact fixed-point, 4g downhill add) against the dump.
4. **VALIDATE:** feed the TAS inputs (from `race01a.tas.sol`) into the model and assert it reproduces the
   `tas.ram` dump **frame-by-frame** (velX16, posX, airMode, angle, slope, posY, temp). Iterate until exact
   for all 1469 frames. This is the gate — do not search until the model is byte-faithful.
5. **Search** posX with a JaffarPlus-style BFS (or a custom search) over the tiny native state. Keep sub-pixel
   in the dedup hash (it's the only lever). Reward EXCLUSIVELY posX.
6. **Confirm any beat on the real emulator** (replay the found input sequence on QuickerNES; check posX/frame)
   before claiming it. A model-only "beat" is not a beat.

---

## 10. Operational directives that carry over (from the user, persistent)

- **Reward EXCLUSIVELY posX** — no momentum/speed/other metric, ever.
- **Never accept a best reward behind the reference TAS**; find the exact first divergence and fix that.
- **Never call a TAS/WR "optimal"** — report exhaustive negatives as "no beat found / search-limited".
- **Long runtimes are fine** (hours/days); a full state DB is NOT a cancel reason (only a collapsed
  best/worst reward spread, or best falling behind the reference, is).
- Hardware: EPYC 9755, **256 threads** for all multithreaded runs; pin via OMP. Launch long runs detached
  (`setsid`). Watch CPU temp (k10temp Tctl) on heavy runs.
- Git: commit directly to **master** (solo repo); commit/push only when asked; push over **HTTPS** (no SSH
  key in the sandbox); keep ROM/.nes assets gitignored.
- Cap build/test parallelism on the 256-core box (never bare `ninja`/`meson test`).
- Keep all Excitebike wording benign/game-related.

---

*Validation status at handoff: speed model + "air velX constant" verified 99.7% against the TAS RAM dump;
finish/RNG/wobble levers closed; the two un-traced arithmetic pieces (exact arc fixed-point, downhill over-cap
add) + the track-layout extraction are the remaining reverse-engineering work before a faithful model exists.*
