# Excitebike native 2D engine — implementation spec

Reverse-engineered model for a from-scratch, byte-exact native engine that replaces the emulator in a
JaffarPlus-style search. Reward is EXCLUSIVELY posX. This spec is the result of tracing the bank-FF
disassembly (`docs/excitebike_FF.commented.asm`) against the ground-truth RAM dumps (`tas.ram`,
`manual.ram`). All findings here are validated against those dumps unless marked OPEN.

Companion data/tools:
- `source/track_layout.hpp` (generated, gitignored): `kTrack[6][1376]` = terrain tile per (lane, column).
- `tools/extract_track_layout.py`: regenerates the above from the dumps (cross-validated, 0 conflicts).
- `tools/sim.py`: Python reference sim of the section machinery + handlers.
- `tools/sim_arc.py`: Python reference sim of the airborne arc + landing/bounce.
- `tools/validate.py`: reward (posX/velX) validator for the C++ engine vs a dump.

The reward/speed model (velX, posX, temperature) is ALREADY byte-exact in `source/engine.hpp`; this spec
covers the TRACK/VERTICAL layer that must be added to make jumps/terrain predictive in 2D.

---

## 1. Coordinate system

- **Cumulative column** = the velocity-independent track position. Increments via the `0x64`/`0xE0`
  counter; ~9 px per column. `0xE0 = (initialE0 + cumColumn) mod 64`.
- **Lane** `0x360` ∈ {0..5}, derived from vertical position `0xB8` (sub_D0AB).
- Terrain tile at the bike = `kTrack[lane][cumColumn]`. `tile >= 0x40` ⇒ section marker.

## 2. Per-frame routine order (from sub_C99A, the active-race state)

1. **input latch**: `0x5C = 0x14` (btn_hold) — buttons A=0x80,B=0x40,U=0x08,D=0x04,L=0x02,R=0x01.
2. `sub_DD8D` overheat/stall (inert in references).
3. **`sub_E733`** section machinery (per object) — selects sections, scans features, DISPATCHES terrain
   handlers incl. the ramp/downhill TAKEOFF. Runs BEFORE this frame's position update (so it reads the
   PREVIOUS frame's `0x60`/`0x64` and the previous frame's `0xC0`).
4. (opponents — not modeled; removed by freezing screen-X 0x81/0x82/0x83=0)
5. **`sub_DDD1`** crash/reset; **`sub_CD1F`→sub_CD59`** speed routine (already byte-exact); 
   **`sub_E96C`** vertical/lane movement; `sub_E836` recompute visual `0x8C` from `0xB8` on the ground.
6. **`sub_DA26`** (per object): `sub_DA58` posX-part1 → `sub_DC1A` LANDING → `sub_DCF2` ARC → `sub_DCDE`
   wobble-timer-dec; then `sub_DBFE` posX-part2.
7. **`sub_E70B`** column counter (late) — uses THIS frame's `0x60`.

## 3. Column counter (sub_E70B) — EXACT

Per frame, with this frame's whole-pixel advance `0x60` (= velX_hi + sub-pixel carry, computed in the
posX update): if `0x60 != 0`: `bd = (0x64 - 0x60) & 0xFF`; if `bd==0 || (bd&0x80)`: `0x64=(bd+8)&0xFF`,
`0xE0=(0xE0+1)&0x3F` (cumColumn++); else `0x64=bd`. 0 mismatches on both dumps.

## 4. Section machinery (sub_E73B) — BYTE-EXACT (0x58/0xC4/0xC8 = 0 mismatches both dumps)

Per frame (player): `0xC8 += prev0x60`. If `0x58==0`: read `c0` (= terrain marker, see §4.1); if
`c0>=0x40` and `((c0-0x40)>>2) < 22`: `0x58 = ((c0-0x40)>>2)+1; 0xC4=0; 0xC8 = prev0x64 - 1`. Then scan
features of block `0x58-1` (track_sections.txt "sec N" = block N; 0x58=N reads block N-1): for each
`(pos,type)` while `pos <= 0xC8`: dispatch (§5), `0xC4 += 2`; `0xFF` terminator (cursor past end) →
`0x58=0; 0xD4=0`.

### 4.1 `0xC0` source / streaming timing — SOLVED (0 mismatches both dumps)
`0xC0` is set each frame by the renderer `sub_D0C6` → `sub_E7FC`, which reads
`buffer[lane][E0]` where `buffer` = the per-lane terrain row at base `tbl_E54E/tbl_E554` =
`{0x400,0x440,0x480,0x4C0,0x500,0x540}` (6 lanes × 64-byte circular slots), indexed by `0xE0`.

The streaming was EMPIRICALLY characterized from the per-frame buffer snapshots (tools/stream_probe.py):
each cumColumn increment writes slot `(E0+42)&63 = master[lane][cum+42]` for ALL 6 lanes — a rock-solid
constant 42-column lookahead (zero variance). So in steady state `buffer[lane][E0] = master[lane][col]`.

The CLOSED FORM that reproduces `0xC0` byte-exact (0 mismatches on 2263 TAS + 5533 manual frames,
tools/stream_check.py / c0_index.py):

  **`0xC0(frame i) = kTrack[lane_i][cum_{i-1}]`**  — CURRENT frame's lane, PREVIOUS frame's cumColumn.

Why the split: the lane `0x360` is updated EARLY in the frame (sub_E96C) so the render samples the
current-frame lane; but `0xE0`/cumColumn is updated LATE (sub_E70B, end of frame) so the column index
is the previous frame's. Equivalently `buffer[lane_i][E0_{i-1}]`. No circular buffer is needed in the
engine — just keep `kTrack` and index by current lane + previous cumColumn. (The earlier "404 cross-run
conflicts" were an artifact of keying by the current cum instead of the previous cum at lane changes.)

With this rule, section machinery (sub_E73B) is byte-exact: `0x58/0xC4/0xC8` = 0 mismatches both dumps,
manual takeoffs 27/27.

## 5. Terrain handlers (dispatch by terrain type byte)

Strip high bits first: `type & 0x80` → ground-angle handler; else `type & 0x40` → E7F2; else
`tbl_E6B7[type]`.
- **Ground angle (E7A3)** [types 0x80-0x8F]: if `airMode==0 && crash==0 && 0xA4!=1`: `0xAC = type&0x0F`;
  if `0x58!=3`: `0xD4 = tbl_E6AD[0xAC-2]`, `tbl_E6AD = [3,1,2,2,0,5,5,6,4,4]`.
- **E7F2** [types 0x40-0x4F]: `0xCC = type&0x0F; 0xD0 = pos`.
- **Ramp 0x06 (E934)**: if `airMode==0 && velX_hi!=0`: set launch type `0x388` (0 fast / 1 slow), then
  TAKEOFF (§6.1).
- **Downhill 0x13 (E8E7)**: if `!crash`: `0x94++` (velX += 256, the §4g over-cap boost); if `airMode==0`:
  TAKEOFF.
- **Crash 0x07 (E818)**: if `airMode==0 && 0xAC<7 && (velX_hi>=3 || (velX_hi==2 && velX_lo&0x80))`:
  `0x98=0xFF`.
- **0x00 (E963)**: `0xCC=0; 0xD4=0; 0xB4=0`.
- Types 01-05,08-0C,0E-0F,10,12,14 (E845/E854/E85D/E86A/E879/E89D/E8BF/E8C6/E8E3/E8EE/E8FF/EA8F/E956):
  not yet modeled. NOTE several of these set `0xD4`/`0xB4` via `sub_E893` (`0xD4=0xB4=A`) and `0xCC`,
  and DO run while airborne (unlike E7A3 which skips when airMode!=0). E.g. type 0x09 (E879) calls
  sub_E893 with A=4. These are the remaining source of the residual `0xAC`/`0xD4` mismatches in
  tools/sim.py (see §6.4); the section machinery + takeoff timing is already byte-exact without them.

## 6. Airborne arc (sub_DD6F) + landing/bounce (sub_DC1A/sub_DCF2) — climb EXACT; landing decision OPEN

### 6.1 Takeoff (sub_DD38)
`airMode=2; 0x380=0x0F; 0x378=velX_lo+0xAF; 0x37C=velX_hi+carry` (launch velZ16 = velX16+175);
if `0x388==2`: `LSR 0x37C / ROR 0x378` (halve). The ramp path also sets `0x384=0` (via loc_DCFA) and
does one immediate arc step (loc_DD1A).

### 6.2 Arc step (each airborne frame, sub_DCF2 → loc_DD1A → sub_DD6F)
Gate: skip if `(0x5C&1)==1 && 0x4C==0`. Else, carry-in `Cin = 0x5C&1`:
`0x38C = tbl_D868[0x5C&3]`, `tbl_D868 = [52,52,24,52]` (lean 2/LEFT → 24 = weaker gravity → longer jump
— a SEARCH LEVER); `0x380 += 0x38C + Cin` (→c1); `0x384 += c1` (→c2);
`0x8C = (0x8C - 0x37C - (1-c2)) + 0x384 + c3` where c3 = carry out of the SBC. The single integration
produces the whole parabola (climb while `0x384 < 0x37C`, descend after). VALIDATED exact through descent
to first ground contact.

### 6.3 Landing + bounce (sub_DC1A, runs before the arc in sub_DA26)
groundY = `0xA0 - 0xBC - 0xB8`. Ground contact when `0x8C > groundY && 0x8C < 0xA8 && 0x364 != 0`.
On contact: `airMode=0; 0x364=0; oldJ=0x388; 0x388=0; 0x8C=groundY-1`.
- if `oldJ != 0` (post-bounce): resolve SKIPPED → clean final landing (airMode stays 0).
- else (first contact) resolve: `Y = 0xD4 + (8 if velX_hi<2)`; if `0xAC < tbl_D86C[Y]` or
  `0xAC >= tbl_D87C[Y]` → CRASH `0x98=0xFF`; elif `0xAC == 0x368` (perfect; `0x368 = tbl_D88B[0xD4]` via
  sub_DCA0) OR `0xD4|0xCC != 0` → clean landing; else → WOBBLE: `0x374=4; 0x388=2; airMode=1`.
- if airMode became 1, `sub_DCF2` (airMode==1 path) RE-LAUNCHES via sub_DD38 (halved velZ since 0x388==2)
  and the bike BOUNCES (airMode back to 2); next contact is always clean. So a jump = one arc + at most
  one bounce. End-of-frame `airMode` never shows the transient 1.

Tables: `tbl_D86C = [03,02,03,02,09,06,08,0F, 03,02,02,02,08,05,07,0F]`,
`tbl_D87C = [0C,09,0A,07,0C,0C,0C,00, 0C,0A,0B,08,0C,0C,0C,..]`, `tbl_D88B = [06,03,04,02,0B,08,09]`.

### 6.4 Angle 0xAC + slope 0xD4 — BYTE-EXACT (non-crash) on both dumps
RESULT (tools/sim.py): TAS `0xAC`/`0xD4` = 0 mismatches (all 2263 frames). Manual `0xD4` = 0; `0xAC` =
41 residuals, ALL inside the single crash at f2939 (crash animation ramps 0xAC down 0F→05, then recovery
resets it to 6) — irrelevant to a posX-optimal search (any crashing trajectory is strictly slower → pruned).
The crash 0xAC animation/recovery (0x98=0xFF state, 0x9C recovery) is intentionally NOT modeled.

Three mechanisms were required to reach exactness:
1. **sub_E927** (runs right after sub_E733): each frame, for the held effect `0xCC != 0`, re-dispatch
   `tbl_E6B7[0xCC]` — so a slope set by an E7F2 feature (0xCC=nibble) PERSISTS, re-applying its handler
   (e.g. CC=3 → handler 0x03 → sub_E893 → `0xD4=0xB4=1`) every frame until 0xCC is cleared (type 0x00).
   This is the dominant 0xD4 source. The track's E7F2 types {0x42,0x43,0x44,0x45,0x49,0x4A} → CC ∈
   {2,3,4,5,9,0xA} → handlers {0x02,0x03,0x04,0x05,0x09,0x0A}, all of which set 0xD4 via sub_E893.
2. **Dispatch handlers set 0xD4 = 0xB4 = A via sub_E893** (UNCONDITIONALLY, no airMode gate):
   type 0x02→6, 0x03→1, 0x04→5, 0x05→2, 0x09→4, 0x0A→3 (skip if 0xA4==1), 0x00→0.
3. **Pre-landing airMode**: sub_E733 (handlers) and the sub_CD59 angle tail run BEFORE the late landing
   (sub_DA26), so they see the START-of-frame airMode (= end of previous frame, +this frame's takeoff),
   NOT the dump's post-landing end-of-frame 0xB0. This sets `Y = airMode>>1` correctly for the tilt
   (table index, timer reload) on landing frames.

#### Angle update detail (lean/tilt/wheelie at tail of sub_CD59)
Runs each frame at the tail of the speed routine (sub_CD59, loc_CDBD onward). `ram_000A = 0x5C & 0x03`
= the tilt buttons (**R=0x01, L=0x02**; U/D=0x0C are lane-change, handled by sub_E96C). Order:
- if `0x98` (crash) != 0 → skip all angle.
- decide LEAN-ACTIVE: if `airMode != 0` → active (air). Else on ground active iff
  `(0x58|0x52)==0` (not in a section, not finished) AND `(velX_hi!=0 || velX_lo>=0xA0)` (fast enough).
- if LEAN-ACTIVE and `ram_000A != 0` → **loc_CE83** (player tilt). else if `airMode==0` and
  `0x368 != 0xAC` → **loc_DCC7** (ground ramp toward rest angle 0x368).

**loc_DCC7 (ground ramp toward 0x368):** if timer `0x2A != 0` → return; else `0x2A = 5` (reload Y),
then move `0xAC` one step toward `0x368`: if `0xAC==0x368` return; if `0xAC < 0x368` → `0xAC++`; else
`0xAC -= 1` (DEC,DEC then a shared INC = net −1).

**loc_CE83 (player tilt / wheelie):** if timer `0x26 != 0` → return; `Y = airMode>>1` (0 ground / 1 air);
`0x26 = tbl_C0D4[Y]` (`tbl_C0D4=[6,4]`). Test `t18 = ram_000A` bit0 (R):
- R held (bit0=1): move `0xAC` toward `tbl_C0C8[Y]` (`[6,2]`): `==`→ret; `<`→`0xAC++`; `>`→`0xAC--`.
- else (L held, bit0=0): `0x388 &= 2`; if `0xAC < tbl_C0CA[Y]` (`[0x0A,0x0B]`) → `0xAC++`, return; else
  WHEELIE: if `(0x5C & 0xC0)==0` (no A/B) → ret; if `airMode!=0` → ret; `0xAC++`; `0x26=0x0D`; if
  `0xAC >= 0x0D` → crash `0x98=1`, `0x26=0x1A`.

**Rest-angle target 0x368 (sub_DCA0, X=player):** `Y = 0xD4`; if `track_finished==1 &&
(0x98|0x9C|0x58)==0` → `0x368 = 0x0A`; else `0x368 = tbl_D88B[Y]` (`tbl_D88B=[06,03,04,02,0B,08,09]`);
then if `0xA4==1` → `0x368 = 6`. Terrain features (E7A3) also set `0xAC` directly to the track nibble.
`0xAC` feeds the perfect-vs-wobble landing decision (§6.3) → affects landing columns and post-landing
speed, so it is reward-relevant via landings.

## 7. Vertical / lane (sub_E96C + input at $EA07/$EA2B) — model known, port pending

Player velZ `0xDC` set by `0x14 & 0x0C` (Up→+1, Down→0xFF) when on the ground (gated by race-started,
`!crash`, `0x9C` in {0,5}); persists until `0xB8` hits a boundary in `tbl_E53D = [14,26,38,50]` → velZ=0.
Each frame `0xB8 += velZ` (skip if section 0x14/0x15). Lane = sub_D0AB(`0xB8`):
`a=0xB8-0x10; if a<0 → 5; Y=5; loop{Y--; if Y==0 → 0; a-=8; if a<0 → Y}` (14/26/38/50 → lanes 5/3/2/0).
Clamps at `0xB8<8` and `0xB8>=0x3A`. Visual `0x8C` (ground) = `~0xA0 - 0xB8` (via sub_DC97).

## 8. Wheelie — not yet traced (Up/Down on ground → velZ/posZ/angle; over-wheelie crash).

## 8b. Standalone engine + differential testing (tools/engine.py, tools/difftest.py)
A full standalone Python engine (`tools/engine.py`) runs PURELY from inputs (seeded at frame 0), with
no dump-peeking — the reference for the C++ port. It combines every routine: column counter, lane
(sub_E96C incl. the edge clamp + velZ flip `0xDC^0xFE`), section machinery + handlers + sub_E927,
angle, speed (sub_CD59), temperature, posX (sub_DA58/DBFE), takeoff (sub_DD38), arc (sub_DD6F),
landing/bounce (sub_DC1A), the climb handlers (sub_E84F/E86F: `0xBC`), the terminator's `0xBC=0`, and
the mud handler 0x0C (sub_CE5C friction). Key timing facts discovered: (a) lane index `0x360` is
computed in render and LAGS `0xB8` by one frame (compute at frame START from pre-update 0xB8); (b) the
angle/section routines see the PRE-landing airMode (landing is late in sub_DA26); (c) the bounce
re-launch goes through loc_DCFA so `0x384=0` (not velX_hi).

RESULTS:
- **tas.ram: BYTE-EXACT** standalone on every reward-critical address (posX, velX, sections, angle,
  slope, airMode, lane, posY, velZ, 0x8C, column) for all 2263 frames. (Only `0x52` race-over flag in
  the final 2 frames is unmodeled — the loop-completion win trigger.)
- **manual.ram: reward-exact through f2281** (posX/velX/sections), after which it hits terrain the TAS
  never uses (a takeoff variant). Earlier divergence is a 1-frame cosmetic 0x8C blip at a BC underflow.

Found + fixed a real bug present in the committed `engine.hpp`: friction with `velX_hi==0` must CLAMP
velX_lo to 0 (sub_CE58 `bra_CE80`), not wrap to 256−n. Never hit by the always-moving TAS; exposed by
coast-to-stop in the manual ref / alt movies.

**Differential testing** (`tools/difftest.py` runs a `.sol` through the emulator `jaffar-player` AND
the engine, diffing every frame; `tools/gen_alts.py` writes suboptimal movies):
- `race01.sol` (TAS): byte-exact except `0x52` final 2 frames.
- `alt_coast` (B-pulse + coast): **BYTE-EXACT, 1401 frames** — throttle/friction/coast fully correct.
- `alt_coast` (B-pulse + coast): **BYTE-EXACT, 1401 frames** — requires the type-0x12 cooling-zone
  (E8D3: on-ground & !stalled → `0x3B6=8`); without it, temp climbs unbounded and spuriously stalls.

The OVERHEAT STALL is now MODELED (and TAS stays byte-exact incl temp `0x3B6` + stall `0x3C`):
  - `sub_D310` full timer routine: `0x20` counter gates the slow timers `0x30-0x3D` (so stall timer
    `0x3C` decrements every 11 frames); fast `0x21-0x2F` (incl angle timers 0x26/0x2A) every frame.
  - `loc_E3AA` overheat arm (late): `temp(0x3B6) >= 0x20` → `0x3C = 0x3E0 = temp`.
  - `sub_DD8D` (early): while `0x3C>8` and bike idle → force velX_lo (0, or 0xC0 in section) + drift
    velZ; at `0x3C==8` → recovery (`temp=5, 0x9C=5, 0x3E0=0, 0x374=5`).
  - `sub_CD1F` input lock: `0x3E0!=0 && 0x9C!=5` → zero throttle `0x5C` before the speed routine.
  mashB now matches the FULL stall (was f422, now f699). Remaining mashB/airlean divergence at f699 is
  the `0x9C` start/recovery state machine (sub_D924 state 5) re-centering the lane (velZ) on recovery exit.

Still UNMODELED long-tail (each TAS-irrelevant — a posX-optimal search never stalls/crashes):
  - `0x9C` start/recovery state machine (sub_D924) — lane re-center after stall (mashB/airlean f699).
  - crash trigger on a bad A-throttle landing (alt_throttleA f442) + crash animation/recovery.
  - more terrain coverage under heavy steering / random play (alt_lanes f251, alt_random f113).
  - the `0x52` loop-completion win flag (race-over; TAS byte-exact except its final 2 frames).

## 9. Status & port plan
DONE (byte-exact, both dumps, all non-crash frames): column counter (§3), 0xC0/streaming (§4.1), section
machinery 0x58/0xC4/0xC8 (§4), terrain handlers + sub_E927 held-effect (§5/§6.4), angle 0xAC + slope 0xD4
(§6.4), takeoff timing (16/16 TAS, 27/27 manual). Verifiers: tools/sim.py, tools/stream_check.py.
Only residuals: 41 manual 0xAC frames inside one crash (pruned in a posX search) — see §6.4.

REMAINING: airborne ARC + landing/bounce (§6) and lane movement (§7) are traced & in the Python sims
(tools/sim_arc.py climb byte-exact) but not yet wired into the C++ engine; wheelie (§8) not traced;
crash animation/recovery intentionally unmodeled.

Port plan:
0. DONE: `source/engine.hpp` is the full C++ port of `tools/engine.py` — byte-exact reward vs tas.ram
   (final posX 12157.328; only 0x52 win-flag differs in the last 2 frames) and byte-exact vs the emulator
   on alt_coast. Build: `meson setup build && ninja -C build`; validate via `exciteBike-player ...
   --initialRam tas.ram --dumpRam native.ram`. Track data = `track_layout.hpp` + `track_sections.hpp`
   (generated/gitignored: tools/extract_track_layout.py + tools/gen_sections_header.py). Dead TAS-path
   `track_data.hpp` removed; the friction clamp bug is fixed.
1. DONE in `tools/engine.py` (standalone, byte-exact on TAS), `0xC0 = kTrack[lane_i][cum_(i-1)]`.
2. WIN FLAG MODELED: race01 is a 3-lap track; lap counter `0x3A4`/`0x57` computed from cumColumn
   (boundaries at cum 668/1335, the streaming lap-advance); the type-0x08 finish-line feature (section 3,
   pos 48) sets `0x52=1` when `0x57==0`. `isWin()` flips at frame 2261, byte-exact. Remaining long-tail
   mechanics (§8b): the `0x9C` start/recovery state machine and crash animation — neither on a posX-optimal
   (non-crashing, non-stalling) path. (overheat stall, mud, cooling, win all modeled.)
3. `python3 tools/difftest.py <movie.sol>` cross-checks engine vs emulator on any movie (the fidelity
   gate generalized beyond the two references).
