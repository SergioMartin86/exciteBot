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

## 4. Section machinery (sub_E73B) — 99.2% (OPEN: 0xC0 streaming timing)

Per frame (player): `0xC8 += prev0x60`. If `0x58==0`: read `c0` (= terrain marker, see §4.1); if
`c0>=0x40` and `((c0-0x40)>>2) < 22`: `0x58 = ((c0-0x40)>>2)+1; 0xC4=0; 0xC8 = prev0x64 - 1`. Then scan
features of block `0x58-1` (track_sections.txt "sec N" = block N; 0x58=N reads block N-1): for each
`(pos,type)` while `pos <= 0xC8`: dispatch (§5), `0xC4 += 2`; `0xFF` terminator (cursor past end) →
`0x58=0; 0xD4=0`.

### 4.1 OPEN — `0xC0` source / streaming timing
`0xC0` is set each frame by the renderer `sub_D0C6` = the terrain buffer `mem[0x400+lane*0x40+0xE0]`.
The buffer is overwritten by the streaming routine (`sub_ECE4`) AFTER the render samples it within a
frame, so near column/lane transitions `0xC0` lags the end-of-frame snapshot that `kTrack` was built
from. Using `kTrack[lane_{i-1}][cumColumn_{i-1}]` for section detection matches 35/36 entries; the
remaining ~19/2263 mismatches are frames where the snapshot caught a slot mid-recycle.

CHARACTERIZED: `0xC0` is NOT a static function of (lane, column). Keying each run's own per-frame
`0xC0` by (lane, column) predicts THAT run's section entries perfectly (manual 49/49; TAS 36/36 with
its own values), but the two runs CONFLICT at 404 cells — i.e. at a given track position the buffer
holds different bytes depending on how fast the bike got there. So the streaming has a velocity-/
timing-dependent PHASE (which 64-wide circular slot has been recycled to which far-ahead column). To
get `0xC0` byte-exact the streaming routine `sub_ECE4` (+ its scroll-driven trigger and the circular
buffer write order into `ram_0400+`) must be ported. Unlike the opponent RNG (cycle-timing-dependent,
unmodelable in a fast engine), the streaming is deterministic given the velocity profile, so it IS
modelable — this is the key remaining track task.

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
  not yet modeled (cosmetic/obstacle variants — port as needed).

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

### 6.4 OPEN — angle 0xAC
On the ground, `0xAC` ramps toward target `0x368` by ±1 (sub_DCC7, gated by timer `0x2A`); terrain
features (E7A3) also set `0xAC` directly. In the AIR the player tilts the bike (Up/Down → `0xAC`) — the
exact air-angle routine still needs locating. `0xAC` only feeds the perfect-vs-wobble landing decision
(reward-secondary, but needed for exact landing columns).

## 7. Vertical / lane (sub_E96C + input at $EA07/$EA2B) — model known, port pending

Player velZ `0xDC` set by `0x14 & 0x0C` (Up→+1, Down→0xFF) when on the ground (gated by race-started,
`!crash`, `0x9C` in {0,5}); persists until `0xB8` hits a boundary in `tbl_E53D = [14,26,38,50]` → velZ=0.
Each frame `0xB8 += velZ` (skip if section 0x14/0x15). Lane = sub_D0AB(`0xB8`):
`a=0xB8-0x10; if a<0 → 5; Y=5; loop{Y--; if Y==0 → 0; a-=8; if a<0 → Y}` (14/26/38/50 → lanes 5/3/2/0).
Clamps at `0xB8<8` and `0xB8>=0x3A`. Visual `0x8C` (ground) = `~0xA0 - 0xB8` (via sub_DC97).

## 8. Wheelie — not yet traced (Up/Down on ground → velZ/posZ/angle; over-wheelie crash).

## 9. Port plan
1. Add the column counter + lane + section machinery + handlers to `engine.hpp` (load `kTrack`).
2. Resolve §4.1 (0xC0 streaming) and §6.4 (angle) for byte-exactness.
3. Add the arc + landing/bounce; integrate with the existing speed/posX model.
4. `python3 tools/validate.py tas` and `... manual` must both match all frames.
