#!/usr/bin/env python3
"""Reference simulator (Python) for the Excitebike section machinery + terrain handlers.

Validates the ported track logic against a ground-truth RAM dump WITHOUT yet modelling the lane
or arc: it drives the per-frame column advance (0x60) and lane (0x360) from the dump so we can
isolate and verify (a) the column counter (0x64/0xE0), (b) the section selection + in-section
machinery (sub_E73B: 0x58/0xC4/0xC8), and (c) the ground-angle / ramp / type-0x13 / crash handlers
(angle 0xAC, slope 0xD4, takeoff airMode 0xB0).

Order per frame mirrors sub_C99A: input latch, then sub_E733 (section machinery, which dispatches
the terrain handlers incl. the ramp takeoff). We compare our 0x58/0xC4/0xC8/0xAC/0xD4 + the takeoff
frame (0xB0 0->2) against the dump.

Usage: python3 tools/sim.py [tas|manual]
"""
import sys, os, json, re
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
F = 2048
which = sys.argv[1] if len(sys.argv) > 1 else 'tas'
RAM = {'tas': 'tas.ram', 'manual': 'manual.ram'}[which]

# --- load tables ---
def load_track():
    txt = open('source/track_layout.hpp').read()
    rows = re.findall(r'\{((?:0x[0-9A-F]{2},?)+)\}', txt)
    return [[int(x, 16) for x in r.rstrip(',').split(',')] for r in rows]
TRACK = load_track()  # TRACK[lane][col]
def parse_sections():
    secs = []
    for line in open('docs/track_sections.txt'):
        m = re.match(r'\s*sec\s+(\d+)\s+@\S+\s+\(\d+ feats\):\s*(.*)', line)
        if not m: continue
        feats = [(int(p), int(t, 16)) for p, t in (tok.split(':') for tok in m.group(2).split())]
        secs.append(feats)
    return secs
SECTIONS = parse_sections()  # SECTIONS[blockidx] = [(pos,type),...]
TBL_E6AD = [3,1,2,2,0,5,5,6,4,4]  # slope = tbl_E6AD[angle-2]
TBL_D88B = [6,3,4,2,0x0B,8,9]     # rest-angle 0x368 = tbl_D88B[slope 0xD4]
TBL_C0D4 = [6,4]                  # tilt timer 0x26 reload  [ground, air]
TBL_C0C8 = [6,2]                  # tilt target (R held)    [ground, air]
TBL_C0CA = [0x0A,0x0B]            # tilt target (L held)    [ground, air]

def load_dump():
    d = open(RAM, 'rb').read()
    return [d[i*F:(i+1)*F] for i in range(len(d)//F)]
FR = load_dump()

s = {}
f0 = FR[0]
for a in (0x4C,0x58,0x60,0x64,0xE0,0xC0,0xC4,0xC8,0xAC,0xD4,0xB0,0x90,0x94,0x98,0xCC,0xA4,0xA0,0xD8,0x388,
          0x368,0x2A,0x26,0x52,0x9C,0x5C):
    s[a] = f0[a]
cumcol = 0          # END-of-frame unwrapped column (matches dump 0xE0)
prevcol = 0         # column as seen by sub_E733 (before this frame's advance)
prevE0 = f0[0xE0]
prev60 = f0[0x60]
prev64 = f0[0x64]
prevlane = f0[0x360]

def section_machinery(lane, prev60, prev64, dcol):
    # sub_E73B (player). C8 += prev-frame 0x60 (runs before this frame's position update sets 0x60).
    s_attr_set(0xC8, (s.get(0xC8,0) + prev60) & 0xFF)
    if s.get(0x58) == 0:
        c0 = TRACK[lane][dcol] if dcol < len(TRACK[lane]) else 0x3B
        t = c0 - 0x40
        if t < 0: return
        t >>= 2
        if t >= 0x16: return
        s_attr_set(0x58, (t + 1) & 0xFF)
        s_attr_set(0xC4, 0)
        s_attr_set(0xC8, (prev64 - 1) & 0xFF)
    # scan features
    while True:
        blk = s.get(0x58) - 1
        feats = SECTIONS[blk] if 0 <= blk < len(SECTIONS) else []
        cur = s.get(0xC4) // 2
        if cur >= len(feats):  # 0xFF terminator (end of section)
            s_attr_set(0x58, 0); s_attr_set(0xD4, 0)
            return
        pos, typ = feats[cur]
        if pos > s.get(0xC8):
            return
        # trigger feature
        if typ & 0x80:        # ground angle handler E7A3
            handler_angle(typ)
        elif typ & 0x40:      # E7F2
            s_attr_set(0xCC, typ & 0x0F)
        else:
            dispatch(typ)
        s_attr_set(0xC4, (s.get(0xC4) + 2) & 0xFF)

def handler_angle(typ):
    # E7A3: if airMode|crash != 0 -> skip; if 0xA4==1 -> skip; AC=nibble; if 0x58!=3: D4=tbl[AC-2]
    if s.get(0xB0) or s.get(0x98): return
    nib = typ & 0x0F
    if s.get(0xA4) == 1: return
    s_attr_set(0xAC, nib)
    if s.get(0x58) != 3:
        idx = nib - 2
        if 0 <= idx < len(TBL_E6AD): s_attr_set(0xD4, TBL_E6AD[idx])

def e893(a):  # sub_E893: D4 = B4 = A (slope/visual). Runs unconditionally (no airMode gate).
    s_attr_set(0xD4, a); s_attr_set(0xB4, a)

def dispatch(typ):
    if typ == 0x06: ramp()
    elif typ == 0x13: type13()
    elif typ == 0x00:  # E963 clear
        s_attr_set(0xCC,0); s_attr_set(0xD4,0); s_attr_set(0xB4,0)
    elif typ == 0x07: crash07()
    elif typ == 0x02: e893(6)              # E85D
    elif typ == 0x03: e893(1)              # E86A
    elif typ == 0x04: e893(5)              # E845 (also D8=0x80, not tracked)
    elif typ == 0x05: e893(2)              # E854
    elif typ == 0x09: e893(4)              # E879
    elif typ == 0x0A:                      # E89D (gated: skip if 0xA4==1)
        if s.get(0xA4) != 1: e893(3)
    # 0x01,0x08,0x0B,0x0C,0x0E,0x0F,0x10,0x12,0x14: no 0xAC/0xD4 effect
def ramp():  # E934 type06 -> takeoff
    if s.get(0xB0) != 0: return
    if s.get(0x94) == 0: return
    takeoff()
def type13():  # E8E7 -> INC 0x94 (+256) then takeoff
    if s.get(0x98): return
    s_attr_set(0x94, (s.get(0x94)+1)&0xFF)
    if s.get(0xB0) != 0: return
    takeoff()
def crash07():  # E818
    if s.get(0xB0): return
    if s.get(0xAC) >= 7: return
    vhi=s.get(0x94)
    if vhi>=3 or (vhi==2 and (s.get(0x90)&0x80)): s_attr_set(0x98,0xFF)
def takeoff():  # sub_DD38 (airMode=2 etc) -- minimal: set airMode 2
    s_attr_set(0xB0, 2)

def s_attr_set(a,v): s[a]=v

# --- angle 0xAC model (tail of sub_CD59) + rest-angle 0x368 (sub_DCA0) + ramp/tilt ---
def set_rest_angle():  # sub_DCA0 (player X=0), runs at frame START using current 0xD4
    y = s.get(0xD4)
    if s.get(0x52) == 1 and (s.get(0x98) | s.get(0x9C) | s.get(0x58)) == 0:
        s_attr_set(0x368, 0x0A)
    else:
        s_attr_set(0x368, TBL_D88B[y] if y < len(TBL_D88B) else 0)
    if s.get(0xA4) == 1:
        s_attr_set(0x368, 6)

def angle_update():  # loc_CDBD tail
    if s.get(0x98): return                      # crashed
    air = s.get(0xB0) != 0
    if air:
        active = True
    else:
        if (s.get(0x58) | s.get(0x52)) != 0:
            active = False
        elif s.get(0x94) != 0 or s.get(0x90) >= 0xA0:
            active = True
        else:
            active = False
    a000A = s.get(0x5C) & 0x03
    if active and a000A != 0:
        tilt(a000A)
    elif not air and s.get(0x368) != s.get(0xAC):
        ground_ramp()

def ground_ramp():  # loc_DCC7, target = 0x368, timer 0x2A reload 5
    if s.get(0x2A) != 0:
        return
    s_attr_set(0x2A, 5)
    ac = s.get(0xAC); tgt = s.get(0x368)
    if ac == tgt: return
    s_attr_set(0xAC, (ac + (1 if ac < tgt else -1)) & 0xFF)

def tilt(a000A):  # loc_CE83, timer 0x26 reload tbl_C0D4[Y]
    if s.get(0x26) != 0:
        return
    y = s.get(0xB0) >> 1
    s_attr_set(0x26, TBL_C0D4[y])
    ac = s.get(0xAC)
    if a000A & 1:                                # R held -> toward C0C8
        tgt = TBL_C0C8[y]
        if ac == tgt: return
        s_attr_set(0xAC, (ac + (1 if ac < tgt else -1)) & 0xFF)
    else:                                        # L held -> toward C0CA, then wheelie
        s_attr_set(0x388, s.get(0x388) & 2)
        if ac < TBL_C0CA[y]:
            s_attr_set(0xAC, ac + 1); return
        if (s.get(0x5C) & 0xC0) == 0: return
        if s.get(0xB0) != 0: return
        s_attr_set(0xAC, ac + 1); s_attr_set(0x26, 0x0D)
        if s.get(0xAC) >= 0x0D:
            s_attr_set(0x98, 1); s_attr_set(0x26, 0x1A)

# --- run ---
mismatch = {a:0 for a in (0x58,0xC4,0xC8,0xAC,0xD4)}
first = {}
takeoff_frames_model=[]; takeoff_frames_dump=[]
prevB0_model=s.get(0xB0); prevB0_dump=FR[0][0xB0]
ac_crash_split=[0,0]  # [during crash 0x98!=0, not crashed]
ac_noncrash_first=[]
for i in range(1, len(FR)):
    f = FR[i]
    pf = FR[i-1]
    # drive inputs / non-modeled state from the dump (we model 0xAC,0xD4,0x368,0x2A,0x26 only).
    # airMode 0xB0: the section handlers (E733) + angle routine (CD59) run BEFORE this frame's
    # landing (sub_DA26, late), so they see the START-of-frame airMode = end of PREVIOUS frame.
    # takeoff() may raise it to 2 mid-E733. (The dump's 0xB0 is the post-landing end-of-frame value.)
    s_attr_set(0x5C, f[0x5C]); s_attr_set(0x52, f[0x52]); s_attr_set(0x9C, f[0x9C])
    s_attr_set(0xB0, pf[0xB0]); s_attr_set(0x98, f[0x98]); s_attr_set(0x94, f[0x94])
    s_attr_set(0x90, f[0x90]); s_attr_set(0xA4, f[0xA4])
    # (NMI) sub_D310: timers 0x21-0x2F decrement every frame if nonzero
    for tmr in (0x26, 0x2A):
        if s.get(tmr): s_attr_set(tmr, s.get(tmr) - 1)
    # sub_CA08 -> sub_DCA0 (frame START): rest-angle 0x368 from the PREVIOUS frame's 0xD4
    set_rest_angle()
    # sub_E733 runs EARLY (before this frame's position update): uses prev-frame 0x60/0x64.
    # 0xC0 = kTrack[lane_i][cum_(i-1)]: lane updates EARLY (sub_E96C) so it's THIS frame's lane,
    # but E0/cum update LATE (sub_E70B) so the column is the PREVIOUS frame's. Verified 0-exact.
    section_machinery(f[0x360], prev60, prev64, cumcol)
    # sub_E927 (C9AE, right after E733): re-dispatch the HELD terrain effect each frame.
    # 0xCC (set by E7F2 features) persists; while nonzero, handler tbl_E6B7[0xCC] re-fires every
    # frame (e.g. CC=3 -> handler 0x03 -> sub_E893 -> 0xD4=0xB4=1). This is the dominant 0xD4 source.
    if s.get(0xCC): dispatch(s.get(0xCC))
    # sub_CD1F->CD59 tail: lean/tilt/ground-ramp angle update
    angle_update()
    # then advance column counter (sub_E70B at C9FC, end of frame) from this frame's 0x60 (verified)
    v = f[0x60]
    if v != 0:
        bd = (s.get(0x64) - v) & 0xFF
        if bd == 0 or (bd & 0x80):
            s_attr_set(0x64, (bd + 8) & 0xFF)
            s_attr_set(0xE0, (s.get(0xE0)+1)&0x3F)
        else:
            s_attr_set(0x64, bd)
    cumcol += (f[0xE0]-prevE0)&0x3F; prevE0=f[0xE0]
    prev60 = f[0x60]; prev64 = f[0x64]; prevlane = f[0x360]
    # compare
    for a in mismatch:
        if s.get(a) != f[a]:
            mismatch[a]+=1
            first.setdefault(a,(i,s.get(a),f[a]))
            if a == 0xAC:
                ac_crash_split[0 if f[0x98] else 1] += 1
                if not f[0x98] and not ac_noncrash_first:
                    ac_noncrash_first.append((i, s.get(0xAC), f[0xAC], f[0xB0], f[0x5C], f[0x58]))
    # track takeoff events
    mb0=s.get(0xB0);
    if prevB0_model==0 and mb0==2: takeoff_frames_model.append(i)
    if prevB0_dump==0 and f[0xB0]==2: takeoff_frames_dump.append(i)
    prevB0_model=mb0; prevB0_dump=f[0xB0]

print(f"=== {RAM} ({len(FR)} frames) ===")
for a in mismatch:
    print(f"  0x{a:03X}: mism={mismatch[a]:5d}  first={first.get(a)}")
print(f"  0xAC mismatches: during-crash(0x98!=0)={ac_crash_split[0]}  not-crashed={ac_crash_split[1]}")
print(f"  first non-crash AC mism (frame,model,dump,airM,5C,58): {ac_noncrash_first}")
print(f"  takeoffs model={len(takeoff_frames_model)} dump={len(takeoff_frames_dump)}")
print(f"   model first 8: {takeoff_frames_model[:8]}")
print(f"   dump  first 8: {takeoff_frames_dump[:8]}")
