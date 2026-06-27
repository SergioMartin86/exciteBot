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

def load_dump():
    d = open(RAM, 'rb').read()
    return [d[i*F:(i+1)*F] for i in range(len(d)//F)]
FR = load_dump()

s = {}
f0 = FR[0]
for a in (0x4C,0x58,0x60,0x64,0xE0,0xC0,0xC4,0xC8,0xAC,0xD4,0xB0,0x90,0x94,0x98,0xCC,0xA4,0xA0,0xD8,0x388):
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

def dispatch(typ):
    if typ == 0x06: ramp()
    elif typ == 0x13: type13()
    elif typ == 0x00:  # E963 clear
        s_attr_set(0xCC,0); s_attr_set(0xD4,0)
    elif typ == 0x07: crash07()
    # other types (08,0B,0C,0F,10,12,14,0D,0E,01..05,0A): not yet modeled
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

# --- run ---
mismatch = {a:0 for a in (0x58,0xC4,0xC8,0xAC,0xD4)}
first = {}
takeoff_frames_model=[]; takeoff_frames_dump=[]
prevB0_model=s.get(0xB0); prevB0_dump=FR[0][0xB0]
for i in range(1, len(FR)):
    f = FR[i]
    # sub_E733 runs EARLY (before this frame's position update): uses prev-frame 0x60/0x64, the
    # column/lane as of end of last frame (0xC0 was set by last frame's render).
    section_machinery(prevlane, prev60, prev64, cumcol)
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
    # track takeoff events
    mb0=s.get(0xB0);
    if prevB0_model==0 and mb0==2: takeoff_frames_model.append(i)
    if prevB0_dump==0 and f[0xB0]==2: takeoff_frames_dump.append(i)
    prevB0_model=mb0; prevB0_dump=f[0xB0]
    # resync airMode from dump each frame (landing not modeled here) to isolate section logic
    s_attr_set(0xB0, f[0xB0]); s_attr_set(0x98, f[0x98]); s_attr_set(0x94,f[0x94]); s_attr_set(0x90,f[0x90])
    s_attr_set(0xA4, f[0xA4])

print(f"=== {RAM} ({len(FR)} frames) ===")
for a in mismatch:
    print(f"  0x{a:03X}: mism={mismatch[a]:5d}  first={first.get(a)}")
print(f"  takeoffs model={len(takeoff_frames_model)} dump={len(takeoff_frames_dump)}")
print(f"   model first 8: {takeoff_frames_model[:8]}")
print(f"   dump  first 8: {takeoff_frames_dump[:8]}")
