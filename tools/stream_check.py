#!/usr/bin/env python3
"""Verify the streaming MODEL reproduces 0xC0 byte-exact with the discovered indexing:
  0xC0(i) = buffer[lane_i][E0_{i-1}]
where buffer is a per-lane 64-wide circular buffer, streamed by writing slot (E0+42)&63 =
master[lane][cum+42] on each cumColumn increment (seeded from frame 0's actual buffers).

Also test the closed form 0xC0(i) == master[lane_i][cum_{i-1}] (steady-state equivalent).

Usage: python3 tools/stream_check.py [tas|manual]
"""
import sys, os, re
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
F = 2048
which = sys.argv[1] if len(sys.argv) > 1 else 'tas'
RAM = {'tas': 'tas.ram', 'manual': 'manual.ram'}[which]
d = open(RAM, 'rb').read(); FR = [d[i*F:(i+1)*F] for i in range(len(d)//F)]
LANE_BASE = [0x400, 0x440, 0x480, 0x4C0, 0x500, 0x540]
AHEAD = 42

def load_track():
    txt = open('source/track_layout.hpp').read()
    rows = re.findall(r'\{((?:0x[0-9A-F]{2},?)+)\}', txt)
    return [[int(x, 16) for x in r.rstrip(',').split(',')] for r in rows]
TRACK = load_track()
def master(lane, col):
    return TRACK[lane][col] if 0 <= col < len(TRACK[lane]) else 0x3B

bufm = [list(FR[0][LANE_BASE[l]:LANE_BASE[l]+0x40]) for l in range(6)]
prevE0 = FR[0][0xE0]; cum = 0
prevCum = 0

n_model = 0; first_model = None
n_closed = 0; first_closed = None
for i in range(1, len(FR)):
    f = FR[i]
    E0_prev = prevE0       # E0 as of end of last frame (what the render samples)
    cum_prev = cum
    # apply streaming for each increment that occurred this frame (late, sub_E70B)
    inc = (f[0xE0] - prevE0) & 0x3F
    for k in range(inc):
        c = cum + k + 1
        slot = ((prevE0 + k + 1) + AHEAD) & 0x3F
        for lane in range(6):
            bufm[lane][slot] = master(lane, c + AHEAD)
    cum += inc; prevE0 = f[0xE0]
    lane = f[0x360]
    actual = f[0xC0]
    if bufm[lane][E0_prev] != actual:
        n_model += 1
        if first_model is None: first_model = (i, bufm[lane][E0_prev], actual, cum_prev, lane)
    if master(lane, cum_prev) != actual:
        n_closed += 1
        if first_closed is None: first_closed = (i, master(lane, cum_prev), actual, cum_prev, lane)

print(f"=== {RAM} ({len(FR)} frames) 0xC0 = f(lane_i, prev) ===")
print(f"  modeled circular buffer  buf[lane_i][E0_(i-1)] : mism={n_model:4d}  first={first_model}")
print(f"  closed form master[lane_i][cum_(i-1)]          : mism={n_closed:4d}  first={first_closed}")
