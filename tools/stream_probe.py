#!/usr/bin/env python3
"""Empirically characterize the terrain streaming into ram_0400..0x57F (6 lanes x 64 bytes).

For each frame we diff the 6 lane buffers against the previous frame and report which slots
changed. We correlate changed slot index with E0 and cumColumn to learn the streaming phase:
when cumColumn increments, which (E0+K) slot is written with terrain for which absolute column.

Usage: python3 tools/stream_probe.py [tas|manual]
"""
import sys, os
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
F = 2048
which = sys.argv[1] if len(sys.argv) > 1 else 'tas'
RAM = {'tas': 'tas.ram', 'manual': 'manual.ram'}[which]
d = open(RAM, 'rb').read(); FR = [d[i*F:(i+1)*F] for i in range(len(d)//F)]
LANE_BASE = [0x400, 0x440, 0x480, 0x4C0, 0x500, 0x540]

def buf(f, lane): return f[LANE_BASE[lane]:LANE_BASE[lane]+0x40]

# track cumColumn via dump E0 (unwrap)
prevE0 = FR[0][0xE0]; cum = 0
changes = []  # (frame, lane, slot, old, new, E0, cum, slot_minus_E0)
for i in range(1, min(len(FR), 600)):
    f = FR[i]; pf = FR[i-1]
    cum += (f[0xE0]-prevE0) & 0x3F; prevE0 = f[0xE0]
    E0 = f[0xE0]
    for lane in range(6):
        b0, b1 = buf(pf, lane), buf(f, lane)
        for slot in range(0x40):
            if b0[slot] != b1[slot]:
                changes.append((i, lane, slot, b0[slot], b1[slot], E0, cum, (slot-E0) & 0x3F))

print(f"=== {RAM}: {len(changes)} buffer-slot changes in first {min(len(FR),600)} frames ===")
# distribution of (slot - E0) mod 64 -- the streaming lookahead distance
from collections import Counter
dist = Counter(c[7] for c in changes)
print("slot-E0 (lookahead) distribution:", dict(sorted(dist.items())))
# show a window of changes around lane transitions
print("\nfirst 40 changes (frame, lane, slot, old->new, E0, cum, slot-E0):")
for c in changes[:40]:
    print(f"  f{c[0]:4d} L{c[1]} slot{c[2]:2d} {c[3]:02X}->{c[4]:02X} E0={c[5]:2d} cum={c[6]:4d} ahead={c[7]:2d}")
