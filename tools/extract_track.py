#!/usr/bin/env python3
"""
Extract the on-path track profile of the reference race from tas.ram (handoff §8, path 2).

Because the optimal route is geometry-forced (no lane slack -- proven), the terrain along the path is a
deterministic function of the bike's absolute X. This tool walks tas.ram, reconstructs absolute posX
(blockTransitions*256 + 0x50 + 0x394/256), and segments the race into contiguous runs of
(airMode, slope) -- i.e. flat-ground / ramp / airborne / downhill stretches -- recording for each the
frame & posX span and the velX behaviour. Output is a human-readable segment map (the track skeleton
the native engine must follow) and the jump launch/land pairs.

Usage: python3 tools/extract_track.py [path/to/tas.ram]   (default: ./tas.ram)
"""
import sys
F = 2048
path = sys.argv[1] if len(sys.argv) > 1 else 'tas.ram'
T = open(path, 'rb').read()
NF = len(T) // F
def fr(f): return T[f*F:(f+1)*F]

# Reconstruct absolute posX exactly as the game's _bikePosX does (count 0x4E toggles).
blk = 0; prev4E = fr(0)[0x4E]
absx = []
for f in range(NF):
    b = fr(f)
    if b[0x4E] != prev4E: blk += 1
    prev4E = b[0x4E]
    absx.append(blk*256 + b[0x50] + b[0x394]/256.0)

# Segment by (airMode!=0, slope).
def key(b): return (1 if b[0xB0] != 0 else 0, b[0xD4])
segs = []
s = 0
for f in range(1, NF+1):
    if f == NF or key(fr(f)) != key(fr(s)):
        b0 = fr(s); b1 = fr(f-1)
        v0 = b0[0x94]*256+b0[0x90]; v1 = b1[0x94]*256+b1[0x90]
        segs.append((s, f-1, 'AIR' if b0[0xB0] else 'GND', b0[0xD4], absx[s], absx[f-1], v0, v1, b0[0xAC], b1[0xAC]))
        s = f

print(f"# Track profile of the reference race ({NF} frames, finishes at the Race Over flag)\n")
print(f"{'frames':>11}  {'kind':3} {'slope':>5}  {'posX_start':>10} {'posX_end':>10}  {'velX':>9}  {'angle':>7}")
for (s,e,k,sl,x0,x1,v0,v1,a0,a1) in segs:
    print(f"{s:5d}-{e:5d}  {k:3} {sl:5d}  {x0:10.2f} {x1:10.2f}  {v0:4d}->{v1:4d}  {a0:3d}->{a1:3d}")

# Jump launch/land pairs (ground->air->ground)
print(f"\n# Jumps (airborne stretches): {sum(1 for x in segs if x[2]=='AIR')}")
for (s,e,k,sl,x0,x1,v0,v1,a0,a1) in segs:
    if k == 'AIR':
        print(f"  launch frame {s:5d} @posX {x0:8.2f}  -> land frame {e+1:5d} @posX {x1:8.2f}  ({e-s+1} frames aloft, velX {v0})")

# Downhill (coast) over-cap segments: ground stretches where velX exceeds the 832 flat cap.
print("\n# Over-cap (downhill) ground stretches (velX > 832):")
for (s,e,k,sl,x0,x1,v0,v1,a0,a1) in segs:
    if k == 'GND' and max(v0,v1) > 832:
        print(f"  frames {s:5d}-{e:5d} @posX {x0:8.2f}-{x1:8.2f}  slope {sl}  velX {v0}->{v1}")
