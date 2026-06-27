#!/usr/bin/env python3
"""Validate the native engine's REWARD (posX/velX) against a ground-truth RAM dump, frame by frame.

Builds the engine, replays a .sol through exciteBike-player seeded from the dump's frame 0, then reports
the first frame where any reward address diverges. Reward addresses are the ones that determine posX:
velX (0x90/0x94) + the posX accumulators (0x50/0x394/0x60/0x3BF/0x4E).

Usage:
  python3 tools/validate.py tas                      # TAS:    race01.jaffar / race01.sol / tas.ram
  python3 tools/validate.py manual                   # manual: race01.manual.jaffar / race01.manual.sol / manual.ram
  python3 tools/validate.py <jaffar> <sol> <ram>     # explicit
The .sol is auto-stripped of trailing blank lines (the parser aborts on an empty final line).
"""
import subprocess, sys, os, tempfile

os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
F = 2048
REWARD = {'scrollX':0x50,'posXsub':0x394,'velXlo':0x90,'velXhi':0x94,'scrollInc':0x60,'posXsubsub':0x3BF,'currBlockX':0x4E}

PRESETS = {
    'tas':    ('reference/race01.jaffar',        'reference/race01.sol',        'tas.ram'),
    'manual': ('reference/race01.manual.jaffar', 'reference/race01.manual.sol', 'manual.ram'),
}
a = sys.argv[1:]
if len(a) == 1 and a[0] in PRESETS: jaffar, sol, ram = PRESETS[a[0]]
elif len(a) == 3: jaffar, sol, ram = a
else: sys.exit(__doc__)

# strip trailing blank lines from the .sol
with open(sol) as f: lines = f.read().split('\n')
while lines and lines[-1].strip() == '': lines.pop()
clean = tempfile.NamedTemporaryFile('w', suffix='.sol', delete=False)
clean.write('\n'.join(lines)); clean.close()

r = subprocess.run(['ninja', '-C', 'build'], capture_output=True, text=True)
if r.returncode != 0: print(r.stdout[-2000:], r.stderr[-2000:]); sys.exit('BUILD FAILED')

nat = tempfile.NamedTemporaryFile(suffix='.ram', delete=False).name
r = subprocess.run(['./build/source/exciteBike-player', jaffar, clean.name, '--initialRam', ram, '--dumpRam', nat],
                   capture_output=True, text=True)
if r.returncode != 0: print(r.stderr); sys.exit('RUN FAILED')

N = open(nat, 'rb').read(); T = open(ram, 'rb').read(); NF = min(len(N), len(T)) // F
addrs = sorted(REWARD.items(), key=lambda x: x[1])
for fr in range(NF):
    nb, tb = N[fr*F:(fr+1)*F], T[fr*F:(fr+1)*F]
    diffs = [(nm, ad, nb[ad], tb[ad]) for nm, ad in addrs if nb[ad] != tb[ad]]
    if diffs:
        print(f"FIRST REWARD DIVERGENCE at frame {fr} of {NF}:")
        for nm, ad, nv, tv in diffs: print(f"  {nm:11s} @0x{ad:03X}: native={nv:3d} truth={tv:3d}")
        print(f"  velX16 native={nb[0x94]*256+nb[0x90]} truth={tb[0x94]*256+tb[0x90]}")
        sys.exit(0)
print(f"ALL {NF} FRAMES MATCH on reward addrs!")
