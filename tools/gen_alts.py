#!/usr/bin/env python3
"""Generate alternative (suboptimal) .sol movies to differential-test the engine vs the emulator.
Inputs are valid jaffar joypad strings "|..|UDLRSsBA|". We craft varied-but-plausible play that
exercises mechanics the two reference movies don't: different throttle, lane changes, air leans,
coasting. Output to scratchpad; then tools/difftest.py runs each through emulator + engine.

Usage: python3 tools/gen_alts.py   (writes alt_*.sol to scratchpad)
"""
import os
SCRATCH = "/tmp/claude-1000/-home-jaffar-exciteBot/3c7e3fc0-5396-41c4-bced-5455f3b15d85/scratchpad"

def inp(U=0,D=0,L=0,R=0,S=0,s=0,B=0,A=0):
    return '|..|%s%s%s%s%s%s%s%s|'%('U' if U else '.','D' if D else '.','L' if L else '.',
        'R' if R else '.','S' if S else '.','s' if s else '.','B' if B else '.','A' if A else '.')

N = 1400
movies = {}

# 1) mash B the whole way (pure throttle, no steering)
movies['alt_mashB'] = [inp(B=1) for _ in range(N)]

# 2) A throttle instead of B (lower cap)
movies['alt_throttleA'] = [inp(A=1) for _ in range(N)]

# 3) B + periodic lane changes (Up for 6 frames every 80, Down for 6 every 80 offset)
m=[]
for i in range(N):
    U = 1 if (i % 80) < 6 else 0
    D = 1 if (i % 80) >= 40 and (i % 80) < 46 else 0
    m.append(inp(B=1,U=U,D=D))
movies['alt_lanes'] = m

# 4) B + alternate air-lean L/R (every other frame R then L) -- exercises arc lever & air friction
m=[]
for i in range(N):
    R = 1 if (i % 20) < 10 else 0
    L = 1 if (i % 20) >= 10 else 0
    m.append(inp(B=1, R=R, L=L))
movies['alt_airlean'] = m

# 5) coast pulses: B for 20 frames, coast for 10 -- triggers friction-to-low-speed paths
m=[]
for i in range(N):
    B = 1 if (i % 30) < 20 else 0
    m.append(inp(B=B))
movies['alt_coast'] = m

# 6) deterministic pseudo-random valid inputs (LCG, no Math.random) -- broad coverage
m=[]; seed=0x1234
for i in range(N):
    seed = (seed*1103515245 + 12345) & 0x7fffffff
    r = seed >> 8
    B = (r & 1); A = (r>>1)&1 if not B else 0
    U = (r>>2)&1; D = (r>>3)&1 if not U else 0
    L = (r>>4)&1; R = (r>>5)&1 if not L else 0
    m.append(inp(U=U,D=D,L=L,R=R,B=B,A=A))
movies['alt_random'] = m

for name, lines in movies.items():
    p = os.path.join(SCRATCH, name+'.sol')
    open(p,'w').write('\n'.join(lines)+'\n')
    print('wrote', p, len(lines), 'frames')
