#!/usr/bin/env python3
"""Validate the airborne ARC (sub_DD6F) + LANDING (sub_DC1A) against a ground-truth dump.

We model the height 0x8C and the landing transition (airMode 0xB0 2->0). To isolate the arc, we
drive the inputs it consumes from the dump: velX (0x90/0x94 -> launch velZ at takeoff), lean
(0x5C&3 -> 0x38C term), the game cycle 0x4C, climbRemaining 0xBC, lane pos 0xB8, launch-type 0x388.
We re-seed at each takeoff (airMode 0->2) from the dump's launch state, then integrate the arc
ourselves and check 0x8C each frame and the landing frame.

Order per object in sub_DA26: sub_DA58 (posX) -> sub_DC1A (landing) -> sub_DCF2 (arc) -> sub_DCDE.
Usage: python3 tools/sim_arc.py [tas|manual]
"""
import sys, os
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
F = 2048
which = sys.argv[1] if len(sys.argv) > 1 else 'tas'
RAM = {'tas': 'tas.ram', 'manual': 'manual.ram'}[which]
d = open(RAM, 'rb').read(); FR = [d[i*F:(i+1)*F] for i in range(len(d)//F)]
TBL_D868 = [52, 52, 24, 52]

def b(x): return x & 0xFF

# modeled airborne state
airMode = FR[0][0xB0]
A380 = FR[0][0x380]; A384 = FR[0][0x384]; A38C = FR[0][0x38C]
A378 = FR[0][0x378]; A37C = FR[0][0x37C]; A364 = FR[0][0x364]
Y8C = FR[0][0x8C]

m8C = 0; first8C = None
land_model = []; land_dump = []
prevB0_dump = FR[0][0xB0]
takeoff_dump = []

for i in range(1, len(FR)):
    f = FR[i]; pf = FR[i-1]
    # --- detect takeoff in dump (airMode 0->2): re-seed our arc from the dump's post-takeoff state ---
    if pf[0xB0] == 0 and f[0xB0] == 2:
        takeoff_dump.append(i)
        airMode = 2
        # sub_DD38 already ran in the dump this frame; seed from dump's launch accumulators
        A380 = f[0x380]; A384 = f[0x384]; A38C = f[0x38C]
        A378 = f[0x378]; A37C = f[0x37C]; A364 = f[0x364]
        Y8C = f[0x8C]
        continue  # the takeoff frame's 0x8C is set by the takeoff path; start integrating next frame

    if airMode == 2:
        # sub_DC1A (landing) runs FIRST: groundY = 0x3F1 - 0xBC - 0xB8 (0x3F1 const 0xA0)
        groundY = b(0xA0 - f[0xBC] - f[0xB8])
        landed = False
        if not (groundY >= Y8C) and not (Y8C >= 0xA8) and A364 != 0:
            landed = True
            airMode = 0
            land_model.append(i)
            Y8C = b(groundY - 1)
        else:
            # sub_DCF2 arc: gate -- if (0x5C bit0 set) and 0x4C==0 -> skip arc this frame
            lean_bit0 = f[0x5C] & 1
            skip = (lean_bit0 == 1 and f[0x4C] == 0)
            if not skip:
                A38C = TBL_D868[f[0x5C] & 3]
                # sub_DD6F carry chain. Cin = lean bit0 (the LSR in the sub_DCF2 gate sets C=bit0).
                cin = f[0x5C] & 1
                t = A380 + A38C + cin                 # LDA 0x380; ADC 0x38C
                A380 = b(t); c1 = 1 if t > 0xFF else 0
                t2 = A384 + c1                         # LDA 0x384; ADC #0
                A384 = b(t2); c2 = 1 if t2 > 0xFF else 0
                sub = Y8C - A37C - (1 - c2)            # LDA prev8C; SBC 0x37C  (borrow-in = 1-c2)
                c3 = 1 if sub >= 0 else 0
                new8C = b(sub) + A384 + c3             # LDA; ADC 0x384  (+ carry c3)
                Y8C = b(new8C)
                if f[0xCC] == 0: A364 = 1
        # compare height
        if Y8C != f[0x8C]:
            m8C += 1
            if first8C is None: first8C = (i, Y8C, f[0x8C])
    # track dump landings
    if pf[0xB0] == 2 and f[0xB0] != 2: land_dump.append(i)
    # resync airMode to dump (so we test arc per-jump, not error cascades across jumps)
    airMode = f[0xB0]

print(f"=== {RAM} arc/landing ===")
print(f"  0x8C (height) mismatches while airborne: {m8C}  first={first8C}")
print(f"  takeoffs(dump)={len(takeoff_dump)} landings model={len(land_model)} dump={len(land_dump)}")
print(f"  model landing frames first 8: {land_model[:8]}")
print(f"  dump  landing frames first 8: {land_dump[:8]}")
