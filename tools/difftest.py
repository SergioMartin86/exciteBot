#!/usr/bin/env python3
"""Differential test: run a .sol movie through BOTH the emulator (jaffar-player / QuickerNES) and the
native engine (tools/engine.py), and report the first divergence + mismatch counts on reward-critical
RAM. This validates the engine on inputs the two reference movies don't cover (e.g. suboptimal play).

The engine is seeded from the emulator dump's frame 0 (post-boot) and then advanced PURELY from the
per-frame latched input (0x5C), with no further peeking. A clean run = the model reproduces the real
game byte-for-byte for that movie.

Usage: python3 tools/difftest.py <movie.sol> [label]
"""
import sys, os, subprocess, importlib.util
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROOT = os.getcwd()
SCRATCH = "/tmp/claude-1000/-home-jaffar-exciteBot/3c7e3fc0-5396-41c4-bced-5455f3b15d85/scratchpad"

spec = importlib.util.spec_from_file_location('engine', os.path.join(ROOT, 'tools/engine.py'))
eng_mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(eng_mod)
Engine = eng_mod.Engine; reverse8 = eng_mod.reverse8

F = 2048
WATCH = {0x50:'posXlo',0x394:'posXsub',0x94:'vXhi',0x90:'vXlo',0x58:'sect',0xAC:'angle',
         0xD4:'slope',0xB0:'air',0xB8:'posY',0x360:'lane',0x98:'crash',0x52:'raceOver'}

def run_emu(sol_path, out_ram):
    cmd = ['./jaffar-player','race01.jaffar', os.path.abspath(sol_path),
           '--reproduce','--unattended','--disableRender','--exitOnEnd','--dumpRam', out_ram]
    subprocess.run(cmd, cwd=os.path.join(ROOT,'reference'), capture_output=True)
    return open(out_ram,'rb').read()

def difftest(sol_path, label=None):
    label = label or os.path.basename(sol_path)
    out_ram = os.path.join(SCRATCH, 'emu_'+os.path.basename(sol_path)+'.ram')
    d = run_emu(sol_path, out_ram)
    FR = [d[i*F:(i+1)*F] for i in range(len(d)//F)]
    if len(FR) < 2:
        print(f"[{label}] emulator produced {len(FR)} frames -- aborting"); return
    eng = Engine(FR[0])
    mism = {a:0 for a in WATCH}; first = None
    posx_emu_final = None
    for i in range(1, len(FR)):
        eng.advance(reverse8(FR[i][0x5C]))
        for a in WATCH:
            if eng.m[a] != FR[i][a]:
                mism[a]+=1
                if first is None: first=(i,a,WATCH[a],eng.m[a],FR[i][a])
    tot = sum(mism.values())
    print(f"[{label}] {len(FR)} frames | total mismatches={tot}", end='')
    if tot==0: print("  -> BYTE-EXACT (engine == emulator)")
    else:
        print(f"\n   first divergence: f{first[0]} 0x{first[1]:03X} {first[2]} model={first[3]} emu={first[4]}")
        for a in WATCH:
            if mism[a]: print(f"     0x{a:03X} {WATCH[a]:8}: {mism[a]}")

if __name__=='__main__':
    difftest(sys.argv[1], sys.argv[2] if len(sys.argv)>2 else None)
