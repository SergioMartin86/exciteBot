#!/usr/bin/env python3
"""Extract the Excitebike track layout directly from the ROM (no nametable needed).

The ground engine reads track features per 'section' via tbl_E55A_lo/tbl_E56F_hi[section] -> a list of
2-byte [position, type] entries terminated by 0xFF (sub_E73B @ $E73B / $E762). 'position' is the in-section
scroll X at which the feature triggers; 'type' selects the terrain handler (sub_E794 dispatch; the 0x80/0x40
high bits flag the ground-angle and type-0x40 handlers). 21 sections (0x00..0x14).

Bank-FF (0xC000-0xFFFF) maps to ROM file offset addr-0xC000+0x10.
"""
import sys
rom = open('reference/Excitebike (JU) [!].nes','rb').read()
def rd(addr): return rom[addr - 0xC000 + 0x10]
NSEC = 21
los = [rd(0xE55A + i) for i in range(NSEC)]
his = [rd(0xE56F + i) for i in range(NSEC)]
print("# Excitebike track sections (from ROM). Each entry = (in-section pos, type) [type&0x3F=handler, +0x80/0x40 flags]")
for s in range(NSEC):
    ptr = los[s] | (his[s] << 8)
    entries = []
    off = 0
    while True:
        pos = rd(ptr + off)
        if pos == 0xFF: break
        typ = rd(ptr + off + 1)
        entries.append((pos, typ))
        off += 2
        if off > 200: entries.append(('OVERFLOW',)); break
    desc = ' '.join(f'{p}:0x{t:02X}' for (p,t) in entries if not isinstance(p,str))
    print(f"sec {s:2d} @0x{ptr:04X} ({len(entries)} feats): {desc}")
