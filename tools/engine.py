#!/usr/bin/env python3
"""Standalone native Excitebike physics model (Python reference for the C++ engine.hpp port).

Runs PURELY from inputs: seed the 2048-byte RAM from frame 0 of a dump (stands in for the unmodeled
boot/countdown), then advance(controllerByte) each frame with NO further peeking at the dump. Reward
is posX. This is the model we differential-test against the emulator (jaffar-player) on the two
references AND on alternative (suboptimal) movies.

Per-frame order mirrors sub_C99A (player object X=0):
  sub_CA08(sub_DCA0 rest-angle) -> (NMI sub_D310 timer dec) -> sub_E733 sections+handlers
  -> sub_E927 held-effect re-dispatch -> sub_CD59 speed+angle-tail -> sub_E96C lane
  -> sub_DA26 (sub_DA58 posX1 -> sub_DC1A landing -> sub_DCF2 arc -> sub_DCDE wobble-dec -> sub_DBFE posX2)
  -> sub_E70B column counter.

Usage: python3 tools/engine.py [tas|manual] [maxframes]
"""
import sys, os, re
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ---- tables ----
TBL_E6AD = [3,1,2,2,0,5,5,6,4,4]
TBL_D88B = [6,3,4,2,0x0B,8,9]
TBL_C0D4 = [6,4]; TBL_C0C8 = [6,2]; TBL_C0CA = [0x0A,0x0B]
TBL_E53D = [14,26,38,50]
TBL_D868 = [52,52,24,52]
TBL_D86C = [0x03,0x02,0x03,0x02,0x09,0x06,0x08,0x0F,0x03,0x02,0x02,0x02,0x08,0x05,0x07,0x0F]
TBL_D87C = [0x0C,0x09,0x0A,0x07,0x0C,0x0C,0x0C,0x00,0x0C,0x0A,0x0B,0x08,0x0C,0x0C,0x0C,0x00]
# speed tables (from engine.hpp)
TBL_ACCEL=[24,63,40]; TBL_FRIC=[56,12,0,60,28,192,127]
TBL_CAPLO=[0x20,0x40,0x7F]; TBL_CAPHI=[0x03,0x03,0x01]; TBL_FRICFLOOR=[0x01,0xB0]
TBL_TEMPEQ=[8,32,17,17]; TBL_TEMPRATE=[63,15,7,7]

def load_track():
    txt = open('source/track_layout.hpp').read()
    rows = re.findall(r'\{((?:0x[0-9A-F]{2},?)+)\}', txt)
    return [[int(x,16) for x in r.rstrip(',').split(',')] for r in rows]
TRACK = load_track()
def parse_sections():
    secs=[]
    for line in open('docs/track_sections.txt'):
        m=re.match(r'\s*sec\s+(\d+)\s+@\S+\s+\(\d+ feats\):\s*(.*)',line)
        if not m: continue
        feats=[(int(p),int(t,16)) for p,t in (tok.split(':') for tok in m.group(2).split())]
        secs.append(feats)
    return secs
SECTIONS = parse_sections()

def b(x): return x & 0xFF

class Engine:
    def __init__(self, ram0):
        self.m = bytearray(ram0[:0x800])
        self.cum = 0            # unwrapped cumulative column (tracks 0xE0)
        self.blockTrans = 0
        self.prevBlockX = self.m[0x4E]
        self.first = True
        self.bikePosX = 0.0

    def dec_timers(self):  # sub_D310: 0x20 frame counter gates the slow (0x30-0x3D) timers
        m=self.m
        m[0x20]=b(m[0x20]-1)
        hi = 0x3D if (m[0x20]&0x80) else 0x2F
        if m[0x20]&0x80: m[0x20]=0x0A
        for a in range(0x21, hi+1):
            if m[a]: m[a]=b(m[a]-1)

    def stall_process(self):  # sub_DD8D (overheat stall), player obj 0
        m=self.m
        if (m[0x3C]|m[0x3E0])==0: return
        c=m[0x3C]
        if c==8:                              # recovery
            m[0x3E0]=0; m[0x3B6]=5; m[0x9C]=5; m[0x374]=5
        elif c<8:
            pass                              # sprites only
        else:                                 # c>8: hold the bike stalled
            if (m[0x94]|m[0x98]|m[0x9C])==0:
                if m[0x58]!=0:
                    m[0x90]=0xC0
                else:
                    m[0x90]=0
                    m[0xDC]=0 if m[0xB8]==0x39 else 1   # tbl_D8C4[0]=0x39

    def lane_of(self, b8):
        a = b8 - 0x10
        if a < 0: return 5
        y = 5
        while True:
            y -= 1
            if y == 0: return 0
            a -= 8
            if a < 0: return y

    # ---- terrain handlers (sub_E893 etc.) ----
    def e893(self, a):  # D4=B4=A; return (C8 - D0) = distance past the feature (drives climb 0xBC)
        m=self.m; m[0xD4]=a; m[0xB4]=a
        return b(m[0xC8]-m[0xD0])
    def e84F(self, a): self.m[0xBC]=a; self.m[0xE4]=a
    def e86F(self, a): self.m[0xBC]=b(self.m[0xE4]-a)
    def takeoff(self, set384):
        m=self.m
        # loc_DCFA/DCFE/DD06 preamble then sub_DD38 then one arc step (loc_DD1A)
        m[0x384]=set384
        # sub_DD38:
        m[0xB0]=2; m[0x380]=0x0F
        z = (m[0x94]<<8)|m[0x90]
        z = (z + 0xAF) & 0xFFFF
        m[0x378]=z & 0xFF; m[0x37C]=(z>>8)&0xFF
        if m[0x388]==2:
            z16=((m[0x37C]<<8)|m[0x378])>>1
            m[0x37C]=(z16>>8)&0xFF; m[0x378]=z16&0xFF
        # one immediate arc step at loc_DD1A (carry-in = ? leftover; use 0x5C bit0 like normal)
        self.arc_step(m[0x5C]&1)
        if m[0xCC]==0: m[0x364]=1
    def arc_step(self, cin):
        m=self.m
        m[0x38C]=TBL_D868[m[0x5C]&3]
        t=m[0x380]+m[0x38C]+cin; m[0x380]=t&0xFF; c1=1 if t>0xFF else 0
        t2=m[0x384]+c1; m[0x384]=t2&0xFF; c2=1 if t2>0xFF else 0
        sub=m[0x8C]-m[0x37C]-(1-c2); c3=1 if sub>=0 else 0
        m[0x8C]=b(b(sub)+m[0x384]+c3)

    def dispatch(self, typ):
        m=self.m
        if typ==0x06:    # ramp E934
            if m[0xB0]==0 and m[0x94]!=0:
                # E934: 0x388 = (velX_hi<2 and velX_lo<0xD8)?1:0 ; then loc_DCFA (set384=0) takeoff
                y=0
                if m[0x94]<2 and m[0x90] < m[0xD8]: y=1
                m[0x388]=y
                if m[0xA0]!=2:
                    self.takeoff(0)
        elif typ==0x13:  # downhill E8E7 -> loc_DCFE
            if m[0x98]==0:
                m[0x94]=b(m[0x94]+1)
                if m[0xB0]==0:
                    self.takeoff(0)
        elif typ==0x00: m[0xCC]=0; m[0xD4]=0; m[0xB4]=0
        elif typ==0x07:  # crash E818
            if m[0xB0]==0 and m[0xAC]<7:
                vhi=m[0x94]
                if vhi>=3 or (vhi==2 and (m[0x90]&0x80)): m[0x98]=0xFF
        elif typ==0x02:  # E85D
            self.e84F(self.e893(6)); m[0xD8]=0x60
        elif typ==0x03:  # E86A
            self.e86F(self.e893(1))
        elif typ==0x04:  # E845
            m[0xD8]=0x80; self.e84F(self.e893(5)>>1)
        elif typ==0x05:  # E854
            self.e86F(self.e893(2)>>1)
        elif typ==0x09:  # E879
            self.e84F(b(self.e893(4)<<1))
            if m[0xA0]!=0: self.e84F(b(m[0xBC]+0x10))
            m[0xD8]=0x40
        elif typ==0x0A:  # E89D (gated by 0xA4)
            if m[0xA4] not in (1,):
                self.e86F(b(self.e893(3)<<1))
        elif typ==0x08:  # EA8F: finish line -> win (0x52=1) iff all laps done (0x57==0)
            m[0x3A]=0x1D
            if m[0x57]==0:
                m[0x32]=0x10; m[0xFD]=2; m[0x52]=1; m[0x3A]=0
        elif typ==0x12:  # E8D3: cooling zone -> reset engine temp 0x3B6=8 (if on ground & not stalled)
            if (m[0xB0]|m[0x3E0]|m[0x3C])==0: m[0x3B6]=8
        elif typ==0x0C:  # E8FF: mud/sand -> sub_CE5C friction (decelerate)
            if m[0xB0]==0:
                if not (m[0xA4]!=0 and (m[0xA4]&2)==0):
                    if m[0x94]!=0:
                        m[0x36C]=1
                        self.applyFriction(5 if (m[0x5C]&0x40) else 6)
        # other types: no 0xBC/0xD4 effect modeled yet

    def handler_angle(self, typ):  # E7A3
        m=self.m
        if m[0xB0] or m[0x98]: return
        nib=typ&0x0F
        if m[0xA4]==1: return
        m[0xAC]=nib
        if m[0x58]!=3:
            idx=nib-2
            if 0<=idx<len(TBL_E6AD): m[0xD4]=TBL_E6AD[idx]

    def section_machinery(self):  # sub_E73B, player
        m=self.m
        lane=m[0x360]
        m[0xC8]=b(m[0xC8]+m[0x60])   # uses prev frame's 0x60 (posX not yet updated this frame)
        if m[0x58]==0:
            col=self.cum
            c0=TRACK[lane][col] if col<len(TRACK[lane]) else 0x3B
            m[0xC0]=c0
            t=c0-0x40
            if t<0: return
            t>>=2
            if t>=0x16: return
            m[0x58]=b(t+1); m[0xC4]=0; m[0xC8]=b(m[0x64]-1)
        while True:
            blk=m[0x58]-1
            feats=SECTIONS[blk] if 0<=blk<len(SECTIONS) else []
            cur=m[0xC4]//2
            if cur>=len(feats):
                # terminator (bra_E7D0): 0x58=0, 0xD4=0; if 0xA0==0 and 0xA4!=1: 0xBC=0 (+A4==2->A4++); 0x36C=0
                m[0x58]=0; m[0xD4]=0
                if m[0xA0]==0 and m[0xA4]!=1:
                    m[0xBC]=0
                    if m[0xA4]==2: m[0xA4]=b(m[0xA4]+1)
                m[0x36C]=0
                return
            pos,typ=feats[cur]
            if pos>m[0xC8]: return
            if typ&0x80: self.handler_angle(typ)
            elif typ&0x40: m[0xCC]=typ&0x0F; m[0xD0]=pos
            else: self.dispatch(typ)
            m[0xC4]=b(m[0xC4]+2)

    def set_rest_angle(self):  # sub_DCA0
        m=self.m; y=m[0xD4]
        if m[0x52]==1 and (m[0x98]|m[0x9C]|m[0x58])==0: m[0x368]=0x0A
        else: m[0x368]=TBL_D88B[y] if y<len(TBL_D88B) else 0
        if m[0xA4]==1: m[0x368]=6

    def angle_update(self):  # loc_CDBD tail
        m=self.m
        if m[0x98]: return
        air = m[0xB0]!=0
        if air: active=True
        else:
            if (m[0x58]|m[0x52])!=0: active=False
            elif m[0x94]!=0 or m[0x90]>=0xA0: active=True
            else: active=False
        a000A=m[0x5C]&3
        if active and a000A!=0: self.tilt(a000A)
        elif not air and m[0x368]!=m[0xAC]: self.ground_ramp()

    def ground_ramp(self):  # loc_DCC7
        m=self.m
        if m[0x2A]!=0: return
        m[0x2A]=5
        ac=m[0xAC]; tgt=m[0x368]
        if ac==tgt: return
        m[0xAC]=b(ac+(1 if ac<tgt else -1))
    def tilt(self, a000A):  # loc_CE83
        m=self.m
        if m[0x26]!=0: return
        y=m[0xB0]>>1
        m[0x26]=TBL_C0D4[y]
        ac=m[0xAC]
        if a000A&1:
            tgt=TBL_C0C8[y]
            if ac==tgt: return
            m[0xAC]=b(ac+(1 if ac<tgt else -1))
        else:
            m[0x388]=m[0x388]&2
            if ac<TBL_C0CA[y]: m[0xAC]=ac+1; return
            if (m[0x5C]&0xC0)==0: return
            if m[0xB0]!=0: return
            m[0xAC]=ac+1; m[0x26]=0x0D
            if m[0xAC]>=0x0D: m[0x98]=1; m[0x26]=0x1A

    # ---- speed (sub_CD59) ----
    def applyAccel(self, y):
        m=self.m
        lo=m[0x90]+TBL_ACCEL[y]; m[0x90]=lo&0xFF
        if lo>0xFF: m[0x94]=b(m[0x94]+1)
        if m[0x94]>TBL_CAPHI[y] or (m[0x94]==TBL_CAPHI[y] and m[0x90]>=TBL_CAPLO[y]):
            m[0x90]=TBL_CAPLO[y]; m[0x94]=TBL_CAPHI[y]
    def applyFriction(self, y):
        m=self.m
        if m[0x94]==0 and m[0x90]<TBL_FRICFLOOR[(m[0xB0]>>1)&1]: return
        lo=m[0x90]-TBL_FRIC[y]; m[0x90]=lo&0xFF
        if lo<0:
            if m[0x94]!=0: m[0x94]=b(m[0x94]-1)   # borrow into hi
            else: m[0x90]=0                         # velX_hi==0: clamp to 0 (sub_CE58 bra_CE80), not wrap
    def stepSpeed(self):
        m=self.m
        if m[0x4C]!=0: return
        if m[0x374]!=0 or m[0x98]!=0: self.applyFriction(0); return
        if m[0xB0]!=0:
            lean=m[0x5C]&3
            self.applyFriction(4 if lean==0 else lean+1); return
        ab=m[0x5C]&0xC0
        if ab==0: self.applyFriction(0); return
        y=0 if (m[0x5C]&0x80) else 1
        vhi,vlo=m[0x94],m[0x90]
        if vhi>TBL_CAPHI[y] or (vhi==TBL_CAPHI[y] and vlo>TBL_CAPLO[y]): self.applyFriction(y+1)
        elif vhi==TBL_CAPHI[y] and vlo==TBL_CAPLO[y]: pass
        else: self.applyAccel(y)

    def stepTemp(self):
        m=self.m
        ab=m[0x5C]&0xC0; y=0
        if ab!=0:
            a=ab if m[0x4F]!=0 else 0x80
            c=(a>>7)&1; a=(a<<1)&0xFF
            for _ in range(2):
                nc=(a>>7)&1; a=((a<<1)|c)&0xFF; c=nc
            y=a
        equ=TBL_TEMPEQ[y]
        if m[0x3B6]<equ:
            s=m[0x3B5]+TBL_TEMPRATE[y]; m[0x3B5]=s&0xFF
            if s>0xFF: m[0x3B6]=b(m[0x3B6]+1)
        elif m[0x3B6]>equ:
            s=m[0x3B5]-0x0B; m[0x3B5]=s&0xFF
            if s<0 and m[0x3B6]!=0: m[0x3B6]=b(m[0x3B6]-1)

    # ---- lane (sub_E96C, player X=0) ----
    def flipVelZ(self):  # bra_E9D9: 0xDC = (0xDC if !=0 else 0xFF) ^ 0xFE  (toggles +1 <-> -1)
        m=self.m
        v=m[0xDC] if m[0xDC]!=0 else 0xFF
        m[0xDC]=v^0xFE
    def stepLane(self):
        m=self.m
        if m[0x58]!=0x14 and m[0x58]!=0x15:
            m[0xB8]=b(m[0xB8]+(m[0xDC] if m[0xDC]<0x80 else m[0xDC]-0x100))
        if m[0xB8] in TBL_E53D:        # bra_EA01: snap velZ=0, skip clamp
            m[0xDC]=0
        else:
            a4=m[0xA4]
            if a4==0: pass             # bra_E9C4
            elif a4==1:
                if m[0xB8]<0x20: m[0xA4]=4; m[0xBC]=0  # (sub_DD06 ramp landing simplified)
            elif a4==3: pass
            else:
                if m[0xB8]>=0x20: m[0xB8]=b(m[0xB8]-1)
            # bra_E9C4 clamp (+ velZ flip at the track edges, player X=0)
            if m[0xB8]<8:              # bra_E9E9
                if m[0x9C]==0: m[0xB8]=7
                elif m[0xB8]<2: m[0xB8]=1
                self.flipVelZ()
            elif m[0xB8]>=0x3A:        # B8=0x39 then bra_E9D2
                m[0xB8]=0x39
                if (m[0x9C]|m[0x3E0])==0: self.flipVelZ()
        # player lane input (EA07+)
        if m[0x4F]!=0 and (m[0x98]|m[0x3E0]|m[0x3F7])==0:
            ok=True
            if m[0xB0]!=0:             # airborne: only if 0x388==2 (and INC it)
                if m[0x388]==2: m[0x388]=b(m[0x388]+1)
                else: ok=False
            if ok and (m[0x9C] in (0,5)):
                ud=m[0x5C]&0x0C
                if ud: m[0xDC]=0xFF if (m[0x5C]&0x04) else 0x01
        # lane index 0x360 computed at frame START (render lags 0xB8 by one frame)

    # ---- landing (sub_DC1A) ----
    def landing(self):
        m=self.m
        if m[0xB0]!=2: return
        groundY=b(0xA0 - m[0xBC] - m[0xB8])
        if not (m[0x8C]>groundY and m[0x8C]<0xA8 and m[0x364]!=0): return
        m[0xB0]=0; m[0x364]=0; oldJ=m[0x388]; m[0x388]=0; m[0x8C]=b(groundY-1)
        if oldJ!=0:
            return  # post-bounce clean landing
        # first-contact resolve
        y=m[0xD4]+(8 if m[0x94]<2 else 0)
        if m[0xAC]<TBL_D86C[y] or m[0xAC]>=TBL_D87C[y]:
            m[0x98]=0xFF; return
        if m[0xAC]==m[0x368] or (m[0xD4]|m[0xCC])!=0:
            return  # clean
        m[0x374]=4; m[0x388]=2; m[0xB0]=1  # wobble -> bounce relaunch in arc step

    def arc(self):  # sub_DCF2
        m=self.m
        if m[0xB0]==0: return
        if m[0xB0]==2:
            lean0=m[0x5C]&1
            if lean0==1 and m[0x4C]==0: return
            self.arc_step(lean0)
            if m[0xCC]==0: m[0x364]=1
        else:  # airMode==1: re-launch (bounce) via loc_DCFA (0x384=0) -> sub_DD38 (halved) -> arc step
            self.takeoff(0)

    def wobble_dec(self):  # sub_DCDE
        m=self.m
        if m[0x374]==0: return
        m[0x374]=b(m[0x374]-1)
        if (m[0x5C]&0xC0)==0: return
        # (sub_DCDE tail clears 0x374 on A/B -- handled in advance() like engine.hpp)

    # ---- posX (sub_DA58 + sub_DBFE) ----
    def stepPosX(self):
        m=self.m
        m[0x60]=m[0x94]
        sp=m[0x394]+m[0x90]; m[0x394]=sp&0xFF
        if sp>0xFF: m[0x60]=b(m[0x60]+1)
    def stepPosX2(self):
        m=self.m
        a=m[0x60]; c=a&1; a=a>>1
        for _ in range(3):
            nc=a&1; a=((c<<7)|(a>>1))&0xFF; c=nc
        ss=a+m[0x3BF]; m[0x3BF]=ss&0xFF
        cin=1 if ss>0xFF else 0
        px=m[0x60]+m[0x50]+cin; m[0x50]=px&0xFF
        if px>0xFF: m[0x4E]^=1

    def colcounter(self):  # sub_E70B
        m=self.m; v=m[0x60]
        if v!=0:
            bd=b(m[0x64]-v)
            if bd==0 or (bd&0x80):
                m[0x64]=b(bd+8); m[0xE0]=(m[0xE0]+1)&0x3F; self.cum+=1
            else:
                m[0x64]=bd

    def updateDerived(self):
        m=self.m
        if not self.first and self.prevBlockX!=m[0x4E]: self.blockTrans+=1
        self.prevBlockX=m[0x4E]; self.first=False
        self.bikePosX=self.blockTrans*256.0 + m[0x50] + m[0x394]/256.0

    def advance(self, ctrl):
        m=self.m
        # game cycle
        m[0x4C]=(m[0x4C]+1)&3
        # input latch (reverse8): A=01->80,... we store the 0x5C-layout byte directly
        m[0x5C]=reverse8(ctrl)
        # lap counter (0x57 loops-remaining / 0x3A4 current-loop) from cumulative column. race01 is a
        # 3-lap track; boundaries at cum 668/1336 (set by the streaming lap-advance, sub_F520/F638).
        # Drives the type-0x08 finish-line win check. (Exact value; init ~1 frame early vs dump.)
        if self.cum != 0:
            laps = 1 + (1 if self.cum>=668 else 0) + (1 if self.cum>=1335 else 0)
            m[0x3A4]=laps; m[0x57]=3-laps
            if m[0x57]==0: m[0x3BC]=1
        # lane index 0x360 (sub_D0AB, render): computed from PREVIOUS frame's 0xB8 (renders lags by 1).
        # Done at frame start so section machinery reads 0xC0=kTrack[lane_i][cum_{i-1}] with lane_i here.
        m[0x360]=self.lane_of(m[0xB8])
        # sub_CA08 -> sub_DCA0 rest-angle (uses current 0xD4)
        self.set_rest_angle()
        # NMI sub_D310: 0x20 counter -> 0x21-0x2F dec every frame; 0x21-0x3D (incl stall 0x3C) every 11
        self.dec_timers()
        # sub_DD8D: overheat-stall processing (force velX/velZ; recovery at 0x3C==8)
        self.stall_process()
        # sub_E733 sections + handlers (reads prev 0x60/0x64, prev cum, current lane 0x360)
        self.section_machinery()
        # sub_E927 held-effect re-dispatch
        if m[0xCC]: self.dispatch(m[0xCC])
        # sub_CD1F input lock: stall/crash/recovery zeroes throttle (0x5C) before the speed routine
        if m[0xA8]!=0 and m[0x9C]!=5 and (m[0x9C]|m[0x98]|m[0x3E0])!=0: m[0x5C]=0
        # sub_CD59 speed + angle tail
        self.stepSpeed()
        self.angle_update()
        self.stepTemp()
        # sub_E359 loc_E3AA overheat trigger (late): temp>=0x20 arms the stall (0x3C=0x3E0=temp)
        if m[0x3B6]>=0x20:
            m[0x3C]=m[0x3B6]; m[0x3E0]=m[0x3B6]
        # sub_E96C lane
        self.stepLane()
        # sub_E836: recompute visual 0x8C from 0xB8 on the ground (sub_DC97: 0x3F1 - 0xBC - 0xB8)
        if m[0xB0]==0: m[0x8C]=b(m[0x3F1]-m[0xBC]-m[0xB8])
        # sub_DA26: posX1 -> landing -> arc -> wobble-dec -> posX2
        self.stepPosX()
        self.landing()
        self.arc()
        self.wobble_dec()
        self.stepPosX2()
        # sub_E70B column counter
        self.colcounter()
        # derived + wobble timer (engine.hpp parity for A/B clear)
        self.updateDerived()
        if m[0x374]!=0 and (m[0x5C]&0xC0): m[0x374]=0

def reverse8(x):
    x=((x&0xF0)>>4)|((x&0x0F)<<4)
    x=((x&0xCC)>>2)|((x&0x33)<<2)
    x=((x&0xAA)>>1)|((x&0x55)<<1)
    return x&0xFF

# inverse of reverse8 for deriving controller byte from a dump's 0x5C (for replay-from-dump testing)
def unreverse_to_ctrl(v): return reverse8(v)

if __name__=='__main__':
    F=2048
    which=sys.argv[1] if len(sys.argv)>1 else 'tas'
    RAM={'tas':'tas.ram','manual':'manual.ram'}[which]
    d=open(RAM,'rb').read(); FR=[d[i*F:(i+1)*F] for i in range(len(d)//F)]
    maxf=int(sys.argv[2]) if len(sys.argv)>2 else len(FR)
    eng=Engine(FR[0])
    # derive controller inputs from each frame's 0x5C (ctrl = reverse8(0x5C))
    watch=[0x50,0x394,0x94,0x90,0x60,0x58,0xC4,0xC8,0xAC,0xD4,0xB0,0xB8,0x360,0xDC,0x8C,0xE0,0x3B6,0x3C,0x52,0x57,0x3A4]
    names={0x52:'raceOver',0x57:'loops',0x3A4:'loop',0x50:'posXlo',0x394:'posXsub',0x94:'vXhi',0x90:'vXlo',0x60:'scrl',0x58:'sect',0xC4:'cur',0xC8:'inpos',0xAC:'ang',0xD4:'slope',0xB0:'air',0xB8:'posY',0x360:'lane',0xDC:'velZ',0x8C:'8C',0xE0:'E0',0x3B6:'temp',0x3C:'stall'}
    mism={a:0 for a in watch}; firstper={a:None for a in watch}
    for i in range(1,maxf):
        ctrl=reverse8(FR[i][0x5C])
        eng.advance(ctrl)
        for a in watch:
            if eng.m[a]!=FR[i][a]:
                mism[a]+=1
                if firstper[a] is None: firstper[a]=(i,eng.m[a],FR[i][a])
    print(f"=== engine vs {RAM} ({maxf} frames) ===")
    for a in watch:
        if mism[a]: print(f"  0x{a:03X} {names[a]:7}: mism={mism[a]:5d}  first f{firstper[a][0]} (model={firstper[a][1]} dump={firstper[a][2]})")
    if not any(mism.values()): print("  BYTE-EXACT on all watched addresses!")
