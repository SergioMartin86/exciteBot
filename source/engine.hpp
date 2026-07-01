#pragma once
/**
 * @file engine.hpp
 * @brief Native, from-scratch re-implementation of the Excitebike (NES) bike physics.
 *
 * The "emulator replacement" at the heart of exciteBot: instead of interpreting the 6502 ROM, it
 * models the game's low work-RAM ($0000-$07FF) and advances the bike physics one frame at a time,
 * PREDICTIVELY (from inputs alone) using the ROM-extracted 2D track. Public surface mirrors a
 * JaffarPlus Game: advance(controllerByte), reward()=posX, isWin()=RAM 0x52>0, isFail()=RAM 0x98>0,
 * serialize()/deserialize(), lram()=the 2048-byte modeled RAM image for the byte-for-byte fidelity gate.
 *
 * FIDELITY (2026-06-27): byte-exact vs the reference TAS (race01.sol) on every reward-critical address
 * (posX, velX, sections, angle, slope, airMode, lane, posY, height, column, temp, stall) for all 2263
 * frames, run purely from inputs (seed frame 0 from tas.ram for the un-modeled boot). This is a direct
 * port of the validated tools/engine.py (see docs/2d_engine_spec.md). Modeled subsystems:
 *   - speed (sub_CD59 + CE29/CE58), posX (sub_DA58/DBFE), engine temperature (sub_E359);
 *   - column counter (sub_E70B); lane/posY (sub_E96C + sub_D0AB), 0xC0 = kTrack[lane_i][cum_(i-1)];
 *   - section machinery (sub_E73B) + terrain handlers + sub_E927 held-effect re-dispatch;
 *   - angle/slope (sub_DCA0 + sub_CD59 tail: ground ramp loc_DCC7, player tilt loc_CE83);
 *   - airborne takeoff (sub_DD38) + arc (sub_DD6F) + landing/bounce (sub_DC1A); mud (0x0C), cooling (0x12);
 *   - overheat stall (loc_E3AA + sub_DD8D + input lock) and the sub_D310 timer routine.
 * Not modeled (TAS-irrelevant long tail; see spec §8b): 0x9C start/recovery state machine, crash
 * animation, 0x52 loop-completion win flag.
 *
 * Seed frame 0 from tas.ram (stands in for the un-modeled boot/countdown).
 */

#include <cstdint>
#include <cstring>
#include <cstddef>

#include "track_layout.hpp"   // generated: kTrack[6][1376] (per-lane terrain tiles)
#include "track_sections.hpp" // generated: kSections / kSectionLen (per-section feature lists)

namespace excitebike
{

/// @brief Reverse the bit order of a byte. The NES controller is read MSB-first, so the jaffar joypad
/// code (A=0x01..R=0x80) lands in RAM 0x5C bit-reversed (A=0x80,B=0x40,U=0x08,D=0x04,L=0x02,R=0x01).
static inline uint8_t reverse8(uint8_t b)
{
  b = (uint8_t)((b & 0xF0) >> 4 | (b & 0x0F) << 4);
  b = (uint8_t)((b & 0xCC) >> 2 | (b & 0x33) << 2);
  b = (uint8_t)((b & 0xAA) >> 1 | (b & 0x55) << 1);
  return b;
}

class Engine
{
public:
  static constexpr size_t LRAM_SIZE = 0x800;

  enum Addr : uint16_t
  {
    A_GAME_CYCLE = 0x004C, A_CURR_BLOCK_X = 0x004E, A_RACE_STARTED = 0x004F,
    A_SCROLL_X = 0x0050, A_RACE_OVER = 0x0052, A_THROTTLE = 0x005C,
    A_VELX_LO = 0x0090, A_VELX_HI = 0x0094, A_CRASH = 0x0098,
    A_ANGLE = 0x00AC, A_AIRMODE = 0x00B0, A_SLOPE = 0x00D4,
    A_POSX_SUB = 0x0394, A_CURRENT_LOOP = 0x03A4, A_LOOPS_REMAIN = 0x0057, A_TEMP = 0x03B6,
  };

  Engine() { reset(); }

  void reset()
  {
    std::memset(_mem, 0, sizeof(_mem));
    _mem[A_ANGLE] = 6;
    _cum = 0; _blockXTransitions = 0; _prevBlockX = _mem[A_CURR_BLOCK_X];
    _firstPostHook = true; _currentStep = 0;
    updateDerived();
  }

  /// @brief Seed the full 2048-byte RAM from a real frame-0 snapshot (first frame of tas.ram), standing
  /// in for the un-modeled boot. The cumulative column is recovered from 0xE0 (= initial column mod 64).
  void seedFromRam(const uint8_t* ram2048, uint8_t initialBlockTransitions = 0)
  {
    std::memcpy(_mem, ram2048, LRAM_SIZE);
    _cum = 0; // 0xC0 lookup uses cum_(i-1); at frame 0 the bike is at the track start (cum 0)
    _blockXTransitions = initialBlockTransitions; _prevBlockX = _mem[A_CURR_BLOCK_X];
    _firstPostHook = true; _currentStep = 0;
    updateDerived();
  }

  /// @brief Advance one frame. @p controllerByte is the jaffar joypad code (A=0x01..R=0x80).
  void advance(uint8_t controllerByte)
  {
    uint8_t* m = _mem;
    m[A_GAME_CYCLE] = (uint8_t)((m[A_GAME_CYCLE] + 1) & 3);   // game cycle 0x4C
    m[A_THROTTLE]   = reverse8(controllerByte);               // input latch 0x5C

    // lap counter (0x57 loops-remaining / 0x3A4 current-loop) from cumulative column. race01 is a 3-lap
    // track; boundaries at cum 668/1335 (the streaming lap-advance, sub_F520/F638). Feeds the type-0x08
    // finish-line win check.
    if (_cum != 0)
    {
      int laps = 1 + (_cum >= 668) + (_cum >= 1335);
      m[A_CURRENT_LOOP] = (uint8_t)laps; m[A_LOOPS_REMAIN] = (uint8_t)(3 - laps);
      if (m[A_LOOPS_REMAIN] == 0) m[0x3BC] = 1;
    }
    // lane index 0x360 (sub_D0AB, render): from PREVIOUS frame's 0xB8 (render lags 0xB8 by one frame)
    m[0x360] = laneOf(m[0xB8]);
    setRestAngle();                                           // sub_CA08 -> sub_DCA0
    decTimers();                                              // NMI sub_D310 (gates stall timer 0x3C)
    stallProcess();                                           // sub_DD8D (overheat stall)
    sectionMachinery();                                       // sub_E733
    if (m[0xCC]) dispatch(m[0xCC]);                           // sub_E927 held-effect re-dispatch
    if (m[0xA8] != 0 && m[0x9C] != 5 && (m[0x9C] | m[0x98] | m[0x3E0]) != 0) m[A_THROTTLE] = 0; // input lock
    stepSpeed();                                             // sub_CD1F -> sub_CD59
    angleUpdate();                                           // sub_CD59 tail
    stepTemp();                                              // sub_E359
    if (m[A_TEMP] >= 0x20) { m[0x3C] = m[A_TEMP]; m[0x3E0] = m[A_TEMP]; } // loc_E3AA overheat arm
    stepLane();                                              // sub_E96C
    if (m[A_AIRMODE] == 0) m[0x8C] = bb(m[0x3F1] - m[0xBC] - m[0xB8]); // sub_E836 (visual 0x8C on ground)
    // sub_DA26: posX-part1 -> landing -> arc -> wobble-dec -> posX-part2
    stepPosX1(); landing(); arc(); wobbleDec(); stepPosX2();
    colCounter();                                            // sub_E70B (late)
    updateDerived();
    if (m[0x374] != 0 && (m[A_THROTTLE] & 0xC0)) m[0x374] = 0; // 0x374 cleared on A/B (sub_DCDE tail)
    _currentStep++;
  }

  // --- Outputs ---------------------------------------------------------------------------------
  uint16_t velX16() const { return (uint16_t)((_mem[A_VELX_HI] << 8) | _mem[A_VELX_LO]); }
  float    reward() const { return _bikePosX; }
  bool     isWin()  const { return _mem[A_RACE_OVER] != 0; }
  bool     isFail() const { return _mem[A_CRASH] != 0; }
  float    bikePosX() const { return _bikePosX; }
  uint8_t  blockXTransitions() const { return _blockXTransitions; }
  uint32_t currentStep() const { return _currentStep; }
  uint32_t cumColumn() const { return _cum; }
  const uint8_t* lram() const { return _mem; }
  uint8_t        ram(uint16_t a) const { return _mem[a]; }

  // --- Packed working-set serialization ---------------------------------------------------------
  // Only the RAM bytes the engine actually reads/writes across frames carry state; everything else is
  // scratch or unmodeled cosmetics. kStateAddrs is that working set (proven complete by the round-trip
  // test in tools/test_serialize.cpp). The packed State is ~9x smaller than the 2KB RAM image and ~32x
  // smaller than the emulator's machine state -- the depth advantage the search needs.
  static constexpr uint16_t kStateAddrs[] = {
    // game cycle, posX, velX, column
    0x4C, 0x4E, 0x50, 0x60, 0x64, 0x90, 0x94, 0xE0, 0x394, 0x3BF,
    // section machinery + terrain-handler persistent state
    0x58, 0xC4, 0xC8, 0xCC, 0xD0, 0xD4, 0xB4, 0xD8, 0xE4, 0xBC,
    // angle + its timers
    0xAC, 0x368, 0x26, 0x2A,
    // airborne arc / landing
    0xB0, 0x8C, 0x364, 0x388, 0x378, 0x37C, 0x380, 0x384, 0x38C, 0x374,
    // lane / posY
    0xB8, 0xDC, 0x360,
    // temperature + overheat stall
    0x3B5, 0x3B6, 0x20, 0x3C, 0x3E0,
    // race/bike flags + (read-only) constants the engine reads
    0x98, 0x9C, 0xA0, 0xA4, 0xA8, 0x4F, 0x3F1, 0x3F7,
    // 0x36C: wheelie/ground-contact latch read across frames by the speed gate (sub_CDEE)
    0x36C,
    // laps / win
    0x52, 0x57, 0x3A4, 0x3BC,
  };
  static constexpr size_t kStateAddrCount = sizeof(kStateAddrs) / sizeof(kStateAddrs[0]);

  struct State
  {
    uint8_t  packed[kStateAddrCount];
    uint16_t cum;                 // cumulative column (<= 1376, fits in 16 bits)
    uint8_t  blockXTransitions, prevBlockX, firstPostHook;
    uint32_t currentStep;
  };
  void serialize(State& s) const
  {
    for (size_t i = 0; i < kStateAddrCount; i++) s.packed[i] = _mem[kStateAddrs[i]];
    s.cum = (uint16_t)_cum; s.blockXTransitions = _blockXTransitions; s.prevBlockX = _prevBlockX;
    s.firstPostHook = _firstPostHook ? 1 : 0; s.currentStep = _currentStep;
  }
  void deserialize(const State& s)
  {
    std::memset(_mem, 0, LRAM_SIZE);  // non-state bytes are scratch -> restore to deterministic zero
    for (size_t i = 0; i < kStateAddrCount; i++) _mem[kStateAddrs[i]] = s.packed[i];
    _cum = s.cum; _blockXTransitions = s.blockXTransitions; _prevBlockX = s.prevBlockX;
    _firstPostHook = s.firstPostHook != 0; _currentStep = s.currentStep;
    updateDerived();
  }

private:
  static inline uint8_t bb(int x) { return (uint8_t)(x & 0xFF); }

  // ---- physics tables ----
  static constexpr uint8_t kAccel[3]    = {24, 63, 40};
  static constexpr uint8_t kFriction[7] = {56, 12, 0, 60, 28, 192, 127};
  static constexpr uint8_t kCapLo[3]    = {0x20, 0x40, 0x7F};
  static constexpr uint8_t kCapHi[3]    = {0x03, 0x03, 0x01};
  static constexpr uint8_t kFricFloor[2]= {0x01, 0xB0};
  static constexpr uint8_t kTempEquil[4]= {8, 32, 17, 17};       // tbl_D8FB
  static constexpr uint8_t kTempRate[4] = {63, 15, 7, 7};        // tbl_D8F7
  static constexpr uint8_t kE6AD[10]    = {3,1,2,2,0,5,5,6,4,4};
  static constexpr uint8_t kD88B[7]     = {6,3,4,2,0x0B,8,9};
  static constexpr uint8_t kC0D4[2]     = {6,4};
  static constexpr uint8_t kC0C8[2]     = {6,2};
  static constexpr uint8_t kC0CA[2]     = {0x0A,0x0B};
  static constexpr uint8_t kE53D[4]     = {14,26,38,50};
  static constexpr uint8_t kD868[4]     = {52,52,24,52};
  static constexpr uint8_t kD86C[16]    = {0x03,0x02,0x03,0x02,0x09,0x06,0x08,0x0F,0x03,0x02,0x02,0x02,0x08,0x05,0x07,0x0F};
  static constexpr uint8_t kD87C[16]    = {0x0C,0x09,0x0A,0x07,0x0C,0x0C,0x0C,0x00,0x0C,0x0A,0x0B,0x08,0x0C,0x0C,0x0C,0x00};

  uint8_t laneOf(uint8_t b8) const
  {
    int a = (int)b8 - 0x10;
    if (a < 0) return 5;
    int y = 5;
    for (;;) { y--; if (y == 0) return 0; a -= 8; if (a < 0) return (uint8_t)y; }
  }

  // ---- terrain handlers ----
  uint8_t e893(uint8_t a) { _mem[0xD4] = a; _mem[0xB4] = a; return bb(_mem[0xC8] - _mem[0xD0]); }
  void e84F(uint8_t a) { _mem[0xBC] = a; _mem[0xE4] = a; }
  void e86F(uint8_t a) { _mem[0xBC] = bb(_mem[0xE4] - a); }

  void takeoff(uint8_t set384)  // loc_DCFA/DCFE/DD06 + sub_DD38 + one arc step (loc_DD1A)
  {
    uint8_t* m = _mem;
    m[0x384] = set384;
    m[0xB0] = 2; m[0x380] = 0x0F;
    int z = ((m[0x94] << 8) | m[0x90]) + 0xAF;
    m[0x378] = bb(z); m[0x37C] = bb(z >> 8);
    if (m[0x388] == 2) { int z16 = ((m[0x37C] << 8) | m[0x378]) >> 1; m[0x37C] = bb(z16 >> 8); m[0x378] = bb(z16); }
    arcStep(m[0x5C] & 1);
    if (m[0xCC] == 0) m[0x364] = 1;
  }
  void arcStep(int cin)  // sub_DD6F
  {
    uint8_t* m = _mem;
    m[0x38C] = kD868[m[0x5C] & 3];
    int t = m[0x380] + m[0x38C] + cin; m[0x380] = bb(t); int c1 = t > 0xFF;
    int t2 = m[0x384] + c1; m[0x384] = bb(t2); int c2 = t2 > 0xFF;
    int sub = (int)m[0x8C] - m[0x37C] - (1 - c2); int c3 = sub >= 0;
    m[0x8C] = bb(bb(sub) + m[0x384] + c3);
  }

  void dispatch(uint8_t typ)
  {
    uint8_t* m = _mem;
    switch (typ)
    {
      case 0x06: // ramp E934
        if (m[0xB0] == 0 && m[0x94] != 0)
        {
          int y = (m[0x94] < 2 && m[0x90] < m[0xD8]) ? 1 : 0; m[0x388] = (uint8_t)y;
          if (m[0xA0] != 2) takeoff(0);
        }
        break;
      case 0x13: // downhill E8E7: airborne (0xB0!=0) -> RTS, NO velX boost (gravity boosts velX only
                 // on the ground; in the air velX is MAINTAINED). Grounded -> loc_DCFE: velX_hi++
                 // (unless stalled 0x98) then takeoff. Omitting the air gate let a jump harvest phantom
                 // downhill speed (search-found big jumps over a downhill diverged from real HW).
        if (m[0xB0] != 0) break;
        if (m[0x98] == 0) { m[0x94] = bb(m[0x94] + 1); takeoff(0); }
        break;
      case 0x00: m[0xCC] = 0; m[0xD4] = 0; m[0xB4] = 0; break;
      case 0x07: // crash E818
        if (m[0xB0] == 0 && m[0xAC] < 7)
        { uint8_t vhi = m[0x94]; if (vhi >= 3 || (vhi == 2 && (m[0x90] & 0x80))) m[0x98] = 0xFF; }
        break;
      case 0x02: e84F(e893(6)); m[0xD8] = 0x60; break;
      case 0x03: e86F(e893(1)); break;
      case 0x04: m[0xD8] = 0x80; e84F((uint8_t)(e893(5) >> 1)); break;
      case 0x05: e86F((uint8_t)(e893(2) >> 1)); break;
      case 0x09: e84F(bb(e893(4) << 1)); if (m[0xA0] != 0) e84F(bb(m[0xBC] + 0x10)); m[0xD8] = 0x40; break;
      case 0x0A: if (m[0xA4] != 1) e86F(bb(e893(3) << 1)); break;
      case 0x08: // finish line E8AF -> win (0x52=1) iff all laps done (0x57==0)
        m[0x3A] = 0x1D;
        if (m[A_LOOPS_REMAIN] == 0) { m[0x32] = 0x10; m[0xFD] = 2; m[A_RACE_OVER] = 1; m[0x3A] = 0; }
        break;
      case 0x12: if ((m[0xB0] | m[0x3E0] | m[0x3C]) == 0) m[0x3B6] = 8; break; // cooling zone E8D3
      case 0x0C: // mud E8FF -> sub_CE5C friction
        if (m[0xB0] == 0 && !(m[0xA4] != 0 && (m[0xA4] & 2) == 0) && m[0x94] != 0)
        { m[0x36C] = 1; applyFriction((m[0x5C] & 0x40) ? 5 : 6); }
        break;
      default: break;
    }
  }

  void handlerAngle(uint8_t typ)  // E7A3
  {
    uint8_t* m = _mem;
    if (m[0xB0] || m[0x98]) return;
    uint8_t nib = typ & 0x0F;
    if (m[0xA4] == 1) return;
    m[0xAC] = nib;
    if (m[0x58] != 3) { int idx = (int)nib - 2; if (idx >= 0 && idx < 10) m[0xD4] = kE6AD[idx]; }
  }

  void sectionMachinery()  // sub_E73B (player)
  {
    uint8_t* m = _mem;
    uint8_t lane = m[0x360];
    m[0xC8] = bb(m[0xC8] + m[0x60]);  // uses prev frame's 0x60 (posX not yet updated this frame)
    if (m[0x58] == 0)
    {
      uint32_t col = _cum;
      uint8_t c0 = (col < kTrackColumns) ? kTrack[lane][col] : 0x3B;
      m[0xC0] = c0;
      int t = (int)c0 - 0x40;
      if (t < 0) return;
      t >>= 2;
      if (t >= 0x16) return;
      m[0x58] = bb(t + 1); m[0xC4] = 0; m[0xC8] = bb(m[0x64] - 1);
    }
    for (;;)
    {
      int blk = m[0x58] - 1;
      const Feat* feats = (blk >= 0 && blk < (int)kNumSections) ? kSections[blk] : nullptr;
      int len = (blk >= 0 && blk < (int)kNumSections) ? kSectionLen[blk] : 0;
      int cur = m[0xC4] / 2;
      if (!feats || cur >= len)
      {
        m[0x58] = 0; m[0xD4] = 0;                       // terminator (bra_E7D0)
        if (m[0xA0] == 0 && m[0xA4] != 1) { m[0xBC] = 0; if (m[0xA4] == 2) m[0xA4] = bb(m[0xA4] + 1); }
        m[0x36C] = 0;
        return;
      }
      uint8_t pos = feats[cur].pos, typ = feats[cur].type;
      if (pos > m[0xC8]) return;
      if (typ & 0x80) handlerAngle(typ);
      else if (typ & 0x40) { m[0xCC] = typ & 0x0F; m[0xD0] = pos; }
      else dispatch(typ);
      m[0xC4] = bb(m[0xC4] + 2);
    }
  }

  void setRestAngle()  // sub_DCA0
  {
    uint8_t* m = _mem; uint8_t y = m[0xD4];
    if (m[0x52] == 1 && (m[0x98] | m[0x9C] | m[0x58]) == 0) m[0x368] = 0x0A;
    else m[0x368] = (y < 7) ? kD88B[y] : 0;
    if (m[0xA4] == 1) m[0x368] = 6;
  }

  void angleUpdate()  // loc_CDBD tail
  {
    uint8_t* m = _mem;
    if (m[0x98]) return;
    bool air = m[0xB0] != 0, active;
    if (air) active = true;
    else if ((m[0x58] | m[0x52]) != 0) active = false;
    else active = (m[0x94] != 0 || m[0x90] >= 0xA0);
    uint8_t a = m[0x5C] & 3;
    if (active && a != 0) tilt(a);
    else if (!air && m[0x368] != m[0xAC]) groundRamp();
  }
  void groundRamp()  // loc_DCC7
  {
    uint8_t* m = _mem;
    if (m[0x2A] != 0) return;
    m[0x2A] = 5;
    int ac = m[0xAC], tgt = m[0x368];
    if (ac == tgt) return;
    m[0xAC] = bb(ac + (ac < tgt ? 1 : -1));
  }
  void tilt(uint8_t a)  // loc_CE83
  {
    uint8_t* m = _mem;
    if (m[0x26] != 0) return;
    int y = m[0xB0] >> 1;
    m[0x26] = kC0D4[y];
    int ac = m[0xAC];
    if (a & 1)
    {
      int tgt = kC0C8[y];
      if (ac == tgt) return;
      m[0xAC] = bb(ac + (ac < tgt ? 1 : -1));
    }
    else
    {
      m[0x388] = m[0x388] & 2;
      if (ac < kC0CA[y]) { m[0xAC] = bb(ac + 1); return; }
      if ((m[0x5C] & 0xC0) == 0) return;
      if (m[0xB0] != 0) return;
      m[0xAC] = bb(ac + 1); m[0x26] = 0x0D;
      if (m[0xAC] >= 0x0D) { m[0x98] = 1; m[0x26] = 0x1A; }
    }
  }

  // sub_CDEE (player path, asm CDEE-CE28): the gate the ground speed step consults to decide friction
  // vs acceleration. Returns true when the step must BRAKE (the routine's nonzero return). Driven by
  // terrain (0xC0==0xE4), the ground-contact state (0xA4/0x70), and bike height 0xB8, latched through
  // 0x36C. The height gate (0xB8 >= 0x38, or < 0x08) is the wheelie/lean brake: the reference TAS keeps
  // its velX-update-frame heights below 0x38, but a search path bounces 0xB8 across it (Down flips velZ),
  // so omitting this let the search harvest phantom speed. Also maintains the 0x36C latch (CE0A/CE1D).
  bool speedFrictionGate()
  {
    uint8_t* m = _mem;
    // bra_CE1D path (latch-gated): reached only when NONE of the CE0A conditions hold and the bike
    // height is in [0x08, 0x38). Terrain 0xE4 / contact-3+posZ1 / height>=0x38 / height<0x08 all fall
    // through to the CE0A latch-arm below (== brake for the player).
    if (m[0xC0] != 0xE4 && !(m[0xA4] == 3 && m[0x70] >= 3) && m[0xB8] >= 0x08 && m[0xB8] < 0x38)
    {
      if (m[0x36C] == 1) return true;                      // CE1F-CE22: latched -> brake
      m[0x36C] = 0;                                        // CE24-CE25: clear latch -> accelerate
      return false;
    }
    if (m[0x36C] != 1) m[0x36C] = 2;                       // CE0A-CE12: arm latch (2) unless already 1
    m[0xFD] = 4;                                           // CE18-CE1A (player, X==0)
    return true;
  }

  // ---- speed (sub_CD59) ----
  void applyAccel(int y)
  {
    uint8_t* m = _mem;
    int lo = m[0x90] + kAccel[y]; m[0x90] = bb(lo); if (lo > 0xFF) m[0x94] = bb(m[0x94] + 1);
    if (m[0x94] > kCapHi[y] || (m[0x94] == kCapHi[y] && m[0x90] >= kCapLo[y])) { m[0x90] = kCapLo[y]; m[0x94] = kCapHi[y]; }
  }
  void applyFriction(int y)
  {
    uint8_t* m = _mem;
    if (m[0x94] == 0 && m[0x90] < kFricFloor[(m[0xB0] >> 1) & 1]) return;
    int lo = (int)m[0x90] - kFriction[y]; m[0x90] = bb(lo);
    if (lo < 0) { if (m[0x94] != 0) m[0x94] = bb(m[0x94] - 1); else m[0x90] = 0; } // clamp to 0 (sub_CE58 bra_CE80)
  }
  void stepSpeed()
  {
    uint8_t* m = _mem;
    if (m[0x4C] != 0) return;
    if (m[0x374] != 0 || m[0x98] != 0) { applyFriction(0); return; }
    if (m[0xB0] != 0) { int lean = m[0x5C] & 3; applyFriction(lean == 0 ? 4 : lean + 1); return; }
    uint8_t ab = m[0x5C] & 0xC0;
    if (ab == 0) { applyFriction(0); return; }
    // sub_CDEE friction gate (asm CD8C-CD93): with the race still running, a height/terrain/contact
    // condition forces friction(0) over acceleration when velX has a high byte. The gate is consulted
    // unconditionally here (it maintains the 0x36C latch); only the brake itself is velX_hi-gated.
    if (m[0x52] == 0)
    {
      bool brake = speedFrictionGate();
      if (brake && m[0x94] != 0) { applyFriction(0); return; }
    }
    int y = (m[0x5C] & 0x80) ? 0 : 1;
    uint8_t vhi = m[0x94], vlo = m[0x90];
    if (vhi > kCapHi[y] || (vhi == kCapHi[y] && vlo > kCapLo[y])) applyFriction(y + 1);
    else if (vhi == kCapHi[y] && vlo == kCapLo[y]) {}
    else applyAccel(y);
  }

  void stepTemp()  // sub_E359 temperature
  {
    uint8_t* m = _mem;
    uint8_t ab = m[0x5C] & 0xC0; int y = 0;
    if (ab != 0)
    {
      uint8_t a = (m[0x4F] != 0) ? ab : 0x80;
      int c = (a >> 7) & 1; a = bb(a << 1);
      for (int i = 0; i < 2; i++) { int nc = (a >> 7) & 1; a = bb((a << 1) | c); c = nc; }
      y = a;
    }
    uint8_t equ = kTempEquil[y];
    if (m[0x3B6] < equ) { int s = m[0x3B5] + kTempRate[y]; m[0x3B5] = bb(s); if (s > 0xFF) m[0x3B6] = bb(m[0x3B6] + 1); }
    else if (m[0x3B6] > equ) { int s = (int)m[0x3B5] - 0x0B; m[0x3B5] = bb(s); if (s < 0 && m[0x3B6] != 0) m[0x3B6] = bb(m[0x3B6] - 1); }
  }

  // ---- timers + overheat stall ----
  void decTimers()  // sub_D310
  {
    uint8_t* m = _mem;
    m[0x20] = bb(m[0x20] - 1);
    int hi = (m[0x20] & 0x80) ? 0x3D : 0x2F;
    if (m[0x20] & 0x80) m[0x20] = 0x0A;
    for (int a = 0x21; a <= hi; a++) if (m[a]) m[a] = bb(m[a] - 1);
  }
  void stallProcess()  // sub_DD8D
  {
    uint8_t* m = _mem;
    if ((m[0x3C] | m[0x3E0]) == 0) return;
    uint8_t c = m[0x3C];
    if (c == 8) { m[0x3E0] = 0; m[0x3B6] = 5; m[0x9C] = 5; m[0x374] = 5; }
    else if (c < 8) { /* sprites only */ }
    else if ((m[0x94] | m[0x98] | m[0x9C]) == 0)
    {
      if (m[0x58] != 0) m[0x90] = 0xC0;
      else { m[0x90] = 0; m[0xDC] = (m[0xB8] == 0x39) ? 0 : 1; } // tbl_D8C4[0]=0x39
    }
  }

  // ---- lane (sub_E96C, player X=0) ----
  void flipVelZ() { uint8_t v = _mem[0xDC] ? _mem[0xDC] : 0xFF; _mem[0xDC] = v ^ 0xFE; }
  bool isBoundary(uint8_t b8) const { for (int i = 0; i < 4; i++) if (b8 == kE53D[i]) return true; return false; }
  void stepLane()
  {
    uint8_t* m = _mem;
    if (m[0x58] != 0x14 && m[0x58] != 0x15)
      m[0xB8] = bb((int)m[0xB8] + (m[0xDC] < 0x80 ? m[0xDC] : m[0xDC] - 0x100));
    if (isBoundary(m[0xB8])) m[0xDC] = 0;
    else
    {
      uint8_t a4 = m[0xA4];
      if (a4 == 0) {}
      else if (a4 == 1) { if (m[0xB8] < 0x20) { m[0xA4] = 4; m[0xBC] = 0; } }
      else if (a4 == 3) {}
      else { if (m[0xB8] >= 0x20) m[0xB8] = bb(m[0xB8] - 1); }
      if (m[0xB8] < 8) { if (m[0x9C] == 0) m[0xB8] = 7; else if (m[0xB8] < 2) m[0xB8] = 1; flipVelZ(); }
      else if (m[0xB8] >= 0x3A) { m[0xB8] = 0x39; if ((m[0x9C] | m[0x3E0]) == 0) flipVelZ(); }
    }
    if (m[0x4F] != 0 && (m[0x98] | m[0x3E0] | m[0x3F7]) == 0)
    {
      bool ok = true;
      if (m[0xB0] != 0) { if (m[0x388] == 2) m[0x388] = bb(m[0x388] + 1); else ok = false; }
      if (ok && (m[0x9C] == 0 || m[0x9C] == 5))
      { uint8_t ud = m[0x5C] & 0x0C; if (ud) m[0xDC] = (m[0x5C] & 0x04) ? 0xFF : 0x01; }
    }
  }

  // ---- landing / arc (sub_DC1A / sub_DCF2) ----
  void landing()  // sub_DC1A
  {
    uint8_t* m = _mem;
    if (m[0xB0] != 2) return;
    uint8_t groundY = bb(0xA0 - m[0xBC] - m[0xB8]);
    if (!(m[0x8C] > groundY && m[0x8C] < 0xA8 && m[0x364] != 0)) return;
    m[0xB0] = 0; m[0x364] = 0; uint8_t oldJ = m[0x388]; m[0x388] = 0; m[0x8C] = bb(groundY - 1);
    if (oldJ != 0) return;                              // post-bounce clean landing
    int y = m[0xD4] + (m[0x94] < 2 ? 8 : 0);
    if (m[0xAC] < kD86C[y] || m[0xAC] >= kD87C[y]) { m[0x98] = 0xFF; return; }
    if (m[0xAC] == m[0x368] || (m[0xD4] | m[0xCC]) != 0) return; // clean
    m[0x374] = 4; m[0x388] = 2; m[0xB0] = 1;            // wobble -> bounce relaunch
  }
  void arc()  // sub_DCF2
  {
    uint8_t* m = _mem;
    if (m[0xB0] == 0) return;
    if (m[0xB0] == 2)
    {
      int lean0 = m[0x5C] & 1;
      if (lean0 == 1 && m[0x4C] == 0) return;
      arcStep(lean0);
      if (m[0xCC] == 0) m[0x364] = 1;
    }
    else takeoff(0);                                   // airMode==1: bounce re-launch (loc_DCFA)
  }
  void wobbleDec()  // sub_DCDE (decrement only; A/B clear handled in advance())
  {
    uint8_t* m = _mem;
    if (m[0x374] == 0) return;
    m[0x374] = bb(m[0x374] - 1);
  }

  // ---- posX (sub_DA58 + sub_DBFE) ----
  void stepPosX1()
  {
    uint8_t* m = _mem;
    m[0x60] = m[0x94];
    int sp = m[0x394] + m[0x90]; m[0x394] = bb(sp); if (sp > 0xFF) m[0x60] = bb(m[0x60] + 1);
  }
  void stepPosX2()
  {
    uint8_t* m = _mem;
    uint8_t a = m[0x60]; int c = a & 1; a = a >> 1;
    for (int i = 0; i < 3; i++) { int nc = a & 1; a = bb((c << 7) | (a >> 1)); c = nc; }
    int ss = a + m[0x3BF]; m[0x3BF] = bb(ss); int cin = ss > 0xFF;
    int px = m[0x60] + m[0x50] + cin; m[0x50] = bb(px); if (px > 0xFF) m[0x4E] ^= 1;
  }

  void colCounter()  // sub_E70B
  {
    uint8_t* m = _mem; uint8_t v = m[0x60];
    if (v != 0)
    {
      uint8_t bd = bb(m[0x64] - v);
      if (bd == 0 || (bd & 0x80)) { m[0x64] = bb(bd + 8); m[0xE0] = (m[0xE0] + 1) & 0x3F; _cum++; }
      else m[0x64] = bd;
    }
  }

  void updateDerived()
  {
    if (!_firstPostHook && _prevBlockX != _mem[A_CURR_BLOCK_X]) _blockXTransitions++;
    _prevBlockX = _mem[A_CURR_BLOCK_X]; _firstPostHook = false;
    _bikePosX = (float)_blockXTransitions * 256.0f + (float)_mem[A_SCROLL_X] + (float)_mem[A_POSX_SUB] / 256.0f;
  }

  uint8_t  _mem[LRAM_SIZE];
  uint32_t _cum = 0;
  uint8_t  _blockXTransitions = 0;
  uint8_t  _prevBlockX = 0;
  bool     _firstPostHook = true;
  uint32_t _currentStep = 0;
  float    _bikePosX = 0.0f;
};

} // namespace excitebike
