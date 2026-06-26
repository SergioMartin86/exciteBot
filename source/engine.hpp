#pragma once
/**
 * @file engine.hpp
 * @brief Native, from-scratch re-implementation of the Excitebike (NES) bike physics.
 *
 * This is the "emulator replacement" at the heart of exciteBot: instead of interpreting the 6502 ROM,
 * it models the game's low work-RAM ($0000-$07FF) directly and advances the modeled bike physics one
 * frame at a time. The public surface mirrors what a JaffarPlus `Game` needs:
 *   - advance(controllerByte) : step one frame given the NES controller byte
 *   - reward()  = Bike Pos X (sub-pixel)         -- the ONLY reward
 *   - isWin()   = Race Over Flag (RAM 0x52) > 0   -- full-race completion
 *   - isFail()  = Crash Flag     (RAM 0x98) > 0
 *   - serialize()/deserialize()                   -- tiny state
 *   - lram()    = the 2048-byte modeled RAM image -- for the byte-for-byte tas.ram fidelity gate
 *
 * FIDELITY STATUS (2026-06-26): validated EXACT against tas.ram (seeded from frame 0) for frames 0..66
 * (the opening flat cruise) on the modeled physics addresses: velX, posX (0x50/0x394/0x60/0x3BF/0x4E),
 * and engine temperature (0x3B6/0x3B5 -- the temperature routine alone matches all 2263 frames). The
 * position update ($DA58 + $DBFE) and temperature routine ($E36B) are byte-exact ports of the 6502 code.
 *
 * NEXT FRONTIER: at frame 67 the bike reaches the first terrain geometry (velZ/posZ/posY begin moving on
 * the approach to a hill/ramp). Going further needs the TRACK TERRAIN PROFILE (§8 -- the make-or-break
 * blocker) plus vertical/airborne integration, downhill over-cap (§4g), takeoff/arc/landing (§4c-§4e) --
 * all stubbed/TODO. Seed frame 0 from tas.ram (Engine::seedFromRam) as a stand-in for the un-modeled
 * boot sequence. Deferred (track-layout bookkeeping, not core physics): loop counters 0x57/0xED/0x3A4
 * and the Race Over win flag 0x52.
 */

#include <cstdint>
#include <cstring>
#include <cstddef>

namespace excitebike
{

/// @brief Reverse the bit order of a byte. The NES controller is read MSB-first, so the jaffar joypad
/// code (A=0x01, B=0x02, Select=0x04, Start=0x08, Up=0x10, Down=0x20, Left=0x40, Right=0x80) lands in
/// RAM 0x5C bit-reversed: A=0x80, B=0x40, Select=0x20, Start=0x10, Up=0x08, Down=0x04, Left=0x02,
/// Right=0x01 -- which is exactly the handoff's "0x5C bit7=A, bit6=B, low-2-bits=lean(L/R)".
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
  static constexpr size_t LRAM_SIZE = 0x800; // 2048 bytes of NES low work-RAM

  // --- Named RAM addresses (from the handoff variable map) -------------------------------------
  enum Addr : uint16_t
  {
    A_GAME_CYCLE     = 0x004C, // game cycle / player index
    A_CURR_BLOCK_X   = 0x004E, // current block X (nametable); toggles 0/1 at each 256-px block
    A_RACE_STARTED   = 0x004F,
    A_SCROLL_X       = 0x0050, // posX low pixel (within block)
    A_RACE_OVER      = 0x0052, // Race Over Flag -- the WIN (0->1 at race completion)
    A_LOOPS_REMAIN   = 0x0057,
    A_THROTTLE       = 0x005C, // buttons: bit7=A(slow), bit6=B(turbo); low-2-bits=lean
    A_SCROLL_INC     = 0x0060,
    A_POSZ1          = 0x0070,
    A_POSY           = 0x008C,
    A_VELX_LO        = 0x0090,
    A_VELX_HI        = 0x0094, // velX16 = RAM[0x94]*256 + RAM[0x90]
    A_CRASH          = 0x0098, // Crash Flag -- the FAIL
    A_ANGLE          = 0x00AC,
    A_AIRMODE        = 0x00B0, // 0=ground, 2=airborne, 1=wobble/recovery
    A_POSZ2          = 0x00B8,
    A_CLIMB_REMAIN   = 0x00BC,
    A_SLOPE          = 0x00D4,
    A_VELZ           = 0x00DC,
    A_INTRALOOP      = 0x00ED,
    A_POSX_SUB       = 0x0394, // posX sub-pixel (accumulates velX low byte per frame)
    A_POSX_SUBSUB    = 0x03BF, // posX sub-sub-pixel accumulator (sub_DBFE; carries an extra px into 0x50)
    A_CURRENT_LOOP   = 0x03A4,
    A_TEMP_SUB       = 0x03B5, // engine-temperature subcounter
    A_TEMP           = 0x03B6, // engine temperature
  };

  // --- Construction / seeding ------------------------------------------------------------------
  Engine() { reset(); }

  /// @brief Reset to a documented flat race-start baseline. Does NOT fully reproduce the post-boot
  /// frame-0 RAM (the boot/countdown is not modeled) -- use seedFromRam() for an exact frame-0 seed.
  void reset()
  {
    std::memset(_mem, 0, sizeof(_mem));
    _mem[A_ANGLE]   = 6; // flat terrain forces angle 6
    _mem[A_AIRMODE] = 0; // on the ground
    _blockXTransitions = 0;
    _prevBlockX        = _mem[A_CURR_BLOCK_X];
    _firstPostHook     = true;
    _currentStep       = 0;
    updateDerived();
  }

  /// @brief Seed the full 2048-byte RAM image from a real frame-0 snapshot (e.g. the first frame of
  /// tas.ram). Stands in for the un-modeled boot sequence so validation can start at the race proper.
  void seedFromRam(const uint8_t* ram2048, uint8_t initialBlockTransitions = 0)
  {
    std::memcpy(_mem, ram2048, LRAM_SIZE);
    _blockXTransitions = initialBlockTransitions;
    _prevBlockX        = _mem[A_CURR_BLOCK_X];
    _firstPostHook     = true; // first updateDerived just latches prevBlockX (no spurious transition)
    _currentStep       = 0;
    updateDerived();
  }

  // --- Per-frame advance -----------------------------------------------------------------------
  /// @brief Advance one frame. @p controllerByte is the jaffar joypad code (A=0x01 .. R=0x80).
  void advance(uint8_t controllerByte)
  {
    // 1) Latch the controller into the throttle/lean RAM byte (NES reads MSB-first => bit-reversed).
    _mem[A_THROTTLE] = reverse8(controllerByte);

    // 2) Speed model (handoff §4a, §4d). Ground: B/A accel toward the hard caps. Air: velX is constant
    //    (player air-friction ~0, validated 99.7%). Coast/downhill over-cap is NOT yet traced.
    stepSpeed();

    // 3) Engine temperature (handoff §4b). Self-managing; not a search lever, modeled for fidelity.
    stepTemperature();

    // 4) Position integration: posX advances exactly velX/256 per frame (handoff §5), carrying into
    //    the block counter (0x4E toggles) at each 256-px boundary.
    stepPosX();

    // TODO(handoff §4c-§4f, §8): terrain following (forces angle/slope from the track profile),
    // takeoff (sets airMode/launch velZ), airborne arc integration, and landing/wobble resolve.
    // These require the track terrain profile (the §8 extraction blocker) before they can run.

    // 5) Derived values (mirrors JaffarPlus stateUpdatePostHook): block-transition count + Bike Pos X.
    updateDerived();
    _currentStep++;
  }

  // --- Outputs ---------------------------------------------------------------------------------
  uint16_t velX16() const { return (uint16_t)((_mem[A_VELX_HI] << 8) | _mem[A_VELX_LO]); }
  float    reward() const { return _bikePosX; }                 // EXCLUSIVELY posX
  bool     isWin()  const { return _mem[A_RACE_OVER] != 0; }    // Race Over Flag > 0
  bool     isFail() const { return _mem[A_CRASH] != 0; }        // Crash Flag > 0
  float    bikePosX() const { return _bikePosX; }
  uint8_t  blockXTransitions() const { return _blockXTransitions; }
  uint32_t currentStep() const { return _currentStep; }

  const uint8_t* lram() const { return _mem; }                  // the 2048-byte modeled RAM image
  uint8_t        ram(uint16_t a) const { return _mem[a]; }

  // --- Tiny serialization (the whole point: state is ~2KB+, vs the emulator's full machine) -----
  // The canonical state is the RAM image plus the few externally-maintained scalars.
  struct State
  {
    uint8_t  mem[LRAM_SIZE];
    uint8_t  blockXTransitions;
    uint8_t  prevBlockX;
    uint8_t  firstPostHook;
    uint32_t currentStep;
  };
  void serialize(State& s) const
  {
    std::memcpy(s.mem, _mem, LRAM_SIZE);
    s.blockXTransitions = _blockXTransitions;
    s.prevBlockX        = _prevBlockX;
    s.firstPostHook     = _firstPostHook ? 1 : 0;
    s.currentStep       = _currentStep;
  }
  void deserialize(const State& s)
  {
    std::memcpy(_mem, s.mem, LRAM_SIZE);
    _blockXTransitions = s.blockXTransitions;
    _prevBlockX        = s.prevBlockX;
    _firstPostHook     = s.firstPostHook != 0;
    _currentStep       = s.currentStep;
    updateDerived();
  }

private:
  // ---- Physics tables (handoff §4a/§4b; bytes lifted from the annotated disassembly) -----------
  // Ground speed, indexed by throttle Y: 0=A(slow), 1=B(turbo), 2=track-finished.
  static constexpr uint16_t kSpeedCap[3]   = {800, 832, 383}; // 0x0320, 0x0340, 0x017F
  static constexpr uint16_t kSpeedAccel[3] = {24, 63, 40};    // tbl_C0BC[0..2]
  // Engine temperature, indexed by throttle Y: 0=coast, 1=B, 2=A, 3=A+B (tbl_D8FB/tbl_D8F7).
  static constexpr uint8_t  kTempEquil[4]  = {8, 32, 17, 17};
  static constexpr uint8_t  kTempRate[4]   = {63, 15, 7, 7};

  void setVelX16(uint16_t v)
  {
    _mem[A_VELX_LO] = (uint8_t)(v & 0xFF);
    _mem[A_VELX_HI] = (uint8_t)(v >> 8);
  }

  void stepSpeed()
  {
    const uint8_t thr = _mem[A_THROTTLE];
    if (_mem[A_AIRMODE] != 0) return; // airborne: velX constant (air-friction ~0) -- §4d (validated)

    uint16_t v = velX16();
    if (thr & 0x80) // A held -> Y0
    {
      v = (uint16_t)(v + kSpeedAccel[0]);
      if (v > kSpeedCap[0]) v = kSpeedCap[0];
      setVelX16(v);
    }
    else if (thr & 0x40) // B held -> Y1 (the flat-cruise case: pins velX at the 832 cap)
    {
      v = (uint16_t)(v + kSpeedAccel[1]);
      if (v > kSpeedCap[1]) v = kSpeedCap[1];
      setVelX16(v);
    }
    else
    {
      // Coast: with no button the accel/clamp routine never runs, so on a downhill gravity pushes
      // velX past the flat cap to the slope's terminal velocity (832->1088->...->1544). The exact
      // arithmetic is slope-driven and NOT yet traced. TODO(handoff §4g) -- needs the terrain profile.
    }
  }

  void stepTemperature()
  {
    // Exact replication of the temperature routine at $E36B-$E3A7 (verified: 0 mismatches vs tas.ram).
    // Throttle index Y: coast=0, B=1, A=2, A+B=3 (from AND #$C0 then ASL/ROL/ROL, with race_started
    // forcing 0x80 when not yet started -- 0x4F is 1 throughout this race so that branch is inert here).
    const uint8_t ab = _mem[A_THROTTLE] & 0xC0;
    int           y  = 0;
    if (ab != 0)
    {
      uint8_t a = (_mem[A_RACE_STARTED] != 0) ? ab : 0x80;
      int c = (a >> 7) & 1; a = (uint8_t)(a << 1);          // ASL
      for (int i = 0; i < 2; i++) { const int nc = (a >> 7) & 1; a = (uint8_t)((a << 1) | c); c = nc; } // ROL x2
      y = a;
    }
    const uint8_t equ = kTempEquil[y];
    if (_mem[A_TEMP] < equ) // HEAT: subcounter += rate[Y]; on carry, temp++
    {
      const uint16_t s = (uint16_t)_mem[A_TEMP_SUB] + kTempRate[y];
      _mem[A_TEMP_SUB] = (uint8_t)(s & 0xFF);
      if (s > 0xFF) _mem[A_TEMP]++;
    }
    else if (_mem[A_TEMP] > equ) // COOL: subcounter -= 11 (fixed); on borrow, temp-- (if >0)
    {
      const int s = (int)_mem[A_TEMP_SUB] - 0x0B;
      _mem[A_TEMP_SUB] = (uint8_t)(s & 0xFF);
      if (s < 0 && _mem[A_TEMP] != 0) _mem[A_TEMP]--;
    }
    // (Overheat checks at $E3AA -- set stall timer 0x3C / obj 0x3E0 when temp >= 0x20 -- modeled later
    // if a search path ever approaches the redline; the reference TAS peaks at 29 and never stalls.)
  }

  void stepPosX()
  {
    // Exact replication of the game's position update (sub_DA58 @ $DA58 + sub_DBFE @ $DBFE), object 0.
    //
    // sub_DA58: the per-frame scroll increment 0x60 = velX_hi, plus a carry from accumulating velX_lo
    // into the sub-pixel byte 0x394:
    //     0x60   = velX_hi
    //     0x394 += velX_lo        (CLC -> no carry-in); if it overflows, INC 0x60
    _mem[A_SCROLL_INC] = _mem[A_VELX_HI];
    const uint16_t sp = (uint16_t)_mem[A_POSX_SUB] + _mem[A_VELX_LO];
    _mem[A_POSX_SUB]  = (uint8_t)(sp & 0xFF);
    if (sp > 0xFF) _mem[A_SCROLL_INC]++;

    // sub_DBFE: rotate 0x60 right by 4 through carry, accumulate into the sub-sub-pixel byte 0x3BF,
    // then 0x50 += 0x60 + (carry from THAT 0x3BF add -- there is no CLC before the ADC at $DC0D, which
    // is the source of the occasional "extra" pixel). A carry out of 0x50 toggles the block byte 0x4E.
    uint8_t a = _mem[A_SCROLL_INC];
    int     c = a & 1; a = (uint8_t)(a >> 1);              // LSR
    for (int i = 0; i < 3; i++)                            // ROR x3 (through carry)
    {
      const int nc = a & 1;
      a = (uint8_t)((c << 7) | (a >> 1));
      c = nc;
    }
    const uint16_t ss = (uint16_t)a + _mem[A_POSX_SUBSUB]; // CLC; ADC 0x3BF
    _mem[A_POSX_SUBSUB] = (uint8_t)(ss & 0xFF);
    const int carryIn = (ss > 0xFF) ? 1 : 0;
    const uint16_t px = (uint16_t)_mem[A_SCROLL_INC] + _mem[A_SCROLL_X] + carryIn; // ADC 0x50 (carry-in!)
    _mem[A_SCROLL_X] = (uint8_t)(px & 0xFF);
    if (px > 0xFF) _mem[A_CURR_BLOCK_X] ^= 1;             // block boundary -> toggle 0x4E
  }

  void updateDerived()
  {
    // Mirrors JaffarPlus ExciteBike::stateUpdatePostHook: count real block changes (skipping the very
    // first hook), then recompute the absolute, sub-pixel Bike Pos X = the reward.
    if (!_firstPostHook && _prevBlockX != _mem[A_CURR_BLOCK_X]) _blockXTransitions++;
    _prevBlockX    = _mem[A_CURR_BLOCK_X];
    _firstPostHook = false;
    _bikePosX = (float)_blockXTransitions * 256.0f + (float)_mem[A_SCROLL_X] + (float)_mem[A_POSX_SUB] / 256.0f;
  }

  uint8_t  _mem[LRAM_SIZE];
  uint8_t  _blockXTransitions;
  uint8_t  _prevBlockX;
  bool     _firstPostHook;
  uint32_t _currentStep;
  float    _bikePosX;
};

} // namespace excitebike
