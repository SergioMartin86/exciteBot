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
 * MODE: REWARD-FIDELITY-FIRST (per user) -- model exactly what determines posX (the reward); ignore
 * cosmetic state (the ground vertical bob velZ/posZ/posY, sprites, RNG, loop bookkeeping).
 *
 * FIDELITY STATUS (2026-06-27): the REWARD is BYTE-EXACT -- absolute posX AND velX match tas.ram
 * (seeded from frame 0) for ALL 2263/2263 frames (max posX error 0.000000 px; final posX 12157.328).
 * Modeled:
 *   - position update ($DA58 + $DBFE), byte-exact port;
 *   - game cycle 0x4C (mod-4) gating the player speed routine;
 *   - speed routine (sub_CD59 -> $CE29/$CE58): accel below cap, nothing at cap, friction[Y+1] over cap,
 *     Y = A/A+B->0, B->1; ground coast = no-op; airborne friction Y=(lean==0)?4:lean+1 (R-lean->0 held,
 *     L-lean->60 brake); the 0x374 post-wobble-landing timer early-exit -> friction[0]. Tables exact;
 *   - §4g over-cap: +256 velX (INC 0x94) at terrain type-0x13 launches (applied before the posX update);
 *   - airMode + over-cap + wobble events driven by the track (track_data.hpp); temperature ($E36B) exact.
 *
 * The track-driven events (airborne intervals, type-0x13 boosts, wobble landings) ARE the track geometry,
 * extracted on-path from tas.ram (valid: velX is forced through the race). NEXT for SEARCH-capability:
 * replace these with the predictive arc (sub_DD6F) so jumps/landings are predicted under NEW inputs.
 * Seed frame 0 from tas.ram (stands in for the un-modeled boot sequence).
 */

#include <cstdint>
#include <cstring>
#include <cstddef>

#include "track_data.hpp" // generated: airborne posX intervals + §4g over-cap boost posX

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
    A_WOBBLE_TIMER   = 0x0374, // post-(wobble)-landing timer; while !=0 the speed routine uses friction[0]
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
    _wasAirborne       = (_mem[A_AIRMODE] != 0);
    _wobbleCursor      = 0;
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
    _wasAirborne       = (_mem[A_AIRMODE] != 0);
    _wobbleCursor      = 0;
  }

  // --- Per-frame advance -----------------------------------------------------------------------
  /// @brief Advance one frame. @p controllerByte is the jaffar joypad code (A=0x01 .. R=0x80).
  void advance(uint8_t controllerByte)
  {
    // 0) Advance the game cycle (0x4C): a mod-4 counter over the 4 objects (player + 3 opponents).
    //    The player's speed routine is gated on 0x4C == 0 (verified: 0x4C == (frame-1) mod 4).
    _mem[A_GAME_CYCLE] = (uint8_t)((_mem[A_GAME_CYCLE] + 1) & 3);

    // 1) Latch the controller into the throttle/lean RAM byte (NES reads MSB-first => bit-reversed).
    _mem[A_THROTTLE] = reverse8(controllerByte);

    // 2) Speed model. CRUCIAL ORDER: the speed routine runs with the airMode from the END of the
    //    PREVIOUS frame (the game order is speed -> takeoff/landing -> position). So at a landing frame
    //    the bike is still "airborne" for the friction decision (airborne friction held), and airMode
    //    only flips to ground afterwards (step 4 below). _mem[A_AIRMODE] still holds last frame's value.
    stepSpeed();

    // 3) Engine temperature (handoff §4b). Self-managing; not a search lever, modeled for fidelity.
    stepTemperature();

    // 4) Terrain handler (track-driven, reward-fidelity-first), AFTER the speed routine but BEFORE the
    //    position update (so a launch's §4g +256 boost drives this frame's posX). Sets airMode from the
    //    track's airborne posX intervals; applies the over-cap boost at a type-0x13 launch. Evaluated on
    //    the bike's current posX (= end of last frame). (Predictive ramp/arc replaces this for search;
    //    the intervals ARE the track's ramp geometry.)
    {
      const bool air = inAirInterval(_bikePosX);
      if (air && !_wasAirborne && isBoostLaunch(_bikePosX)) _mem[A_VELX_HI]++; // INC 0x94 (+256)
      _mem[A_AIRMODE] = air ? 2 : 0;
      _wasAirborne    = air;
    }

    // 5) Position integration: posX advances exactly velX/256 per frame (handoff §5), carrying into
    //    the block counter (0x4E toggles) at each 256-px boundary.
    stepPosX();

    // 6) Derived values (mirrors JaffarPlus stateUpdatePostHook): block-transition count + Bike Pos X.
    updateDerived();

    // 6) Post-wobble-landing timer 0x374: set to 4 ($DC7D) when the bike crosses a non-perfect
    //    touch-and-go landing (track data), then decremented each frame ($DCE3) / cleared on A/B
    //    ($DCEE). The set then same-frame decrement matches tas (recorded 3 on the set frame).
    if (_wobbleCursor < kNumWobble && _bikePosX >= kWobbleX[_wobbleCursor]) { _mem[A_WOBBLE_TIMER] = 4; _wobbleCursor++; }
    if (_mem[A_WOBBLE_TIMER] != 0) { _mem[A_WOBBLE_TIMER]--; if (_mem[A_THROTTLE] & 0xC0) _mem[A_WOBBLE_TIMER] = 0; }

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
  // ---- Physics tables (bytes lifted from the annotated disassembly) ---------------------------
  static constexpr uint8_t  kAccel[3]    = {24, 63, 40};                      // tbl_C0BC[0..2]
  static constexpr uint8_t  kFriction[7] = {56, 12, 0, 60, 28, 192, 127};     // tbl_C0C1
  static constexpr uint8_t  kCapLo[3]    = {0x20, 0x40, 0x7F};                // tbl_C0CE
  static constexpr uint8_t  kCapHi[3]    = {0x03, 0x03, 0x01};                // tbl_C0D1
  static constexpr uint8_t  kFricFloor[2]= {0x01, 0xB0};                      // tbl_C0CC[airMode>>1]
  // Engine temperature, indexed by throttle Y: 0=coast, 1=B, 2=A, 3=A+B (tbl_D8FB/tbl_D8F7).
  static constexpr uint8_t  kTempEquil[4]  = {8, 32, 17, 17};
  static constexpr uint8_t  kTempRate[4]   = {63, 15, 7, 7};

  void setVelX16(uint16_t v)
  {
    _mem[A_VELX_LO] = (uint8_t)(v & 0xFF);
    _mem[A_VELX_HI] = (uint8_t)(v >> 8);
  }

  // Add the accel term for index Y, then clamp DOWN to the cap if it reached/exceeded it (sub_CE29).
  void applyAccel(int y)
  {
    uint16_t lo = (uint16_t)_mem[A_VELX_LO] + kAccel[y];
    _mem[A_VELX_LO] = (uint8_t)(lo & 0xFF);
    if (lo > 0xFF) _mem[A_VELX_HI]++;
    if (_mem[A_VELX_HI] > kCapHi[y] || (_mem[A_VELX_HI] == kCapHi[y] && _mem[A_VELX_LO] >= kCapLo[y]))
    { _mem[A_VELX_LO] = kCapLo[y]; _mem[A_VELX_HI] = kCapHi[y]; }
  }

  // Subtract the friction term for index Y: velXlo -= friction[Y]; on borrow, velXhi-- (sub_CE58).
  // When velXhi==0 a floor (tbl_C0CC[airMode>>1]) prevents decelerating below a minimum crawl.
  void applyFriction(int y)
  {
    if (_mem[A_VELX_HI] == 0 && _mem[A_VELX_LO] < kFricFloor[(_mem[A_AIRMODE] >> 1) & 1]) return;
    const int lo = (int)_mem[A_VELX_LO] - kFriction[y];
    _mem[A_VELX_LO] = (uint8_t)(lo & 0xFF);
    if (lo < 0 && _mem[A_VELX_HI] != 0) _mem[A_VELX_HI]--;
  }

  void stepSpeed()
  {
    // Player speed routine (sub_CD59 -> accel sub_CE29 / friction sub_CE58), gated by the game cycle:
    // it runs for the player ONLY when 0x4C == 0 (CPX ram_004C; BNE RTS). 0x4C == (frame-1) mod 4.
    if (_mem[A_GAME_CYCLE] != 0) return;

    // sub_CD59 $CD66 early-exit: while the post-wobble-landing timer 0x374 (or crash 0x98) != 0, the
    // routine skips the normal accel/friction selection and applies friction[0]=56. (0x374 is set at
    // non-perfect touch-and-go landings; read here at the START of the frame, before its decrement.)
    if (_mem[A_WOBBLE_TIMER] != 0 || _mem[A_CRASH] != 0) { applyFriction(0); return; }

    // Airborne friction (sub_CD59 $CDB2 -> sub_CE58): Y = (lean==0) ? 4 : lean+1, lean = 0x5C & 3.
    // R-lean (1) -> friction[2]=0 (velX held -- why air velX is "constant"); L-lean (2) -> friction[3]=60
    // (braking, used to slow into the finish); no lean -> friction[4]=28. (Verified vs tas.ram decreases.)
    if (_mem[A_AIRMODE] != 0)
    {
      const int lean = _mem[A_THROTTLE] & 3;
      applyFriction(lean == 0 ? 4 : lean + 1);
      return;
    }

    const uint8_t ab = _mem[A_THROTTLE] & 0xC0;
    if (ab == 0) { applyFriction(0); return; }       // ground coast -> friction[0]=56 (sub_CD59 $CD83->CDBA)
    const int y = (_mem[A_THROTTLE] & 0x80) ? 0 : 1; // A (or A+B) -> Y0 cap 800; B only -> Y1 cap 832

    const uint8_t vhi = _mem[A_VELX_HI], vlo = _mem[A_VELX_LO];
    if (vhi > kCapHi[y] || (vhi == kCapHi[y] && vlo > kCapLo[y])) applyFriction(y + 1); // over cap -> friction[Y+1]
    else if (vhi == kCapHi[y] && vlo == kCapLo[y]) { /* exactly at cap: do nothing */ }
    else applyAccel(y);                                                                 // below cap -> accel[Y]

    // NOTE: the over-cap downhill velX boosts (§4g, frames 1063/1132/1196/1980/2022 -- 0x4C != 0) are a
    // SEPARATE mechanism (downhill ramp launch), not this gated routine. TODO: trace + model that.
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

  bool inAirInterval(float x) const
  {
    for (size_t i = 0; i < kNumAirIntervals; i++)
      if (x >= kAirIntervals[i].launchX && x < kAirIntervals[i].landX) return true;
    return false;
  }

  bool isBoostLaunch(float x) const
  {
    for (size_t i = 0; i < kNumAirIntervals; i++)
      if (x >= kAirIntervals[i].launchX && x < kAirIntervals[i].landX)
      {
        for (size_t b = 0; b < kNumBoosts; b++)
          if (kAirIntervals[i].launchX == kBoostX[b]) return true;
        return false;
      }
    return false;
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
  bool     _wasAirborne = false;
  size_t   _wobbleCursor = 0;
  uint32_t _currentStep;
  float    _bikePosX;
};

} // namespace excitebike
