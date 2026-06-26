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
 * FIDELITY STATUS (2026-06-26): this is the SCAFFOLD. The pieces validated in the reverse-engineering
 * (see docs/excitebike_physics_model_handoff.md) are implemented: input->0x5C decode, the B/A ground
 * speed caps, air-velX-is-constant, the engine-temperature model, and the posX/block-transition
 * integration. The pieces NOT yet traced are stubbed and marked `// TODO(handoff §...)`:
 *   - the track terrain profile (§8 -- the make-or-break blocker; slope/ramp/downhill per X)
 *   - the downhill over-cap acceleration (§4g)
 *   - takeoff / airborne-arc / landing resolve (§4c-§4e)
 * Until those land, advance() will NOT reproduce tas.ram beyond the flat-cruise segment. Seed the
 * frame-0 state from tas.ram (Engine::seedFromRam) as a stand-in for the un-modeled boot sequence.
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
    A_POSX_SUB       = 0x0394, // posX sub-pixel (accumulates velX/256 per frame)
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
  // Engine temperature, indexed by throttle Y: 0=coast, 1=B, 2=A.
  static constexpr uint8_t  kTempEquil[3]  = {8, 32, 17};     // tbl_D8FB
  static constexpr uint8_t  kTempRate[3]   = {63, 15, 7};     // tbl_D8F7

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
    const uint8_t thr = _mem[A_THROTTLE];
    const int     ty  = (thr & 0x80) ? 2 : (thr & 0x40) ? 1 : 0; // A / B / coast
    const uint16_t sub = (uint16_t)_mem[A_TEMP_SUB] + kTempRate[ty];
    _mem[A_TEMP_SUB] = (uint8_t)(sub & 0xFF);
    if (sub > 0xFF) // carry -> step the temperature one notch toward the equilibrium for this throttle
    {
      if (_mem[A_TEMP] < kTempEquil[ty]) _mem[A_TEMP]++;
      else if (_mem[A_TEMP] > kTempEquil[ty]) _mem[A_TEMP]--;
    }
  }

  void stepPosX()
  {
    // posX position within the current block is (scroll_X : posX_sub) as a 16-bit pixel.subpixel; it
    // advances by velX16 (units of 1/256 px) each frame. Overflow past 0xFFFF crosses a 256-px block
    // boundary -> toggle the block-X byte (which updateDerived() counts as a transition).
    const uint32_t pos = ((uint32_t)_mem[A_SCROLL_X] << 8) | _mem[A_POSX_SUB];
    const uint32_t np  = pos + velX16();
    if (np > 0xFFFF) _mem[A_CURR_BLOCK_X] ^= 1; // crossed a block boundary
    _mem[A_SCROLL_X]  = (uint8_t)((np >> 8) & 0xFF);
    _mem[A_POSX_SUB]  = (uint8_t)(np & 0xFF);
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
