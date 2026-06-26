#pragma once
/**
 * @file input.hpp
 * @brief Parse a JaffarPlus NES input string ("|..|UDLRSsBA|") into the controller-1 joypad byte.
 *
 * Format (controller-1 = Joypad, controller-2 = None, as ExciteBike's race01.jaffar uses):
 *   |<P><r>|<U><D><L><R><S><s><B><A>|
 * where the console field is Power/reset and the 8 joypad columns are Up Down Left Right Start
 * Select B A. Each column is its letter when pressed or '.' when not. The returned byte uses the
 * exact bit layout from quickerNES/inputParser.hpp::parseJoyPadInput:
 *   A=0x01  B=0x02  Select=0x04  Start=0x08  Up=0x10  Down=0x20  Left=0x40  Right=0x80
 * (the NES controller's serial read order). Engine::reverse8 maps this into RAM 0x5C.
 */

#include <cstdint>
#include <stdexcept>
#include <string>

namespace excitebike
{

/// @brief Parse the joypad code from a "|..|UDLRSsBA|" input line. Throws std::runtime_error on a
/// malformed string -- mirroring jaffar-player's "Could not decode input string" abort (so a stray
/// trailing blank line is caught the same way).
inline uint8_t parseJoypad(const std::string& s)
{
  auto bad = [&]() -> uint8_t { throw std::runtime_error("Could not decode input string: '" + s + "'"); };

  // Expect: '|' P r '|' U D L R S s B A '|'  == 13 characters.
  if (s.size() != 13) return bad();
  if (s[0] != '|' || s[3] != '|' || s[12] != '|') return bad();

  // Console field (Power, reset): accept '.', 'P', 'r' but ignore (ExciteBike never uses them).
  auto col = [&](size_t i, char on, uint8_t bit, uint8_t& code) {
    if (s[i] == on) code |= bit;
    else if (s[i] != '.') bad();
  };

  uint8_t code = 0;
  col(4,  'U', 0x10, code);
  col(5,  'D', 0x20, code);
  col(6,  'L', 0x40, code);
  col(7,  'R', 0x80, code);
  col(8,  'S', 0x08, code);
  col(9,  's', 0x04, code);
  col(10, 'B', 0x02, code);
  col(11, 'A', 0x01, code);
  return code;
}

} // namespace excitebike
