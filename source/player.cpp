/**
 * @file player.cpp
 * @brief exciteBike-player: replays a JaffarPlus .sol movie through the native excitebike::Engine
 *        instead of the QuickerNES emulator, mirroring how `jaffar-player` reproduces a solution.
 *
 * It is the validation harness driver: feed it the reference movie and it produces the same kind of
 * outputs jaffar-player does -- a per-step RAM dump (--dumpRam, to diff byte-for-byte against tas.ram)
 * and a final-state report (--printFinalState: Current Step / Bike Pos X / Race Over Flag, plus the
 * first win/fail step). Win/fail rules are read from the .jaffar config (Game Configuration > Rules),
 * exactly like jaffar-player, using the same jaffarCommon JSON tooling.
 *
 * Frame-0 seeding: the real player reaches the race-start state by emulating the boot/countdown
 * (race01.initial.sol). The native engine does not model the boot, so seed frame 0 from a real RAM
 * snapshot with --initialRam <file> (e.g. the first 2048 bytes of tas.ram). Without it the engine
 * uses a documented flat baseline (Engine::reset).
 *
 * Usage (mirrors the documented reproduce command):
 *   exciteBike-player <config.jaffar> <movie.sol> [--printFinalState] [--dumpRam <out>]
 *                     [--initialRam <ram>] [--disableRender] [--unattended] [--exitOnEnd]
 */

#include "engine.hpp"
#include "input.hpp"

#include <jaffarCommon/file.hpp>
#include <jaffarCommon/json.hpp>
#include <jaffarCommon/string.hpp>

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

using excitebike::Engine;

namespace
{

// --- Config-driven win/fail rules (Game Configuration > Rules), like jaffar-player ----------------
struct Cond
{
  std::string prop;
  std::string op;
  double      val;
};
struct Rule
{
  std::vector<Cond> conds;
  bool              win  = false;
  bool              fail = false;
};

// Resolve a rule Property name to the live value in the engine state. Names match exciteBike.hpp's
// registerGameProperty() so the same config rules evaluate identically.
double propValue(const Engine& e, const std::string& p)
{
  if (p == "Bike Pos X") return e.bikePosX();
  if (p == "Bike Pos X1") return e.ram(0x50);
  if (p == "Bike Pos X2") return e.ram(0x394);
  if (p == "Race Over Flag") return e.ram(0x52);
  if (p == "Crash Flag") return e.ram(0x98);
  if (p == "Bike Air Mode") return e.ram(0xB0);
  if (p == "Bike Angle") return e.ram(0xAC);
  if (p == "Current Loop") return e.ram(0x3A4);
  if (p == "Loops Remaining") return e.ram(0x57);
  if (p == "Bike Engine Temp") return e.ram(0x3B6);
  throw std::runtime_error("Unsupported rule Property in this scaffold: '" + p + "'");
}

bool evalCond(const Engine& e, const Cond& c)
{
  const double a = propValue(e, c.prop);
  const double b = c.val;
  if (c.op == ">") return a > b;
  if (c.op == ">=") return a >= b;
  if (c.op == "<") return a < b;
  if (c.op == "<=") return a <= b;
  if (c.op == "==") return a == b;
  if (c.op == "!=") return a != b;
  throw std::runtime_error("Unsupported rule Op: '" + c.op + "'");
}

bool ruleHolds(const Engine& e, const Rule& r)
{
  for (const auto& c : r.conds)
    if (!evalCond(e, c)) return false;
  return true;
}

std::vector<Rule> parseRules(const jaffarCommon::json::object& gameConfig)
{
  std::vector<Rule> rules;
  if (!gameConfig.contains("Rules")) return rules;
  for (const auto& rj : gameConfig.at("Rules"))
  {
    Rule r;
    if (rj.contains("Conditions"))
      for (const auto& cj : rj.at("Conditions"))
        r.conds.push_back({cj.at("Property").get<std::string>(), cj.at("Op").get<std::string>(), cj.at("Value").get<double>()});
    if (rj.contains("Actions"))
      for (const auto& aj : rj.at("Actions"))
      {
        const auto t = aj.at("Type").get<std::string>();
        if (t == "Trigger Win") r.win = true;
        if (t == "Trigger Fail") r.fail = true;
      }
    rules.push_back(std::move(r));
  }
  return rules;
}

// state type after evaluating all rules at the current engine state
enum class StateType
{
  normal,
  win,
  fail
};
StateType evalState(const Engine& e, const std::vector<Rule>& rules)
{
  bool win = false, fail = false;
  for (const auto& r : rules)
    if (ruleHolds(e, r))
    {
      if (r.win) win = true;
      if (r.fail) fail = true;
    }
  if (fail) return StateType::fail; // fail takes precedence (crash-free play is enforced)
  if (win) return StateType::win;
  return StateType::normal;
}

// Read up to LRAM_SIZE bytes from a file (used to seed frame 0 from a real RAM snapshot, e.g. tas.ram).
bool readInitialRam(const std::string& path, uint8_t* out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  f.read(reinterpret_cast<char*>(out), Engine::LRAM_SIZE);
  return f.gcount() == (std::streamsize)Engine::LRAM_SIZE;
}

} // namespace

int main(int argc, char** argv)
try
{
  // --- Minimal CLI parse (jaffar-player-compatible flags; render/attended flags are no-ops here) ---
  std::string configFile, solutionFile, dumpRamPath, initialRamPath;
  bool        printFinalState = false;
  std::vector<std::string> positionals;

  for (int i = 1; i < argc; i++)
  {
    const std::string a = argv[i];
    auto next = [&](const char* flag) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + flag);
      return argv[++i];
    };
    if (a == "--dumpRam") dumpRamPath = next("--dumpRam");
    else if (a == "--initialRam") initialRamPath = next("--initialRam");
    else if (a == "--printFinalState") printFinalState = true;
    else if (a == "--disableRender" || a == "--unattended" || a == "--exitOnEnd" || a == "--reproduce" || a == "--reload") { /* no-op for the native player */ }
    else if (a == "--frameskip" || a == "--initialSequence" || a == "--runCommand") next(a.c_str()); // consume + ignore
    else if (a.rfind("--", 0) == 0) throw std::runtime_error("Unknown flag: " + a);
    else positionals.push_back(a);
  }
  if (positionals.size() < 2) throw std::runtime_error("Usage: exciteBike-player <config.jaffar> <movie.sol> [--printFinalState] [--dumpRam out] [--initialRam ram]");
  configFile   = positionals[0];
  solutionFile = positionals[1];

  // --- Load + parse the .jaffar config (for the win/fail rules), exactly like jaffar-player ---------
  std::string configStr;
  if (!jaffarCommon::file::loadStringFromFile(configStr, configFile)) throw std::runtime_error("Could not read config: " + configFile);
  const auto config     = jaffarCommon::json::object::parse(configStr);
  const auto gameConfig = jaffarCommon::json::getObject(config, "Game Configuration"); // by value (small)
  const std::vector<Rule> rules = parseRules(gameConfig);

  // --- Load the movie: one raw input per line ------------------------------------------------------
  std::string movieStr;
  if (!jaffarCommon::file::loadStringFromFile(movieStr, solutionFile)) throw std::runtime_error("Could not read movie: " + solutionFile);
  std::vector<std::string> inputs;
  for (const auto& line : jaffarCommon::string::split(movieStr, '\n'))
    if (!line.empty()) inputs.push_back(line);
  const size_t N = inputs.size();

  // --- Seed frame 0 --------------------------------------------------------------------------------
  Engine engine;
  if (!initialRamPath.empty())
  {
    uint8_t seed[Engine::LRAM_SIZE];
    if (!readInitialRam(initialRamPath, seed)) throw std::runtime_error("Could not read 2048-byte seed from: " + initialRamPath);
    engine.seedFromRam(seed);
  }
  else
  {
    engine.reset();
    fprintf(stderr, "[exciteBike-player] WARNING: no --initialRam; using Engine::reset() flat baseline "
                    "(frame 0 will NOT match tas.ram until the boot sequence is modeled).\n");
  }

  // --- Replay, dumping per-step RAM (steps 0..N) and tracking first win/fail -----------------------
  std::string ramDump;
  if (!dumpRamPath.empty()) ramDump.reserve((N + 1) * Engine::LRAM_SIZE);

  ssize_t firstWinStep = -1, firstFailStep = -1;
  auto recordStep = [&](ssize_t step) {
    if (!dumpRamPath.empty()) ramDump.append(reinterpret_cast<const char*>(engine.lram()), Engine::LRAM_SIZE);
    const auto st = evalState(engine, rules);
    if (st == StateType::win && firstWinStep < 0) firstWinStep = step;
    if (st == StateType::fail && firstFailStep < 0) firstFailStep = step;
  };

  recordStep(0); // initial state
  for (size_t i = 0; i < N; i++)
  {
    engine.advance(excitebike::parseJoypad(inputs[i]));
    recordStep((ssize_t)(i + 1));
  }

  if (!dumpRamPath.empty())
  {
    std::ofstream out(dumpRamPath, std::ios::binary);
    if (!out) throw std::runtime_error("Could not open dump file: " + dumpRamPath);
    out.write(ramDump.data(), (std::streamsize)ramDump.size());
  }

  // --- Final-state report (printInfo-style lines so the same grep works as for jaffar-player) -------
  if (printFinalState)
  {
    const auto stateType = evalState(engine, rules);
    const char* stStr = stateType == StateType::win ? "Win" : (stateType == StateType::fail ? "Fail" : "Normal");
    printf("[J+]  + Current Step:                     %04u\n", engine.currentStep());
    printf("[J+]  + Current / Remaining Loop:         %02u/%02u\n", engine.ram(0x3A4), engine.ram(0x57));
    printf("[J+]  + Block X:                          Count: %02u\n", engine.blockXTransitions());
    printf("[J+]  + Bike Pos X:                       %.3f (%02u, %02u)\n", engine.bikePosX(), engine.ram(0x50), engine.ram(0x394));
    printf("[J+]  + Bike Vel X:                       %u (%02u %02u)\n", engine.velX16(), engine.ram(0x90), engine.ram(0x94));
    printf("[J+]  + Bike Air Mode:                    %02u\n", engine.ram(0xB0));
    printf("[J+]  + Bike Angle:                       %02u\n", engine.ram(0xAC));
    printf("[J+]  + Bike Engine Temp:                 %02u\n", engine.ram(0x3B6));
    printf("[J+]  + Race Over Flag:                   %02u\n", engine.ram(0x52));
    printf("[J+] Final Step:                  %ld\n", (long)N);
    printf("[J+] Final State Type:            %s\n", stStr);
    printf("[J+] First Win Step:              %s\n", firstWinStep < 0 ? "none" : std::to_string(firstWinStep).c_str());
    printf("[J+] First Fail Step:             %s\n", firstFailStep < 0 ? "none" : std::to_string(firstFailStep).c_str());
  }

  return 0;
}
catch (const std::exception& e)
{
  fprintf(stderr, "[exciteBike-player] ERROR: %s\n", e.what());
  return 1;
}
