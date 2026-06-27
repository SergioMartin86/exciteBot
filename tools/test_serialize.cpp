/**
 * @file test_serialize.cpp
 * @brief Round-trip validation of the packed working-set serializer (Engine::serialize/deserialize).
 *
 * Proves the packed State captures the COMPLETE per-frame dependency set:
 *  (1) Per-frame induction: at every frame, serialize the reference engine, deserialize into a fresh
 *      engine, advance BOTH one frame with the same input, and assert they agree on every packed
 *      address + reward + velX + win/fail. If a persistent byte were missing from the set, the fresh
 *      engine would read 0 for it and diverge here. Identical for all frames => the set is complete
 *      (identical packed state -> identical next frame -> identical forever, by induction).
 *  (2) End-to-end: restore at several checkpoints and run to the end; assert final posX + win match.
 *
 * Build:  g++ -std=c++20 -O2 -I source -I extern/jaffarCommon/include -D__JAFFAR_COMMON_INLINE__=inline \
 *             tools/test_serialize.cpp -o build/test_serialize
 * Run:    ./build/test_serialize tas.ram
 */
#include "engine.hpp"
#include <cstdio>
#include <cstdint>
#include <vector>
#include <fstream>

using excitebike::Engine;
static constexpr size_t F = 2048;

int main(int argc, char** argv)
{
  const char* ramPath = argc > 1 ? argv[1] : "tas.ram";
  std::ifstream f(ramPath, std::ios::binary);
  if (!f) { fprintf(stderr, "cannot open %s\n", ramPath); return 1; }
  std::vector<uint8_t> d((std::istreambuf_iterator<char>(f)), {});
  const size_t N = d.size() / F;
  auto frame = [&](size_t i) { return d.data() + i * F; };
  auto ctrlOf = [&](size_t i) { return excitebike::reverse8(frame(i)[0x5C]); }; // input that produced frame i

  printf("packed State = %zu bytes (was 2060); %zu working-set addresses\n",
         sizeof(Engine::State), Engine::kStateAddrCount);

  // (1) per-frame induction
  Engine a; a.seedFromRam(frame(0));
  Engine b;
  long mism = 0; size_t firstBad = 0;
  for (size_t i = 1; i < N; i++)
  {
    Engine::State s; a.serialize(s); b.deserialize(s);
    uint8_t c = ctrlOf(i);
    a.advance(c); b.advance(c);
    bool ok = (a.reward() == b.reward()) && (a.velX16() == b.velX16()) &&
              (a.isWin() == b.isWin()) && (a.isFail() == b.isFail()) &&
              (a.cumColumn() == b.cumColumn());
    for (size_t k = 0; ok && k < Engine::kStateAddrCount; k++)
      if (a.ram(Engine::kStateAddrs[k]) != b.ram(Engine::kStateAddrs[k])) ok = false;
    if (!ok) { if (!mism) firstBad = i; mism++; }
  }
  printf("(1) per-frame restore+advance divergences: %ld%s\n", mism,
         mism ? "" : "  -> packed state is COMPLETE");
  if (mism) printf("    first divergence at frame %zu\n", firstBad);

  // (2) end-to-end from checkpoints: restore at K, run to end, compare final posX + win
  long e2eFail = 0;
  for (size_t K = 1; K < N; K += 137)
  {
    Engine ref; ref.seedFromRam(frame(0));
    for (size_t i = 1; i <= K; i++) ref.advance(ctrlOf(i));
    Engine::State s; ref.serialize(s);
    Engine r; r.deserialize(s);
    for (size_t i = K + 1; i < N; i++) { uint8_t c = ctrlOf(i); ref.advance(c); r.advance(c); }
    if (ref.reward() != r.reward() || ref.isWin() != r.isWin()) e2eFail++;
  }
  printf("(2) end-to-end checkpoint restores mismatching final posX/win: %ld%s\n", e2eFail,
         e2eFail ? "" : "  -> reward+win deterministic across restore");

  return (mism || e2eFail) ? 1 : 0;
}
