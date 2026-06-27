# The native exciteBike player: replays a .sol movie through excitebike::Engine (engine.hpp).
# engine.hpp and input.hpp are header-only; player.cpp is the only translation unit.
exciteBike_player = executable('exciteBike-player',
  'player.cpp',
  include_directories : [include_directories('.'), jaffarCommon_inc],
  install : false)

# Round-trip validator for the packed working-set serializer (proves State completeness).
test_serialize = executable('test-serialize',
  '../tools/test_serialize.cpp',
  include_directories : [include_directories('.'), jaffarCommon_inc],
  install : false)
test('serializer round-trip (tas)', test_serialize, args : [meson.project_source_root() / 'tas.ram'])
