# The native exciteBike player: replays a .sol movie through excitebike::Engine (engine.hpp).
# engine.hpp and input.hpp are header-only; player.cpp is the only translation unit.
exciteBike_player = executable('exciteBike-player',
  'player.cpp',
  include_directories : [include_directories('.'), jaffarCommon_inc],
  install : false)
