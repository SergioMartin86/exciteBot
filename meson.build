project('exciteBot', 'cpp',
  version : '0.1.0',
  default_options : ['cpp_std=c++20', 'warning_level=2', 'buildtype=release'])

# jaffarCommon (git submodule in extern/) provides the same header-only tooling jaffar-player uses:
# JSON config parsing, string split, and file IO. Its headers use the __JAFFAR_COMMON_INLINE__ macro,
# which we define exactly as jaffarCommon's own meson does (plain `inline` is sufficient for the
# header-only utilities we consume; we link none of its compiled sources).
add_project_arguments('-D__JAFFAR_COMMON_INLINE__=inline', language : 'cpp')

jaffarCommon_inc = include_directories(
  'extern/jaffarCommon/include',
  'extern/jaffarCommon/third_party')

subdir('source')
