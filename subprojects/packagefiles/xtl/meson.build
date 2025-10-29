project('xtl', 'cpp', version:'0.8.0', license:'BSD-3-Clause', default_options : ['warning_level=3', 'cpp_std=c++20'])

json_dep = dependency('nlohmann_json',
    version:'>=3.11.0',
    required: false,
    allow_fallback: true)

# Suppress some warnings that pop in this project on Fedora release builds. See navtk #798
# and/or https://github.com/xtensor-stack/xtl/issues/260
xtl_cpp_arg = []
if get_option('buildtype') == 'release' and build_machine.system() == 'linux' and meson.get_compiler('cpp').get_id() == 'gcc'
    xtl_cpp_arg = ['-Wno-maybe-uninitialized', '-Wno-array-bounds', '-Wno-stringop-overflow']
endif

xtl_dep = declare_dependency(
    include_directories:include_directories('include', is_system: true),
    dependencies: [json_dep],
    compile_args: xtl_cpp_arg)

extra_cflags = []

# Workaround from mesonbuild/meson github issue 2550
if meson.is_subproject()
    install_dir = get_option('subproj_includedir')
    extra_cflags += '-I' + get_option('prefix') / install_dir
else
    install_dir = get_option('includedir')
endif

install_subdir('include/xtl', install_dir: install_dir)

pkgconfig = import('pkgconfig')
pkgconfig.generate(name: meson.project_name(),
    version: meson.project_version(),
    description: 'Basic tools (containers, algorithms) used by other quantstack packages.',
    extra_cflags: extra_cflags)

meson.override_dependency('xtl', xtl_dep)
