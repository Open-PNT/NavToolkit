project('xtensor', 'cpp',
        version:'0.27.1',
        license:'BSD-3-Clause',
        default_options : ['warning_level=3', 'cpp_std=c++20'])

xtensor_cpp_arg = []

xtl_dep = dependency('xtl',
    version:['>=0.7.5', '<1.0.0'])

json_dep = dependency('nlohmann_json',
    version:'>=3.11.0',
    required: false,
    allow_fallback : true)

xsimd_dep = dependency('xsimd',
    version: ['>=11.0.0', '<12.0.0'],
    required: get_option('XTENSOR_USE_XSIMD'),
    allow_fallback: true)

if xsimd_dep.found()
    xtensor_cpp_arg = ['-DXTENSOR_USE_XSIMD']
endif

xtensor_dep = declare_dependency(
    include_directories:include_directories('include', is_system: true),
    dependencies: [xtl_dep, json_dep, xsimd_dep],
    compile_args: xtensor_cpp_arg)

extra_cflags = []

# Workaround from mesonbuild/meson github issue 2550
if meson.is_subproject()
    install_dir = get_option('subproj_includedir')
    extra_cflags += '-I' + get_option('prefix') / install_dir
else
    install_dir = get_option('includedir')
endif

install_subdir('include/xtensor', install_dir: install_dir)

pkgconfig = import('pkgconfig')
pkgconfig.generate(name: meson.project_name(),
    version: meson.project_version(),
    description: 'xtensor is a C++ library meant for numerical analysis with multi-dimensional array expressions.',
    extra_cflags: extra_cflags)

meson.override_dependency('xtensor', xtensor_dep)
