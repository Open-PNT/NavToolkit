project('spdlog', 'cpp', version : '1.13.0', license: 'MIT',
        default_options : ['cpp_std=c++11'])

inc = include_directories('include')

thread_dep = dependency('threads')

spdlog_dep = declare_dependency(include_directories : inc,
                                dependencies : thread_dep,
                                compile_args: ['-DFMT_HEADER_ONLY'])

extra_cflags = []

# Workaround from mesonbuild/meson github issue 2550
if meson.is_subproject()
    install_dir = get_option('subproj_includedir')
    extra_cflags += '-I' + get_option('prefix') / install_dir
else
    install_dir = get_option('includedir')
endif

install_subdir('include/spdlog', install_dir: install_dir)

pkgconfig = import('pkgconfig')
pkgconfig.generate(name: meson.project_name(),
    version: meson.project_version(),
    description: 'Very fast, header-only/compiled, C++ logging library.',
    extra_cflags: extra_cflags)

meson.override_dependency('spdlog', spdlog_dep)
