project('xtensor-python', 'cpp',
        version:'0.29.0',
        license:'BSD-3-Clause',
        default_options : ['warning_level=3', 'cpp_std=c++20'])

pybind11_dep = subproject('pybind11').get_variable('pybind11_dep')

xtensor_dep = dependency('xtensor',
    version: ['>=0.25.0', '<1.0.0'])

python = import('python').find_installation('python3',
                                            modules: ['numpy'])

numpy_dep = dependency('numpy', required: false, version: '>=2.0.0')

# TODO: Remove this once we can require a minimum meson version of 1.4.0 and a minimum numpy version
# of 2.0.0
if is_disabler(numpy_dep) or not numpy_dep.found()
    numpy_dep = dependency('numpy', required: false)
endif
if is_disabler(numpy_dep) or not numpy_dep.found()
    npy_incstr = run_command(
        [python, '-c', 'import numpy; print(numpy.get_include())'],
        check: false).stdout().strip()

    if (npy_incstr == '')
        error('Unable to find numpy (Looking for it using ' + python.path() + ')')
    endif

    numpy_dep = declare_dependency(include_directories: npy_incstr)
endif

xtensor_python_dep = declare_dependency(
    include_directories: include_directories('include', is_system: true),
    # Numpy should be listed first (#1022).
    dependencies:[numpy_dep, xtensor_dep, pybind11_dep])

extra_cflags = []

# Workaround from mesonbuild/meson github issue 2550
if meson.is_subproject()
    install_dir = get_option('subproj_includedir')
    extra_cflags += '-I' + get_option('prefix') / install_dir
else
    install_dir = get_option('includedir')
endif

install_subdir('include/xtensor-python', install_dir: install_dir)

pkgconfig = import('pkgconfig')
pkgconfig.generate(name: meson.project_name(),
    version: meson.project_version(),
    description: 'Python bindings for the xtensor C++ multi-dimensional array library.',
    requires: ['xtensor', 'pybind11'],
    extra_cflags: extra_cflags)

meson.override_dependency('xtensor-python', xtensor_python_dep)
