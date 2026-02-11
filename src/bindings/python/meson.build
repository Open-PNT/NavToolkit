
srcs_pybindings = run_command([python, find_sources_py, meson.current_source_dir(), 'cpp'], check: true).stdout().split()

python_bindings_errors = []
docstring_extraction_errors = []

meson_flag_prefix = '-D'
if meson.is_subproject()
    meson_flag_prefix += meson.project_name() + ':'
endif

if not get_option ('python_bindings')
    xtensor_python_dep = disabler()
    pybind11_dep = disabler()
    aspn_xtensor_py_dep = disabler()
    aspn_xtensor_python_lib_location = ''
    python_bindings_errors += ['Meson not configured with ' +
                               meson_flag_prefix + 'python_bindings=true']
else
    xtensor_python_dep = dependency('xtensor-python',
        version: ['>=0.24.1', '<1.0.0'],
        required: false,
        allow_fallback: true,
        disabler: true)

    if not xtensor_python_dep.found()
        python_bindings_errors += ['xtensor_python not found']
    endif

    pybind11_dep = subproject('pybind11').get_variable('pybind11_dep')

    if not pybind11_dep.found()
        python_bindings_errors += ['pybind11 not found']
    endif

    if not aspn_xtensor_py_dep.found()
        python_bindings_errors += ['aspn_xtensor_py_dep not found']
    endif
endif

subprojects_dir = join_paths(meson.global_source_root(), 'subprojects')

python_with_parser = import('python').find_installation('python3',
                                                        modules: ['cxxheaderparser'],
                                                        required: false)

navtk_generated_hpp = []
python_bindings_cpp_args = ['-DNAVTK_PYTHON_TENSOR']

if not get_option('python_docstrings')
    docstring_extraction_errors += ['Meson configured with ' +
                      meson_flag_prefix + 'python_docstrings=false']
elif python_with_parser.found()
    navtk_generated_hpp = custom_target(
        'navtk_generated.hpp',
        output : 'navtk_generated.hpp',
        input : headers,
        command : [python_with_parser] + [meson.project_source_root() / 'util' / 'extract_docs.py', meson.current_build_dir(), meson.project_source_root()])

    python_bindings_cpp_args += ['-DNAVTK_PYTHON_DOCSTRINGS']
else
    docstring_extraction_errors += ['Could not find cxxheaderparser']
endif

# This is the library that contains all of the navtk source code, but compiled
# with xtensor_python types instead of xtensor types.  Used for Mac OS to link
# against since they cannot link against an extension module.
navtoolkit_python_lib = static_library('navtoolkit_python',
    sources: [srcs_navtk, src_ex, srcs_pybindings, navtk_generated_hpp],
    include_directories: incdir,
    dependencies: [deps, aspn_xtensor_py_dep, xtensor_python_dep, pybind11_dep],
    override_options: ['b_coverage=false', 'b_sanitize=none'],
    cpp_args : python_bindings_cpp_args,
    gnu_symbol_visibility: 'hidden',
    install: true)

navtoolkit_python_dep = declare_dependency(link_with: navtoolkit_python_lib,
    include_directories: incdir,
    dependencies: [deps, aspn_xtensor_py_dep, xtensor_python_dep, pybind11_dep, libpython3_dep])

# This is the Python extension module that can be imported in Python via "import navtk"
python_bindings_lib = python.extension_module('navtk',
    link_whole: navtoolkit_python_lib,
    override_options: ['b_coverage=false', 'b_sanitize=none'],
    cpp_args : python_bindings_cpp_args,
    install: true)

# target to build the python bindings (and dependencies) and nothing else
run_target('python_bindings', depends: python_bindings_lib, command: [python, '-c', 'import sys; print(sys.version)'])

# The location of the Python extension module will be the current build directory
# This can be added to PYTHONPATH to be able to import the module
python_bindings_lib_location = meson.current_build_dir()

python_path =  python_bindings_lib_location + ':' + aspn_xtensor_python_lib_location

### Stubgen

python_with_mypy = import('python').find_installation('python3', modules: ['mypy'], required: false)

if (python_with_mypy.found() and not is_disabler(aspn_xtensor_py_dep))

    generate_stubs = meson.project_source_root() / 'util' / 'generate_stubs.py'

    env = environment()
    env.append('PYTHONPATH', python_path)

    custom_target('stubs',
                  input : python_bindings_lib,
                  output : ['navtk' ],
                  command : [python_with_mypy, generate_stubs, '-v', '-pnavtk', '-o' + meson.current_build_dir()],
                  env: env,
                  build_by_default : true,
                  install_tag: 'python-runtime',
                  depends: aspn_xtensor_lib)
    install_data('py.typed',
                 install_tag: 'python-runtime',
                 install_dir: python.get_install_dir() / 'navtk')
    install_subdir(meson.current_build_dir() / 'navtk',
                   install_tag: 'python-runtime',
                   install_dir: python.get_install_dir())
endif

### Summary

if not is_disabler(python_bindings_lib)
    summary({'Configured to compile': true,
             'PYTHONPATH location': python_path},
        section: 'navtk Python bindings',
        bool_yn: true)

    if docstring_extraction_errors.length() == 0
        summary({'Python docstrings included': true},
            section: 'navtk Python bindings',
            bool_yn: true)
    else
        summary({'Python docstrings included': false,
                 'Python docstrings problem(s)': docstring_extraction_errors},
            section: 'navtk Python bindings',
            bool_yn: true)
    endif

    if (python_with_mypy.found())
        summary({'Python stub generation enabled': true},
            section: 'navtk Python bindings',
            bool_yn: true)
    else
        summary({'Python stub generation enabled': false,
                 'Python stub generation problem(s)': 'mypy not found'},
            section: 'navtk Python bindings',
            bool_yn: true)
    endif

else
    summary({'Configured to compile': false,
             'Problem(s)': python_bindings_errors},
        section: 'navtk Python bindings',
        bool_yn: true)
endif
