
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

    python_with_aspn_cpp = import('python').find_installation('python3', modules: ['aspn23_xtensor'], required: false)

    if python_with_aspn_cpp.found()
        aspn_xtensor_python_lib_location = ''

        result = run_command(
            [python_with_aspn_cpp, '-c', 'import aspn23_xtensor; import os; print(os.path.relpath(os.path.dirname(aspn23_xtensor.__file__)))'],
            check: false)

        aspn_install_dir_abs = run_command(
            [python_with_aspn_cpp, '-c', 'import aspn23_xtensor; import os; print(os.path.dirname(aspn23_xtensor.__file__))'],
            check: true).stdout().strip() / 'aspn23_xtensor'

        if result.returncode() == 0
            aspn_install_dir = result.stdout().strip() / 'aspn23_xtensor'
        else
            aspn_install_dir = aspn_install_dir_abs
        endif

        # Get path to ASPN-C headers vendored by ASPN-C++ Python module by iterating over potential
        # locations.
        fs = import('fs')
        possible_header_paths = [
            aspn_install_dir.replace('lib', 'include/site').replace('site-packages/', ''),
            aspn_install_dir.replace('lib', 'include/site/python' + python_with_aspn_cpp.language_version()).replace('site-packages/', ''),
            aspn_install_dir.replace('lib', 'include').replace('site-packages/', ''),
            aspn_install_dir.replace('lib', 'include').replace('site-packages/', '').replace('dist-packages/', '').replace('local/', ''),
            aspn_install_dir.replace('Lib', 'include/site/python' + python_with_aspn_cpp.language_version()).replace('site-packages/', ''),
        ]

        aspn_include_dir = ''
        foreach possible_path: possible_header_paths
            if fs.is_dir(possible_path)
                aspn_include_dir = possible_path
                break
            endif
        endforeach

        if aspn_include_dir == ''
            # Must be an additional path that needs to be added to possible_header_paths.
            error('Found ASPN-C++ installation, but could not locate the ASPN-C++ headers')
        endif

        aspn_xtensor_py_dep = declare_dependency(
            include_directories: include_directories(aspn_install_dir, aspn_include_dir),
            link_args: ['-L' + aspn_install_dir_abs, '-laspn_xtensor_py'])
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

pathsep = build_machine.system() == 'windows' ? ';' : ':'
python_path =  python_bindings_lib_location + pathsep + aspn_xtensor_python_lib_location

vendored_openblas_dll = meson.global_source_root() / '.openblas' / 'bin' / 'libopenblas.dll'
if host_machine.system() == 'windows' and fs.exists(vendored_openblas_dll)
    configure_file(input: vendored_openblas_dll,
                   output: 'libopenblas.dll',
                   copy: true)
endif

### Stubgen

pybind11_stubgen = find_program('pybind11-stubgen', required : false)

# Ensure both pybind11-stubgen and ASPN-C++ can be found
stubgen_ready = pybind11_stubgen.found() and not is_disabler(aspn_xtensor_py_dep)

# TODO Add support to generate stubs on Windows ARM platforms
acceptable_platform = build_machine.system() != 'windows' or host_machine.cpu_family() != 'aarch64'

if (stubgen_ready and acceptable_platform)

    env = environment()
    env.append('PYTHONPATH', python_path)

    custom_target('stubs',
                  input : python_bindings_lib,
                  output : ['navtk' ],
                  command : [pybind11_stubgen, '-o' + meson.current_build_dir(), 'navtk', '--exit-code'],
                  env: env,
                  build_by_default : true,
                  install_tag: 'python-runtime')
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

    if (pybind11_stubgen.found())
        summary({'Python stub generation enabled': true},
            section: 'navtk Python bindings',
            bool_yn: true)
    else
        summary({'Python stub generation enabled': false,
                 'Python stub generation problem(s)': 'pybind11-stubgen not found'},
            section: 'navtk Python bindings',
            bool_yn: true)
    endif

else
    summary({'Configured to compile': false,
             'Problem(s)': python_bindings_errors},
        section: 'navtk Python bindings',
        bool_yn: true)
endif
