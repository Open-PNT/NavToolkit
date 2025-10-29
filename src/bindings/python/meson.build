
srcs_pybindings = run_command([python, find_sources_py, meson.current_source_dir(), 'cpp'], check: true).stdout().split()

python_bindings_errors = []
cindex_errors = []

meson_flag_prefix = '-D'
if meson.is_subproject()
    meson_flag_prefix += meson.project_name() + ':'
endif

if not get_option ('python_bindings')
    xtensor_python_dep = disabler()
    pybind11_dep = disabler()
    aspn_xtensor_py_dep = disabler()
    aspn_xtensor_python_lib_location = ''
    python_bindings_errors += ['Meson configured with ' +
                               meson_flag_prefix + 'python_bindings=false']
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


extract_docs_prog = [join_paths(meson.project_source_root(), 'util', 'extract_docs.py')]
extract_docs_test_flags = [
    '-C' + meson.project_source_root(),
    '-std=c++20',
    '-I' + subprojects_dir + '/*/include',
    '-I' + join_paths(get_option('prefix'), get_option('subproj_includedir')),
    '-I' + subprojects_dir + '/firehose-outputs/aspn-c/src/',
    '-I' + subprojects_dir + '/firehose-outputs/aspn-cpp/src/',
    '-I' + meson.project_source_root() + '/src',
    '-I' + meson.project_source_root()]
extract_docs_test = extract_docs_prog + extract_docs_test_flags

extract_docs_real = extract_docs_prog + [
    '-Q@OUTPUT@'
] + extract_docs_test_flags + [
    meson.project_source_root() + '/src/navtk/**/*.hpp',
    meson.project_source_root() + '/examples/utils/*.hpp',
    meson.project_build_root() + '/src/navtk/utils/version.hpp',
    '-o' + meson.current_build_dir() + '/navtk_generated.hpp']

extract_docs_test_without_filename = extract_docs_test
# note: hard-coded path here is a file with a lot of dependency references,
# to make sure subprojects are wired up correctly.
# TODO: PNTOS-453 we need a better solution.
extract_docs_test += [
    meson.project_source_root() +
    '/src/navtk/filtering/fusion/StandardFusionEngine.cpp']

extract_docs_test_result = run_command([python] + extract_docs_test + ['-hardexit'], check: false)
working_cindex = extract_docs_test_result.returncode() == 0

navtk_generated_hpp = []
python_bindings_cpp_args = ['-DNAVTK_PYTHON_TENSOR']

if not get_option('python_docstrings')
    cindex_errors += ['Meson configured with ' +
                      meson_flag_prefix + 'python_docstrings=false']
    working_cindex = false
elif (working_cindex)
    navtk_generated_hpp = custom_target(
        'navtk_generated.hpp',
        output : 'navtk_generated.hpp',
        input : headers,
        command : [python] + extract_docs_real)

    python_bindings_cpp_args += ['-DNAVTK_PYTHON_DOCSTRINGS']
else
    cindex_errors += [extract_docs_test_result.stderr()]

    # Re-run the test with a filename that _doesn't_ reference any problem
    # files to see whether cindex is installed/working.
    cindex_test = [python] + extract_docs_test_without_filename + [
        meson.project_source_root() + '/src/navtk/not_null.hpp',
        '-hardexit'
    ]
    if run_command(cindex_test, check: false).returncode() == 0
        cindex_errors += ['Generating docstrings for the python bindings requires all subprojects to be downloaded',
                          '(even those you already have installed on your system).',
                          'Please run `meson subprojects download`, or disable docstrings with ' +
                          meson_flag_prefix + 'python_docstrings=false']
    else
        cindex_errors += ['Using clang.cindex failed using ' + python.full_path() + '. Install the clang python bindings and ensure the following command runs:']
        cindex_errors += [python.full_path() + ' \'' + '\' \''.join(extract_docs_test) + '\'']
    endif
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

if (python_with_mypy.found())

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

    if working_cindex
        summary({'Python docstrings included': true},
            section: 'navtk Python bindings',
            bool_yn: true)
    else
        summary({'Python docstrings included': false,
                 'Python docstrings problem(s)': cindex_errors},
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
