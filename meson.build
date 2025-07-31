project('navtoolkit', 'cpp',
    default_options: ['cpp_std=c++17', 'backend_max_links=4', 'cpp_args=-DGLIBCXX_ASSERTIONS=1'],
    meson_version: '>=0.59.1',
    version: '2.1.0-3.dev')

# Warn if using an untested version of meson.
if meson.version().version_compare('>=1.5')
  warning('This project is untested for Meson versions greater than 1.4 and is not guaranteed to '
  + 'work with this version')
endif

python = import('python').find_installation('python3')
python_version = python.language_version()

fs = import('fs')

# Troubleshooting
# ---------------
# - If Meson cannot download packages from wrapdb, see util/emergency_wrapdb.py
find_program('pkg-config', required: true)

if host_machine.cpu_family() == 'aarch64' or host_machine.cpu_family() == 'arm'
    add_project_arguments('-Wno-psabi', language: 'cpp')
endif

if (host_machine.system() == 'windows')
    add_project_arguments(['-Wa,-mbig-obj', '-D_USE_MATH_DEFINES', '-Wno-attributes'], language : 'cpp')
endif

if get_option('buildtype') == 'debug'
    add_project_arguments(['-DXTENSOR_ENABLE_ASSERT'], language: 'cpp')
endif

is_release_build = get_option('buildtype') == 'release'
# Do what the matrix_validation option says, but if it is "auto" then set it to "disabled" if it is a release build
use_matrix_validation = get_option('matrix_validation').disable_auto_if(is_release_build)

if not use_matrix_validation.allowed()
    add_project_arguments(['-DNO_MATRIX_VAL'], language: 'cpp')
endif

## Find Sources
find_sources_py = join_paths(meson.project_source_root(), 'util', 'find_sources.py')

src_navtk_path = join_paths(meson.project_source_root(), 'src', 'navtk')
src_walkthrough_path = join_paths(meson.project_source_root(), 'docs', 'src')
src_gdal_path = join_paths(meson.project_source_root(), 'optional', 'gdal', 'src')
src_ex_path = join_paths(meson.project_source_root(), 'examples', 'utils')

srcs_navtk = run_command([python, find_sources_py, src_navtk_path, 'cpp'], check: true).stdout().split()
srcs_walkthrough = run_command([python, find_sources_py, src_walkthrough_path, 'cpp'], check: true).stdout().split()
headers = run_command([python, find_sources_py, src_navtk_path, 'hpp'], check: true).stdout().split()

incdir = [include_directories('src'), include_directories('examples')]

srcs_gdal = run_command([python, find_sources_py, src_gdal_path, 'cpp'], check: true).stdout().split()
headers_gdal = run_command([python, find_sources_py, src_gdal_path, 'hpp'], check: true).stdout().split()
src_ex = run_command([python, find_sources_py, src_ex_path, 'cpp'], check: true).stdout().split()
## Find Deps

cpp_compiler = meson.get_compiler('cpp')
cpp_std = get_option('cpp_std')

xtensor_dep = dependency('xtensor',
    version: ['>=0.25.0', '<1.0.0'],
    include_type: 'system')

xtensor_blas_dep = dependency('xtensor-blas',
    version: ['>=0.21.0', '<1.0.0'])

gdal_dep = disabler()
fs_dep = disabler()
gdal_dep_errors = []
if not get_option('gdal_extensions').disabled()
    gdal_dep = dependency('gdal',
                        # When updating this version number, also update the
                        # summary() block and min_gdal_version variable below.
                        version: '>=2.2.3',
                        required: get_option('gdal_extensions'))
    fs_dep = cpp_compiler.find_library('stdc++fs', required: false)
else
    gdal_dep_errors += ['The option "gdal_extensions" was set to "disabled"']
endif

deps = [xtensor_dep, xtensor_blas_dep]

# The filesystem library is in std::experimental:: on some platforms but std::
# on others, depending on how new the compiler/stdlib are. The header file
# itself may also be in one of two places.
found_working_fs = false
foreach fs_header : [['HAS', 'filesystem'],
                     ['EXPERIMENTAL', 'experimental/filesystem']]
    if found_working_fs
        break
    elif not cpp_compiler.check_header(fs_header[1], dependencies: fs_dep)
        continue
    endif
    foreach fs_namespace : ['::std::filesystem',
                            '::std::experimental::filesystem',
                            '::std::__fs::filesystem']
        fs_using = 'using ' + fs_namespace + '::path;'
        fs_code = '#include<' + fs_header[1] + '>\n' + fs_using
        if cpp_compiler.compiles(fs_code,
                                 name: fs_using,
                                 dependencies: fs_dep,
                                 args: ['-std=' + cpp_std])
            add_project_arguments([
                '-DNAVTK_STD_FILESYSTEM_NAMESPACE=' + fs_namespace,
                '-DNAVTK_STD_' + fs_header[0] + '_FILESYSTEM_HEADER=1'
            ], language: 'cpp')
            found_working_fs = true
            break
        endif
    endforeach
endforeach
if not found_working_fs
    gdal_dep_errors += ['GDAL support requires the std::filesystem or std::experimental::filesystem library. Your compiler/stdlib are too old.']
    fs_dep = disabler()
endif


## GDAL
must_reverify_gdal_version = false
if not gdal_dep.found()
    # workaround for missing pkgconfig files if gdal is where we expect.
    gdal_dep = cpp_compiler.find_library('gdal', required: false)
    must_reverify_gdal_version = true
endif
if gdal_dep.found()
    gdal_include = '#include<gdal/gdal_priv.h>'
    if cpp_compiler.compiles(gdal_include, dependencies: gdal_dep)
        add_project_arguments(['-DGDAL_INCLUDE_IN_SUBFOLDER'], language: 'cpp')
    else
        gdal_include = '#include<gdal_priv.h>'
        if not cpp_compiler.compiles(gdal_include, dependencies: gdal_dep)
            gdal_dep_errors += ['GDAL is in pkg-config, but gdal_priv.h could not be #included (see meson-log.txt for details).']
            gdal_dep = disabler()
        endif
    endif
else
    gdal_dep_errors += ['GDAL not found.']
endif
if gdal_dep.found() and meson.is_cross_build()
    gdal_dep_errors += ['NavToolkit does not support GDAL on cross builds']
    gdal_dep = disabler()
endif
if gdal_dep.found() and must_reverify_gdal_version
    # GDAL_VERSION_NUM is in the form of ((maj)*1000000+(min)*10000+(rev)*100)
    min_gdal_version = 2*1000000 + 2*10000 + 3*100
    gdal_version_test_code = gdal_include + '\n' + \
        '#if GDAL_VERSION_NUM < ' + min_gdal_version.to_string() + '\n' + \
        '#  error\n' + \
        '#endif'
    gdal_version_ok = cpp_compiler.compiles(
        gdal_version_test_code,
        dependencies: gdal_dep)
    if not gdal_version_ok
        gdal_dep_errors += ['GDAL library is available, but too old.']
        gdal_dep = disabler()
    endif
endif
if gdal_dep.found() and found_working_fs
    summary({'Configured to compile': true},
            section: 'GDAL support',
            bool_yn: true)
    add_project_arguments(['-DNAVTK_GDAL_ENABLED'], language: 'cpp')
    incdir += [include_directories('optional/gdal/src')]
    srcs_navtk += srcs_gdal
    headers += headers_gdal
    deps += [gdal_dep, fs_dep]
else
    summary({'Configured to compile': gdal_dep.found() and found_working_fs,
             'GDAL >= 2.2.3 available': gdal_dep.found(),
             'C++ Filesystem Support available': found_working_fs,
             'Problem(s)': gdal_dep_errors},
            section: 'GDAL support',
            bool_yn: true)
    gdal_dep = disabler()
endif

# ASPN-CPP
aspn_xtensor_dep = dependency('aspn23-xtensor', required: false)
# If dependency type is 'internal', then it came from a subproject. So do the normal subproject
# setup to get access to additional variables.
if not aspn_xtensor_dep.found() or aspn_xtensor_dep.type_name() == 'internal'
    firehose_outputs = subproject('firehose-outputs',
                                required: true,
                                default_options: ['aspn-cpp-stl=disabled',
                                                  'aspn-cpp-eigen=disabled',
                                                  'aspn-cpp-xtensor=enabled',
                                                  'aspn-cpp-xtensor-py=enabled',
                                                  'dds-generated=disabled',
                                                  'lcm-generated=disabled'])
    aspn_xtensor_dep = firehose_outputs.get_variable('aspn_xtensor_dep')
    aspn_xtensor_py_dep = firehose_outputs.get_variable('aspn_xtensor_py_dep')
    if not is_disabler(aspn_xtensor_py_dep)
        aspn_xtensor_lib = firehose_outputs.get_variable('python_bindings_lib')
        aspn_xtensor_python_lib_location = firehose_outputs.get_variable('aspn_xtensor_python_lib_location', '')
    endif
else
    aspn_xtensor_py_dep = dependency('aspn23-xtensor-py', required: false, disabler: true)
    if aspn_xtensor_py_dep.found()
        aspn_xtensor_python_lib_location = aspn_xtensor_py_dep.get_variable(pkgconfig: 'aspn_xtensor_python_lib_location', default_value: '')
    endif
endif

conf = configuration_data()
conf.set_quoted('NAVTK_DATA_DIR_PREFIX',
                join_paths(get_option('prefix'), get_option('datadir')))
if target_machine.system() == 'windows'
    conf.set_quoted('NAVTK_OS_DATA_DIRS_ENV_VAR', 'PROGRAMDATA')
    conf.set_quoted('NAVTK_OS_DATA_HOME_ENV_VAR', 'APPDATA')
elif target_machine.system() == 'darwin'
    conf.set_quoted('NAVTK_OS_DATA_HOME_DEFAULT', '~/Library/Application Support')
else
    conf.set_quoted('NAVTK_OS_DATA_DIRS_ENV_VAR', 'XDG_DATA_DIRS')
    conf.set_quoted('NAVTK_OS_DATA_DIRS_DEFAULT', '/usr/local/share:/usr/share')
    conf.set_quoted('NAVTK_OS_DATA_HOME_ENV_VAR', 'XDG_DATA_HOME')
    conf.set_quoted('NAVTK_OS_DATA_HOME_DEFAULT', '~/.local/share')
endif

configure_file(output: 'navtoolkit_meson_config.h',
               configuration: conf)

subdir('src/navtk/utils')

## Top-Level Header
navtoolkit_hpp = custom_target('navtoolkit.hpp',
    output: 'navtoolkit.hpp',
    install: true,
    install_dir: join_paths('include', 'navtk'),
    command: [python, '-c', 'from sys import argv; open(argv[1], "w").writelines(("#include <%s>"+chr(10)) % x for x in argv[2:])', join_paths(meson.current_build_dir(), 'navtoolkit.hpp')] + headers)


## Logging Library
spdlog_dep = dependency('spdlog',
    version: ['>=1.13.0', '<2.0.0'],
    include_type: 'system')
deps += spdlog_dep

cpp_compiler = meson.get_compiler('cpp')

## Primary Libraries
if get_option('main_library')
    navtoolkit_libs = both_libraries('navtoolkit',
        sources: [srcs_navtk],
        include_directories: incdir,
        dependencies: [deps, aspn_xtensor_dep],
        soversion: meson.project_version(),
        install: true)

    navtoolkit_dep = declare_dependency(link_with: navtoolkit_libs.get_static_lib(),
        include_directories: incdir,
        dependencies: [deps, aspn_xtensor_dep])
else
    navtoolkit_dep = disabler()
endif

meson.override_dependency('navtoolkit', navtoolkit_dep)

# Used by the examples and by unit tests
navtk_data_subproj = disabler()
DATA_DIRECTORY = ''
if get_option('get_navtk_data')
    navtk_data_subproj = subproject('navtk-data')
    DATA_DIRECTORY = navtk_data_subproj.get_variable('DATA_DIRECTORY')
endif

# Since pybind11 intentionally doesn't link to libpython, we need to link it
# for our example programs.
libpython3_dep = python.dependency(embed: true, required: false)

subdir('src/bindings/python') # provides python_bindings_lib, xtensor_python_dep, pybind11_dep

python_example_files = \
[join_paths(meson.project_source_root(), 'examples', 'bias_with_update', 'bias_example_with_update.py'),
    join_paths(meson.project_source_root(), 'examples', 'experimental', 'particle_filter_example.py'),
    join_paths(meson.project_source_root(), 'examples', 'scalar_fogm_example.py'),
    join_paths(meson.project_source_root(), 'examples', 'straight_flight_example.py'),
    join_paths(meson.project_source_root(), 'examples', 'virtual_state_block_example.py')]

if get_option('build_docs_tests_examples')
    subdir('test')
    subdir('docs')

    foreach src : srcs_walkthrough
        name = src.split('/')[-1]
        walkthrough = executable(name,
            sources: [src, navtoolkit_hpp],
            dependencies: [xtensor_dep, navtoolkit_dep])
        test(name,
            walkthrough,
            env: common_test_env,
            timeout: test_timeout)
    endforeach

    # pybind11 leaks a lot of memory, so disable leak checkers for now
    embedded_interpreter_env = common_test_env
    embedded_interpreter_env.append('ASAN_OPTIONS', 'detect_leaks=false', separator: ':')

    subdir('examples')
endif

src_dirs = ['src', 'test', 'examples', 'docs', 'optional']
extensions = ['cpp', 'hpp']
files_to_format = []
foreach src_dir : src_dirs
  foreach extension : extensions
    files = run_command([python, find_sources_py, src_dir, extension], check: true).stdout().split()
    foreach file : files
      if file != ''
        files_to_format += ['@SOURCE_ROOT@/' + file]
      endif
    endforeach
  endforeach
endforeach

clang_format_ok_versions = ['>=18.0', '<19.0']
clang_format_dep = find_program('clang-format-18', 'clang-format',
                                required: false,
                                version: clang_format_ok_versions)
if clang_format_dep.found()
    run_target('format', command: [clang_format_dep, '-style=file', '--verbose', '-i'] + files_to_format)
else
    run_target('format', command: [python, '-c', 'import sys; print("`format` is disabled because clang-format %r does not appear to be installed. Consider using docker_interface.py to format your code." % sys.argv[1:])'] + clang_format_ok_versions)
endif

black_dep = find_program('black', required: false)
if black_dep.found()
    run_target('format_py', command: [black_dep,
                                      '@CURRENT_SOURCE_DIR@',
                                      '--skip-string-normalization',
                                      '--skip-magic-trailing-comma',
                                      '--line-length', '79'])
else
    run_target('format_py', command: [python,'-c', 'print("black not found - consider using `docker_interface.py`."); import sys; sys.exit(1)'])
endif

flake8_dep = find_program('flake8', required: false)
if flake8_dep.found()
    run_target('flake8', command: [flake8_dep,
                                      '@CURRENT_SOURCE_DIR@src',
                                      '@CURRENT_SOURCE_DIR@test',
                                      '@CURRENT_SOURCE_DIR@util',
                                      '@CURRENT_SOURCE_DIR@docs',
                                      '@CURRENT_SOURCE_DIR@examples',
                                      '@CURRENT_SOURCE_DIR@optional'])
else
    run_target('flake8', command: [python,'-c', 'print("flake8 not found - consider using `docker_interface.py`."); import sys; sys.exit(1)'])
endif

if get_option('main_library')
    subdir('install')
endif
