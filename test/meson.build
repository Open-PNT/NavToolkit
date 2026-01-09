srcs_test = run_command([python, find_sources_py, 'navtk', 'cpp'], check: true).stdout().split()
incdir_test = include_directories('navtk')

# We need all three of these to be the same version, so if any of them isn't provided by the system,
# all of them need to be subprojected in.
#
# When meson doesn't find a depenedency named gtest, gtest_main, or gmock through pkg-config, it
# tries to auto-detect them on the system using special cases hardcoded into meson itself.
# https://github.com/mesonbuild/meson/blob/1.3.2/mesonbuild/dependencies/dev.py#L130
# On Ubuntu, however, this returns a false-positive for gmock when libgtest-dev is installed. Adding
# a `version: '>0.0'` parameter to these dependency() calls prevents these buggy internal special
# cases (which return 'version unknown') from being considered.
gtest_dep = dependency('gtest', allow_fallback: false, required: false, version: '>0.0')
gtest_main_dep = dependency('gtest_main', allow_fallback: false, required: false, version: '>0.0')
gmock_dep = dependency('gmock', allow_fallback: false, required: false, version: '>0.0')
if not (gtest_dep.found() and gtest_main_dep.found() and gmock_dep.found())
    gtest_subproj = subproject('gtest')
    gtest_dep = gtest_subproj.get_variable('gtest_dep')
    gtest_main_dep = gtest_subproj.get_variable('gtest_main_dep')
    gmock_dep = gtest_subproj.get_variable('gmock_dep')
endif

# Build a test executable (the binary for our gtest c++ test suite)
test_exe = executable('navtoolkit_tests',
    sources: srcs_test,
    include_directories: incdir_test,
    dependencies: [gtest_dep, navtoolkit_dep],
    cpp_args : ['-D_NAVTK_UNIT_TESTS'])


# Create test tasks for both fast and slow tests (categorized by their total runtime under address
# sanitization) based on test name filters. Run the slow tests using a looser, but faster
# ASAN_OPTIONS in order to save on runtime while preserving test coverage. To maximize our ASAN
# coverage, most tests should be considered "fast" so they run with as much ASAN instrumentation as
# the platform can support. Mark a test "slow" if it's running long loops to check for accumulating
# numerical errors.
test_slowness_markers = ['*Converges*', '*SteadyState*', '*_SLOW*']
test_timeout = get_option('test_timeout')

common_test_env = environment()

common_test_env.append('UBSAN_OPTIONS', 'halt_on_error=true', separator: ':')

slow_env = common_test_env
fast_env = common_test_env
# detect_stack_use_after_return is false by default
fast_env.append('ASAN_OPTIONS', 'detect_stack_use_after_return=true', separator: ':')
slow_env.append('ASAN_OPTIONS', 'quarantine_size_mb=64', separator: ':')

# Openblas's threading implementation spawns a thread pool before gtest even begins, which causes
# warnings from EXPECT_DEATH tests.
fast_env.append('OPENBLAS_NUM_THREADS', '1')

foreach env : [slow_env, fast_env]
    # don't draw plots while running unit tests
    env.set('MPLBACKEND', 'template')

    # tell open_data_file where the navtk-data folder is
    navtk_data_dir = DATA_DIRECTORY
    if target_machine.system() == 'windows'
        # Wine uses drive letter z: to refer to the linux filesystem root.
        navtk_data_dir = 'z:' + '\\'.join(navtk_data_dir.split('/'))
    endif
    env.set('NAVTK_DATA_DIR', navtk_data_dir)
endforeach

gtest_color = []
if get_option('test_color')
    gtest_color=['--gtest_color=yes']
endif

test('Fast Tests', test_exe,
    env: fast_env,
    args: ['--gtest_filter=-' + ':'.join(test_slowness_markers)] + gtest_color,
    timeout: test_timeout,
    protocol: 'gtest')

test('Slow Tests', test_exe,
    env: slow_env,
    args: ['--gtest_filter=' + ':'.join(test_slowness_markers)] + gtest_color,
    timeout: test_timeout,
    protocol: 'gtest')

if (gdal_dep.found())
    srcs_gdal_test = run_command([python, find_sources_py, '../optional/gdal/test', 'cpp'], check: true).stdout().split()
    gdal_test_exe = executable('navtoolkit_gdal_tests',
        sources: [srcs_gdal_test, 'navtk/spdlog_assert.cpp'], # TODO PNTOS-423 Don't hard-code this file path
        include_directories: [incdir_test, '../optional/gdal/test'],
        dependencies: [gtest_main_dep, gmock_dep, navtoolkit_dep, gdal_dep, fs_dep],
        cpp_args : ['-D_NAVTK_UNIT_TESTS'])

    test('GDAL Tests', gdal_test_exe,
        env: fast_env,
        args: ['--gtest_filter=-*_SLOW'] + gtest_color,
        timeout: test_timeout,
        protocol: 'gtest')

    test('Slow GDAL Tests', gdal_test_exe,
        env: slow_env,
        args: ['--gtest_filter=*_SLOW'] + gtest_color,
        timeout: test_timeout,
        protocol: 'gtest')
endif

# Running "ninja test" will execute all tests.  This target "ninja tests" will provide more output.
run_target('tests', command: ['meson', 'test', '-C', meson.project_build_root(), '-v'])

# Copy files needed for unit tests
configure_file(input : 'data/hour1190.18n', output : 'hour1190.18n', copy : true)

subdir('python')
