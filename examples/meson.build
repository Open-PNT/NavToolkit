aspn_cpp_lcm_dep = dependency('aspn-cpp-lcm',
    required: get_option('lcm_examples'),
    allow_fallback: true,
    disabler: true)

bias_example_with_update = executable('bias_example_with_update',
    sources: 'bias_with_update/bias_example_with_update.cpp',
    include_directories: 'bias_with_update',
    dependencies: [navtoolkit_dep],
    cpp_args : ['-D_NAVTK_UNIT_TESTS'])

test('Pure C++ Examples',
    bias_example_with_update,
    env: common_test_env,
    timeout: test_timeout)

run_target('run_bias_example_with_update', command: [bias_example_with_update])

straight_flight_example = executable('straight_flight_example',
    sources: ['straight_flight_example.cpp', src_ex],
    dependencies: [xtensor_python_dep, pybind11_dep, libpython3_dep, navtoolkit_dep],
    cpp_args : ['-D_NAVTK_UNIT_TESTS'])

test('C++ Examples with Embedded Python',
    straight_flight_example,
    env: embedded_interpreter_env,
    args: ['0', '2'],
    timeout: test_timeout)

run_target('run_straight_flight_example', command: [straight_flight_example])

aliased_example=executable('aliased_example',
    sources: 'aliased_example.cpp',
    dependencies: [deps, navtoolkit_dep],
    cpp_args : ['-D_NAVTK_UNIT_TESTS', '-DENABLE_SHOW'])

test('Aliased Example',
    aliased_example,
    env: common_test_env,
    args: ['10'],
    timeout: test_timeout)

run_target('run_aliased_example', command: [aliased_example])


# Data for examples
log_files = []
if not is_disabler(navtk_data_subproj)
    log_files = navtk_data_subproj.get_variable('log_files', [])
endif

if log_files.length() == 0
    warning('Did not find \'log_files\' variable in navtk data subproject. Examples that require a logfile will not be compiled.')
else
    # Create optional example utilities library
    gps_ins_lib_src = run_command([python, find_sources_py, 'gps_ins/utils', 'cpp'], check: true).stdout().split()
    gps_ins_lib = static_library('gps_ins',
        sources: gps_ins_lib_src,
        include_directories: incdir,
        dependencies: [xtensor_python_dep, pybind11_dep, libpython3_dep, navtoolkit_dep, aspn_cpp_lcm_dep])
    gps_ins_dep = declare_dependency(link_with: gps_ins_lib,
        include_directories: [incdir],
        dependencies: [aspn_cpp_lcm_dep])

    gps_ins_loosely_coupled_flight_example = executable('gps_ins_loosely_coupled_flight_example',
        sources: ['gps_ins/gps_ins_loosely_coupled_flight.cpp', src_ex],
        dependencies: [xtensor_python_dep, pybind11_dep, libpython3_dep, aspn_cpp_lcm_dep, gps_ins_dep, navtoolkit_dep],
        cpp_args : ['-D_NAVTK_UNIT_TESTS'])

    test('GPS/INS Loosely Coupled Buffered Flight Example',
        gps_ins_loosely_coupled_flight_example,
        env: embedded_interpreter_env,
        args: [join_paths(DATA_DIRECTORY, log_files[0]), '0', '120'],
        timeout: test_timeout)

    run_target('run_gps_ins_loosely_coupled_flight_example', command: [gps_ins_loosely_coupled_flight_example, join_paths(DATA_DIRECTORY, log_files[0])])

    check_lcm = run_command([python, '-c', 'import lcm'], check: false)

    if check_lcm.returncode() == 0 and not meson.is_cross_build()
        # Make python gps_ins_loosely_coupled_flight_example run target
        python_lcm_generated_path = join_paths(meson.project_source_root(), 'subprojects', 'lcm-generated', 'python')
        python_example_pythonpath = python_bindings_lib_location + ':' + python_lcm_generated_path + ':' + aspn_xtensor_python_lib_location
        gps_ins_python_path = join_paths(meson.project_source_root(), 'examples', 'gps_ins_python', 'gps_ins_loosely_coupled_flight.py')
        run_target('run_gps_ins_loosely_coupled_flight_example_py',
                  command: [python, gps_ins_python_path, join_paths(DATA_DIRECTORY, log_files[0])],
                  env: {'PYTHONPATH': python_example_pythonpath},
                  depends: python_bindings_lib)
        if get_option('buildtype') == 'release'
            test('Python Loosely Coupled Flight Example',
                python,
                args : [gps_ins_python_path, join_paths(DATA_DIRECTORY, log_files[0]), '0', '3'],
                env: {'PYTHONPATH': python_example_pythonpath},
                depends: python_bindings_lib,
                timeout: 120)
        endif
    endif

endif

foreach file : python_example_files
    module_name = fs.stem(file)
    run_target('run_' + module_name + '_py',
            command: [python, file],
            env: {'PYTHONPATH': python_bindings_lib_location + ':' + aspn_xtensor_python_lib_location},
            depends: python_bindings_lib)
endforeach
