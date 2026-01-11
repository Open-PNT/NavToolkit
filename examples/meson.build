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

foreach file : python_example_files
    module_name = fs.stem(file)
    run_target('run_' + module_name + '_py',
            command: [python, file],
            env: {'PYTHONPATH': python_bindings_lib_location + ':' + aspn_xtensor_python_lib_location},
            depends: python_bindings_lib)
endforeach
