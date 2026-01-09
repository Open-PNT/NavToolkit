if meson.is_cross_build() == false
    python_examples_env = environment()
    python_examples_env.append('PYTHONPATH',
        python_bindings_lib_location,
        join_paths(meson.project_source_root(), 'examples'),
        join_paths(meson.project_source_root(), 'examples', 'bias_with_update'),
        aspn_xtensor_python_lib_location)
    python_examples_env.set('MPLBACKEND', 'template')
    python_examples_env.set('NAVTK_DATA_DIR', DATA_DIRECTORY)

    python_test_dir = join_paths(meson.project_source_root(), 'test/python')

    python_test_files = run_command([
        python,
        find_sources_py,
        python_test_dir,
        'py'],
        check: true).stdout().split()
    python_test_modules = []

    # Splitting out the python tests like this is necessary because of an
    # import-order problem that I haven't been able to figure out.  If all the
    # python tests are grouped together, via unittest discover, and discover
    # imports the test modules in an order that results in navtk being
    # imported before pylab, the plotting code in the pf example hangs
    # indefinitely at 100% cpu, but only on Ubuntu Focal and only when run
    # within docker.
    #
    # TODO (PNTOS-303): Figure out why the init order (pylab vs navtk)
    # matters, fix it, and then re-simplify this file back to a single target.
    foreach file : python_test_files
        module_name = fs.stem(file)
        if not module_name.contains('test')
            continue
        endif
        test('Python ' + module_name,
            python,
            args : ['-m', 'unittest', '--locals', '-cv', module_name],
            env : python_examples_env,
            depends: python_bindings_lib,
            workdir: python_test_dir,
            timeout: 120)
    endforeach

    foreach file : python_example_files
        module_name = fs.stem(file)
        example_args = [file]
        if module_name == 'particle_filter_example'
            example_args += ['10']
        endif
        test('Python ' + module_name,
             python,
             args : example_args,
             env: python_examples_env,
             depends: python_bindings_lib,
             timeout: 120)
    endforeach

    if python_test_files.length() == 0
        warning('No python tests to import in ' + python_test_dir)
    endif
endif
