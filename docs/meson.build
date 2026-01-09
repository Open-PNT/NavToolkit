mathjax_directory = subproject('mathjax').get_variable('mathjax_directory')

build_documentation_py = meson.current_source_dir() / 'build_documentation.py'

if get_option('show_sphinx_warnings')
    run_target('docs', command: [python, build_documentation_py, mathjax_directory, meson.project_build_root(), '--show-warnings'])
else
    run_target('docs', command: [python, build_documentation_py, mathjax_directory, meson.project_build_root()])
endif
