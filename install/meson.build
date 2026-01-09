# The variable 'headers' has absolute paths to all of our headers.
# Example: /home/user/projects/navtk/src/navtk/utils/algorithm.hpp
# By default, Meson's `install_headers` function will put every header file
# directly in the /{prefix}/include folder.
# For example, if {prefix} is usr/local: /usr/local/include/algorithm.hpp
# However, we want to keep the nesting, so we need to isolate the piece of the
# path between the last `navtk` folder and the file name, and supply that as
# the `subdir` argument of `install_headers`.
# Example subdir: navtk/utils
# So the final install directory would be:
# /usr/local/include/navtk/utils/algorithm.hpp
# Generically:
# /{prefix}/{install_dir}/{subdir}/filename.hpp

foreach h : headers
    full_path = h.split('/')
    last = full_path.length()
    count = 0
    navtk_found = false
    subdir_path = []

    # Construct `subdir_path`
    foreach piece : full_path
        count += 1

        # Discard filename at the end
        if count == last
            break
        endif

        if piece == 'navtk'
            navtk_found = true
            # If there are multiple `navtk` in the path, we only want
            # to use the latest one.
            subdir_path = []
        endif

        if navtk_found
            subdir_path += piece
        endif
    endforeach

    install_headers(h, subdir: join_paths(subdir_path))
endforeach

pkg = import('pkgconfig')

subproj_include = '-I' + get_option('prefix') / get_option('subproj_includedir')

git_commit = run_command([python, join_paths(meson.project_source_root(), 'util', 'git_commit_hash.py')], check: true).stdout().strip()

pkg.generate(navtoolkit_libs,
    filebase: 'navtoolkit',
    name: 'NavToolkit',
    description: 'Sensor fusion and estimation library',
    version: meson.project_version(),
    requires: ['xtensor', 'xtensor-blas'],
    extra_cflags: [subproj_include],
    variables: ['commit_hash=' + git_commit])
