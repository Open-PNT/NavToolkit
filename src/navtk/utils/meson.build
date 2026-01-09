commit_hash_py = join_paths(meson.current_source_dir(), '..', '..', '..', 'util', 'git_commit_hash.py')

version = meson.project_version()
version_pieces = version.split('-')
version_token = version_pieces[-1].split('.')[0]
version_numbers = version_pieces[0].split('.')

# Add the git index to the command. This is done, not because it's needed by the command, but for
# the indirect feature that changing the git index will trigger this to run.
git_index = join_paths(meson.project_source_root(), '.git', 'index')
commit_hash = run_command([python, commit_hash_py, git_index], check: true).stdout().strip()

conf_data = configuration_data()
conf_data.set('NAVTK_VERSION_MAJOR', version_numbers[0])
conf_data.set('NAVTK_VERSION_MINOR', version_numbers[1])
conf_data.set('NAVTK_VERSION_PATCH', version_numbers[2])
conf_data.set('NAVTK_VERSION_TOKEN', version_token,
              description : 'VERSION_TOKEN: 0 for dev; 1 for rc1, 2 for rc2, etc; 200 for release')
version_hex = '''((NAVTK_VERSION_MAJOR * 0x1000000) | \
                              (NAVTK_VERSION_MINOR * 0x10000) | \
                              (NAVTK_VERSION_PATCH * 0x100) | \
                              (NAVTK_VERSION_TOKEN))'''
conf_data.set('NAVTK_VERSION_HEX', version_hex)
conf_data.set_quoted('NAVTK_VERSION_STRING', version)
conf_data.set_quoted('NAVTK_GIT_COMMIT_HASH', commit_hash)

version_hpp = configure_file(output : 'version.hpp',
                            configuration : conf_data,
                            install : true,
                            install_dir : 'include/navtk/utils')

srcs_navtk += version_hpp
