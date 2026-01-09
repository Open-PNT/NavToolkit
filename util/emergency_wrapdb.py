#!/usr/bin/env python3
"""
When Meson wrapdb is down, use the emergency fallback wrap downloader
provided by Meson - ghwt.py.
ghwt.py source: https://github.com/mesonbuild/meson/blob/master/ghwt.py

You can check if wrapdb is down by trying to perform the following
command in the project root:  meson wrap list

If wrapdb is down and you are missing packages from wrapdb:
1. In the SUBPROJECTS dictionary below, add a new entry for each subproject
   that is needed
   - Only those subprojects whose .wrap file in the subprojects folder
     has an entry for "patchurl" that starts with
     "https://wrapdb.mesonbuild.com" needs to be added
   - The dictionary key should be the basename of the .wrap file
2. In the SUBPROJECTS dictionary below, remove entries for any subprojects
   that are no longer needed
3. Update the value for each dictionary entry to the appropriate branch.
   - Typically the branch names correspond to the dependency version
     number.  When this script calls ghwt.py, ghwt.py should print out
     all possible branches for each subproject. Select one of those.
4. Execute this script from the project root: python util/emergency_wrapdb.py
5. Run meson without attempting to download packages:
    meson setup build -Dwrap_mode=nodownload
"""

import sys
import configparser
from pathlib import Path
from ghwt import run as ghwt


# Future enhancement:  It might be possible to determine which branch to
# use based on 'patch_url' in the .wrap file
SUBPROJECTS = (('gtest', '1.8.0'),)


def main():

    for subproject, branch in SUBPROJECTS:

        # Get subproject's "directory" entry from .wrap file, then use
        # that to see if the subproject already exists.

        # Assume this script is in the root/util folder, and the wrap
        # files are in the root/subprojects folder
        root_path = Path(__file__).resolve().parents[1]
        wrap_file_path = root_path.joinpath(
            'subprojects', subproject + '.wrap'
        )
        parser = configparser.ConfigParser()
        parser.read(wrap_file_path)
        # If there is no 'directory' entry in the .wrap file, fallback
        # to subproject name
        subproject_dirname = parser['wrap-file'].get('directory', subproject)
        subproject_path = root_path.joinpath('subprojects', subproject_dirname)

        if subproject_path.joinpath('meson.build').exists():
            print(
                'Subproject',
                subproject,
                'already appears to be installed.' '  Skipping.',
            )
        else:
            retcode = ghwt(['install', subproject, branch])
            if retcode == 1:
                print(
                    'Try deleting everything in the subprojects folder '
                    'except the .wrap files.'
                )
                print(
                    'If that fails, please fix NavToolkit script '
                    'util/emergency_wrapdb.py.'
                )
                return 1


if __name__ == '__main__':
    sys.exit(main())
