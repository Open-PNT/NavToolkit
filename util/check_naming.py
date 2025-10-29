#!/usr/bin/env python3

from glob import glob
from os.path import join
import sys
from sys import version_info
from navtk_multiprocessing import MultiProcessManager
import numpy as np


def repo_glob(*parts, flag=''):
    '''
    Expands paths with * into the possible values, prepending flag to the
    results.
    '''
    return [
        '%s%s' % (flag, path) for path in glob(join(*parts), recursive=True)
    ]


def build_extract_args():
    '''
    Provides default arguments.
    '''
    x = str(version_info.minor)
    # -fsized-deallocation: https://github.com/pybind/pybind11/issues/1604
    args = [
        '-std=c++20',
        '-fsized-deallocation',
        f'-I/usr/lib/python3.{x}/site-packages/numpy/core/include',
        f'-I{np.get_include()}',
        '-isystem/usr/lib/python3/dist-packages/numpy/core/include',
        f'-I/usr/include/python3.{x}',
        '-Isubprojects/firehose-outputs/aspn-cpp/src',
        '-Isubprojects/firehose-outputs/aspn-c/src',
        '-D_NAVTK_UNIT_TESTS',
        '-DLCM_PYTHON',  # Workaround for a header that won't be generated
        '-I.',
        f'-I{repo_glob("test/navtk")[0]}',
        f'-I{repo_glob("examples")[0]}',
    ]
    args.extend(repo_glob('subprojects', '*', flag="-I"))
    args.extend(repo_glob('subprojects', '*', 'include', flag="-I"))
    args.extend(
        repo_glob('subprojects', '*', 'googletest', 'include', flag="-I")
    )
    args.extend(repo_glob('subprojects', 'lcm-generated', 'cpp', flag="-I"))
    args.extend(repo_glob('src', flag="-I"))
    filenames = []
    for srcdir in ('test', 'src/navtk', 'examples'):
        filenames.extend(repo_glob(srcdir, '**', '*.cpp'))
        filenames.extend(repo_glob(srcdir, '**', '*.hpp'))
    return args, filenames


def main(args=None):
    '''
    If no arguments are specified, this checks C++ files for conformance to
    naming conventions.
    '''
    args, filenames = build_extract_args() if not args else args
    args.insert(0, sys.executable)
    args.insert(1, 'util/process_file_naming.py')
    args.extend(' ')
    manager = MultiProcessManager()
    for filename in filenames:
        args[-1] = filename
        manager.run_process(args)
    manager.finish_processes()

    if manager.exit_code == 0:
        print('No naming issues detected; check successful.')
    else:
        print('Naming issues detected; check failed.')
    sys.exit(manager.exit_code)


if __name__ == '__main__':
    sys.argv.append('-hardexit')  # awful, awful hack
    main()
