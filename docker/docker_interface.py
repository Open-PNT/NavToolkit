#!/usr/bin/env python3

"""Docker Interface

Creates Docker containers for specific platforms, and uses those
containers to build and test pntOS.

This script will create build folders for each unique platform it is told to
build.  This could use a lot of disk space.  It is safe to delete the folder
specified by BUILD_PATH to clear up space.

This script will also create many Docker images.  Refer to Docker
documentation for how to clear the local Docker image cache if needed.

WARNING: Your SSH keys may be mounted into the containers.  If you do not
want this behavior, comment out the appropriate section in the docker_run()
function.

WARNING: If you have a ~/.ccache folder it may be mounted into the containers.
If you do not want this behavior, comment out the appropriate section in
the docker_run() function.
"""

import sys
from os import (
    environ,
    getcwd,
    path,
    makedirs,
    cpu_count,
    getuid,
    mkdir,
    chown,
    stat,
)
from subprocess import check_call, CalledProcessError
import argparse

# This script has different behavior for CI and normal users
CI_USER = environ.get('CI_SERVER') == 'yes'
# Folder that contains the per-platform build folders.
# Relative to project root.
BUILD_PATH = 'docker/build'
# Folder that contains the per-platform Docker image ID files.
# Relative to project root.
IIDFILE_PATH = path.join(BUILD_PATH, 'iidfiles')
# True if OS is linux, False otherwise.
LINUX_USER = sys.platform == 'linux'


# If running as root in a clone not owned by root, don't create root-owned
# folders because that's almost certainly not what the user intended.
if getuid():
    permission_safe_makedirs = makedirs
else:

    def permission_safe_makedirs(name):
        parent = path.dirname(path.abspath(name))
        if parent != name and not path.exists(parent):
            permission_safe_makedirs(parent)
        stat_result = stat(parent)
        parent_uid = stat_result.st_uid
        parent_gid = stat_result.st_gid
        mkdir(name)
        chown(name, parent_uid, parent_gid)


class Platform(object):
    """Represents a Docker container for a particular platform.

    Holds information needed for this script to create a Docker image
    for a particular platform and to create a Docker container from
    that Docker image to run commands inside.

    Attributes:
        platform_name: A string for a unique name for the platform
        dockerfile: A string holding the filename for the Docker file
        base: A string holding the Docker image tag to use for a base
        kw: Keyword arguments passed to the constructor
    """

    def __init__(self, platform_name, dockerfile, base, **kw):
        self.platform_name = platform_name
        self.dockerfile = dockerfile
        self.base = base
        self.kw = kw

    @property
    def build_directory(self):
        """Returns a relative location to the build folder"""
        return path.join(BUILD_PATH, self.platform_name.replace("-", "_"))

    @property
    def iid_file(self):
        """Returns a relative location to the Docker Image ID file."""
        return path.join(IIDFILE_PATH, self.platform_name.replace("-", "_"))


# The name of the platform to be used when no platform name is supplied.
DEFAULT_PLATFORM_NAME = 'ubuntu-noble'

# Meson setup arguments that are common to all platforms.
COMMON_MESON_SETUP_ARGS = ['--warnlevel=3', '-Dwerror=true']

# PLATFORMS is a map which holds all platform information objects. The key to
# the map is also the first argument to the constructor: the platform name.
PLATFORMS = {
    it.platform_name: it
    for it in [
        # -Db_sanitize=address,undefined: Build with ASAN and UBSAN
        # --buildtype=release: Make a release build instead of default debug build
        Platform(
            DEFAULT_PLATFORM_NAME,
            'Dockerfile.ubuntu',
            'ubuntu:24.04',
            meson_setup_args=[
                '-Db_sanitize=address,undefined',
                '--buildtype=release',
            ],
        ),
        Platform('ubuntu-noble-debug', 'Dockerfile.ubuntu', 'ubuntu:24.04'),
        # ASAN is buggy on this cross-compiler
        # --cross-file=cross/armhf-linux.cross: Tell Meson to use specific tools
        # --buildtype=release: Make a release build instead of default debug build
        Platform(
            'cross-armv7',
            'Dockerfile.ubuntu-cross',
            'ubuntu:20.04',
            meson_setup_args=[
                '--cross-file=cross/armhf-linux.cross',
                '-Dbuildtype=release',
            ],
        ),
        # ASAN_OPTIONS=detect_leaks=false: The memory leak detector is unsupported
        #   for ARM64
        # -Db_sanitize=address,undefined: Build with ASAN and UBSAN
        # --cross-file=cross/arm64-linux.cross: Tell Meson to use specific tools
        # -Dtest_timeout=1200: Increase the test timeout limit (emulator slowdown)
        # --buildtype=release: Make a release build instead of default debug build
        Platform(
            'cross-arm64',
            'Dockerfile.ubuntu-cross',
            'ubuntu:20.04',
            docker_run_args=['-e', 'ASAN_OPTIONS=detect_leaks=false'],
            meson_setup_args=[
                '-Db_sanitize=address,undefined',
                '--cross-file=cross/arm64-linux.cross',
                '-Dtest_timeout=1200',
                '-Dbuildtype=release',
            ],
        ),
        # ASAN is unsupported for cross-compiling Windows binaries
        # --buildtype=release: Make a release build instead of default debug build
        # --cross-file=cross/x64-win.cross: Tell Meson to use specific tools
        # -Dpython_bindings=false: Python is not currently supported when
        #   cross-compiling for Windows
        # -Dtest_timeout=1200: Increase the test timeout limit (emulator slowdown)
        Platform(
            'cross-win64',
            'Dockerfile.ubuntu-cross',
            'ubuntu:20.04',
            meson_setup_args=[
                '--buildtype=release',
                '--cross-file=cross/x64-win.cross',
                '-Dpython_bindings=false',
                '-Dtest_timeout=1200',
                '-Dlcm_examples=disabled',
                '-Dfirehose-outputs:aspn-cpp-xtensor-py=disabled',
            ],
        ),
        # -Db_sanitize=address,undefined: Build with ASAN and UBSAN
        # --buildtype=release: Make a release build instead of default debug build
        Platform(
            'fedora-gcc',
            'Dockerfile.fedora',
            'fedora:39',
            meson_setup_args=[
                '-Db_sanitize=address,undefined',
                '--buildtype=release',
            ],
        ),
        # CXX=ccache clang++: Use ccache with clang++ compiler instead of g++
        # -Db_lundef=false: Work around shared object problems when using ASAN
        #   with Clang (see https://github.com/mesonbuild/meson/issues/764)
        # -Db_sanitize=address,undefined: Build with ASAN and UBSAN
        # --buildtype=release: Make a release build instead of default debug build
        Platform(
            'fedora-clang',
            'Dockerfile.fedora',
            'fedora:39',
            docker_run_args=[
                '--env',
                'CC=ccache clang',
                '--env',
                'CXX=ccache clang++',
            ],
            meson_setup_args=[
                '-Db_lundef=false',
                '-Db_sanitize=address,undefined',
                '--buildtype=release',
            ],
        ),
        # -Db_sanitize=address,undefined: Build with ASAN and UBSAN
        # --buildtype=release: Make a release build instead of default debug build
        Platform(
            'ubuntu-jammy',
            'Dockerfile.ubuntu',
            'ubuntu:22.04',
            meson_setup_args=[
                '-Db_sanitize=address,undefined',
                '--buildtype=release',
            ],
        ),
    ]
}


def call(command):
    '''
    Prints out `command`, then sends it to be executed by the system shell.
    '''
    print_command = command.copy()
    for ii, arg in enumerate(print_command):
        if ' ' in arg:
            print_command[ii] = f'"{arg}"'
    print(
        'Running the command:\n{}'.format(' '.join(print_command)), flush=True
    )
    check_call(command)


def safe_add_volume(outside_path, inside_path, flags=''):
    '''
    Return a tuple containing a volume flag and its argument to be used
    as part of an argv, after verifying that the outside_path is something
    that is safe to map and won't result in docker's implicit path creation
    trashing the user's permissions.

    If the outside_path does not exist and the user does not have permissions
    to create it, warn that it could not be mapped and return an empty tuple
    instead rather than letting a potentially-rootful docker create it as
    root-owned.
    '''
    if not path.exists(outside_path):
        try:
            makedirs(outside_path)
        except OSError:
            print(
                "WARNING: could not map folders; outside path does not exist\n"
                f"    outside_path: {outside_path!r}\n"
                f"    inside_path: {inside_path!r}\n",
                file=sys.stderr,
                flush=True,
            )
            return ()
    terms = [outside_path, inside_path]
    if flags:
        terms.append(flags)
    return "--volume", ":".join(terms)


def docker_run(args, task=None):
    '''
    Run the supplied `task` in a Docker container for supplied platform.

    WARNING: The Docker container will be created based on an image that
    should have already been generated using the Platform object before
    calling `run`.  See `docker_build`.

    The platform used will be specified in args.platform_name.

    `task` should be a string consisting of the command to run inside a docker
    container that will be generated for the given `platform`, or a None
    object which will cause the script to open a shell for interactive use.
    '''
    verify_docker_build(args)

    platform = PLATFORMS[args.platform_name]
    cwd = getcwd()
    home_dir = environ.get('HOME')

    # --interactive --tty so that it is possible to use CTRL+C to kill the
    #   container
    # --cap-add SYS_PTRACE to give LeakSanitizer enough permissions to run
    #   without error
    # --security-opt label=disable allows volume mounts to have write access
    #   on podman
    # --rm will delete the container after it exits
    command = [
        'docker',
        'run',
        '--interactive',
        '--tty',
        '--cap-add',
        'SYS_PTRACE',
        *safe_add_volume(cwd, '/work'),
        '--security-opt',
        'label=disable',
        '--rm',
    ]

    # Args supplied by Platform object constructor to pass to "docker run"
    command += platform.kw.get('docker_run_args', '')

    # Mount in ccache directory if it exists
    ccache_dir = f'{home_dir}/.ccache'
    if path.isdir(ccache_dir):
        command += safe_add_volume(ccache_dir, '/home/docker/.ccache')
        print(f'ccache enabled, using cache location: {ccache_dir}')
    else:
        print(f'ccache disabled, cache location does not exist: {ccache_dir}')

    # Use user's SSH keys
    if not CI_USER:
        if LINUX_USER:
            auth_sock = environ.get('SSH_AUTH_SOCK')
            if auth_sock and path.exists(auth_sock):
                command += [
                    *safe_add_volume(auth_sock, '/.ssh-auth-sock'),
                    '-e',
                    'SSH_AUTH_SOCK=/.ssh-auth-sock',
                ]
        ssh_dir = home_dir + '/.ssh'
        command += safe_add_volume(ssh_dir, '/home/docker/.ssh', 'ro')
        command += safe_add_volume(ssh_dir, '/root/.ssh', 'ro')

    command += [open(platform.iid_file).read()]

    if task:
        command += [task]

    call(command)


def platforms(args):
    '''
    Called when `docker_interface.py platforms` is executed.
    Prints the name of every platform stored in the PLATFORMS map.  These
    names can be used in other `docker_interface.py` actions.
    '''
    for i in PLATFORMS:
        print(i)


def docker_build(args):
    '''
    Called when `docker_interface.py docker_build [platform]` is executed.
    Pulls the image base from Docker Hub and then builds the [platform]-
    specific image on top of that base.  The image will then be in the local
    Docker cache.  Also saves the image ID to a text file to be re-used during
    other build stages to check to see if the image is indeed cached locally.

    Note that this function will call `docker pull`, and that Docker Hub has
    a rate limit: <https://docs.docker.com/docker-hub/download-rate-limit/>.
    The Docker client will warn if the rate limit has been exceeded.
    '''
    platform = PLATFORMS[args.platform_name]
    docker_pull_cmd = ['docker', 'pull', platform.base]
    if CI_USER:
        # The CI will easily exceed the Docker Hub rate limit.  The servers
        # have a script `rate_limit` which will prevent issuing a pull request
        # unless a particular time period has elapsed since the last request.
        # 6 hours = 60 * 60 * 6 seconds = 21600 seconds
        docker_pull_cmd = ['rate_limit', '21600'] + docker_pull_cmd
    call(docker_pull_cmd)

    if not path.isdir(IIDFILE_PATH):
        permission_safe_makedirs(IIDFILE_PATH)

    # Build the docker image.
    # --iidfile <file>: writes the ID of the final image to the specified file
    # -f docker/<dockerfile>: specifies which dockerfile to use
    # --build-arg BASE=<base>: sets the BASE variable inside the dockerfile
    # docker: the folder for the docker build context
    build_cmd = [
        'docker',
        'build',
        '--iidfile',
        platform.iid_file,
        '-f',
        'docker/{}'.format(platform.dockerfile),
        '--build-arg',
        'BASE={}'.format(platform.base),
        'docker',
    ]
    call(build_cmd)


def verify_docker_build(args):
    '''
    Check to see if a Docker image for this platform exists in local cache.
    Call "docker_build" if it does not.
    '''

    # We can check if the Docker image that we want is in local cache by
    # checking the contents of the iid_file for this platform, and then
    # querying docker to see if an image with that ID exists.

    if not path.isdir(IIDFILE_PATH):
        permission_safe_makedirs(IIDFILE_PATH)

    # If the iid_file does not exist then we need to build the image.
    platform = PLATFORMS[args.platform_name]
    if not path.isfile(platform.iid_file):
        docker_build(args)

    image_in_local_cache = True

    # `docker image inspect <image id>` will return 0 if the image id
    # specified exists in local cache, and 1 otherwise.
    # `--format=image_check` is added so that the Docker client doesn't print
    # all of the image information.
    command = ['docker', 'image', 'inspect', '--format=image_check']
    command += [open(platform.iid_file).read()]

    # The `call` function throws an exception if `command` returns non-zero
    try:
        call(command)
    except CalledProcessError:
        image_in_local_cache = False

    if not image_in_local_cache:
        docker_build(args)


def setup(args):
    '''
    Called when `docker_interface.py setup [platform]` is executed.
    Remove any existing build directory for the specified platform.
    Then set up a new one using Meson.
    '''
    platform = PLATFORMS[args.platform_name]

    docker_run(args, 'rm -rf ' + platform.build_directory)

    meson_args = COMMON_MESON_SETUP_ARGS
    # Arguments supplied by Platform object constructor to use in Meson setup
    meson_args += platform.kw.get('meson_setup_args', [])
    meson_args = ' '.join(meson_args)

    command = 'meson setup {} {}'.format(platform.build_directory, meson_args)
    docker_run(args, command)


def run_with_build_dir(args, task=None):
    '''
    Check to see if the build directory exists for this platform.  If it does
    not, call "setup" for this platform.  In either event, forward the `task`
    argument to the "docker_run" function.
    '''
    platform = PLATFORMS[args.platform_name]
    if not path.isdir(platform.build_directory):
        setup(args)

    docker_run(args, task)


def build(args):
    '''
    Called when `docker_interface.py build [platform]` is executed.
    Build the project for [platform] using an appropriate docker container.
    '''
    platform = PLATFORMS[args.platform_name]
    command = 'ninja -C {} -j {}'.format(platform.build_directory, args.j)
    run_with_build_dir(args, command)


def test(args):
    '''
    Called when `docker_interface.py test [platform]` is executed.
    Run tests on specified [platform], building it first if necessary.
    '''
    platform = PLATFORMS[args.platform_name]
    command = 'ninja -C {} -j {} test'.format(platform.build_directory, args.j)
    run_with_build_dir(args, command)


def test_all(args):
    '''
    Called when `docker_interface.py test_all` is executed.
    Run "test" for each supported platform.
    '''
    for platform_name in PLATFORMS:
        args.platform_name = platform_name
        test(args)


def format(args):
    '''
    Called when `docker_interface.py format` is executed.
    Format the code so it conforms to the standard.  The format command will
    be run on the default platform.
    '''
    platform = PLATFORMS[DEFAULT_PLATFORM_NAME]
    args.platform_name = platform.platform_name
    command = 'ninja -C {} -j {} format'.format(
        platform.build_directory, args.j
    )
    run_with_build_dir(args, command)
    command = 'ninja -C {} -j {} format_py'.format(
        platform.build_directory, args.j
    )
    run_with_build_dir(args, command)


def flake8(args):
    '''
    Called when `docker_interface.py flake8` is executed. Runs flake8 to check
    for compliance against PEP8. The command will be run on the default
    platform.
    '''
    platform = PLATFORMS[DEFAULT_PLATFORM_NAME]
    args.platform_name = platform.platform_name
    command = 'ninja -C {} -j {} flake8'.format(
        platform.build_directory, args.j
    )
    run_with_build_dir(args, command)


def check_documentation(args):
    '''
    Called when `docker_interface.py check_documentation` is executed.
    Check the documentation for warnings.
    '''
    platform = PLATFORMS[DEFAULT_PLATFORM_NAME]
    args.platform_name = platform.platform_name
    run_with_build_dir(args, 'docs/check_documentation.py')


def debug(args):
    '''
    Called when `docker_interface.py debug [platform] [command]` is executed.
    Run [command] within the docker container specified by [platform].
    If no [command] is provided, open a shell inside the container.
    '''
    docker_run(args, args.command)


def main():
    parser = argparse.ArgumentParser(
        description='''Creates Docker containers for specific platforms, and
                       uses those containers to build and test NavToolkit.'''
    )

    parser.add_argument(
        '-v',
        '--verbose',
        help='DEPRECATED: Has no effect.',
        action='store_true',
    )

    actions = {
        'platforms': 'List available platforms',
        'docker_build': 'Constructs a docker image for one platform.',
        'setup': '''Removes any existing build folder, and configures Meson so
                    that NavToolkit is ready to build on one platform. Will
                    run "docker_build".''',
        'build': '''Builds NavToolkit on one platform, will run "setup" if
                    needed''',
        'test': '''Execute tests for NavToolkit on one platform, will run
                   "build" if needed''',
        'test_all': 'Run "test" on all platforms',
        'format': 'Format C++ and Python source code',
        'flake8': 'Run flake8 on Python source code',
        'check_documentation': 'Run Doxygen and check for warnings',
        'debug': '''Runs [arguments] as a command in the docker container.
                    Defaults to opening a shell in the container.''',
    }

    can_specify_platform = {'docker_build', 'setup', 'build', 'test', 'debug'}

    can_specify_cores = {'build', 'test', 'test_all', 'format', 'flake8'}

    subparsers = parser.add_subparsers()

    for act in actions:
        sub_parser = subparsers.add_parser(act, help=actions[act])

        # Link the action name to a function in this file by the same name
        sub_parser.set_defaults(func=getattr(sys.modules[__name__], act))
        if act in can_specify_platform:
            sub_parser.add_argument(
                'platform_name',
                nargs='?',
                help='Platform to run on, defaults to %(default)s',
                choices=list(PLATFORMS),
                default=DEFAULT_PLATFORM_NAME,
            )
        if act == 'debug':
            sub_parser.add_argument(
                'command', help='Command to execute for debugging', nargs='?'
            )
        if act in can_specify_cores:
            sub_parser.add_argument(
                '-j',
                type=int,
                metavar='N',
                help='Number of parallel threads, defaults to #CPU cores + 2',
                default=(cpu_count() + 2),
            )

    args = parser.parse_args()

    # If user didn't specify arguments, print the help
    if len(sys.argv) < 2:
        args = parser.parse_args(['--help'])

    # Call the function in this file that user specified as an argument
    args.func(args)


if __name__ == '__main__':
    main()
