#!/usr/bin/env python3

import os
from glob import glob
import sys
from shutil import rmtree
import re

from mkdoc import mkdoc
from navtk_multiprocessing import MultiProcessManager


def find_files(pattern, prefix, callback=None):
    '''
    Returns a generator object that results in files or directories matching
    pattern. For example, if pattern is 'subprojects/*/include' then this will
    result in all of the directories in subprojects that have an 'include/'
    child directory. Prefix gets appended to the beginning of each result.
    '''
    matches = glob(pattern, recursive=True)
    if not matches:
        matches = [pattern]
    for match in matches:
        if callback is None or callback(match):
            yield '%s%s' % (prefix, match)


def process_globs(args):
    '''
    Process the given arguments:
        -C Change the working directory to the specified path.
        -Q Sets the access and modified file times of the specified file.
        -I Include the specified directory. Can match *.
        [path] Search through path for files. Can match *.
    '''
    args_out = []
    files = []
    out_file = None

    for next_index, arg in enumerate(args, start=1):
        if arg.startswith('-C'):
            os.chdir(arg[2:] or args.pop(next_index))
        elif arg.startswith('-Q'):
            with open(arg[2:], 'w'):
                os.utime(arg[2:])
        elif arg.startswith('-I'):
            i = arg[2:] or args.pop(next_index)
            args_out.extend(
                find_files(os.path.normpath(i), '-I', os.path.isdir)
            )
        elif arg.startswith('-o'):
            out_file = arg[2:]
        elif not arg.startswith('-'):
            files.extend(find_files(os.path.normpath(arg), ''))
        else:
            args_out.append(arg)

    return args_out, files, out_file


def increment_string(string):
    '''
    Converts the suffix to an int, increments it, converts it back to a string,
    and returns the original string with the incremented final character.
    '''
    pieces = string.split('_')
    if pieces[-1].isdigit():
        pieces[-1] = str(int(pieces[-1]) + 1)
        return '_'.join(pieces)
    return string


def disambiguate_names(name, names):
    '''
    If name is in names, then either append _2 to name or, if name already has
    a number appended, increment that number. Do this recursively until name is
    not an element of names.
    '''
    # Check if name is a duplicate
    if name in names:
        new_name = name
        # If name already ends with _X, then increment
        new_name = increment_string(name)
        # Otherwise it's the first so append _2
        if name == new_name:
            new_name += '_2'
        # Check that new name is not a duplicate
        return disambiguate_names(new_name, names)
    return name


def main():
    '''
    Extract the system arguments. Extract the docstrings from files found in
    the specified directories, on file at a time. This uses subprocess calls
    rather than calling mkdoc directly for all the files to avoid a memory
    leak. The end result should be a single file, named according to the -o
    option.
    '''
    args, files, final_file = process_globs(sys.argv[1:])

    # Short-circuit the no-output case, assuming that if there are no output
    # arguments then it is a single file and does not need to be done via
    # subprocesses.
    if not final_file:
        args.append(*files)
        sys.exit(0 if mkdoc(args) else 1)

    # Use the out argument to infer a temporary directory for output.
    temp_dir = '/'.join(final_file.split('/')[0:-1]) + '/temp/'

    # Clean out output directory
    rmtree(temp_dir, ignore_errors=True)
    os.mkdir(temp_dir)

    # Create a bunch of extracted files
    args.extend(' ')
    manager = MultiProcessManager()
    out_files = []  # Keep track of all files created.
    for file in files:
        args[-1] = file

        # Run mkdoc on a single file, outputting it to the temporary directory.
        file = temp_dir + file.split('/')[-1].strip('.hpp')
        out_files.append(file)
        manager.run_process(
            [sys.executable, 'util/extract_header.py', *args, file]
        )
    # Pause until all processes have completed
    manager.finish_processes()

    # Combine the extracted files into navtk_generated.hpp. The first file
    # should be copied in as-is, but subsequent files should have the macro
    # definitions removed first.
    first_file = True
    # TODO This currently produces a partial file if an error is encountered
    # part of the way through. Ideally, no file would be created or a complete
    # file would be generated.
    with open(final_file, 'w') as big_file:
        ending_text = ''
        unique_names = []
        for file in out_files:
            with open(file, 'r') as little_file:
                text = little_file.read()
                if first_file:
                    # Write the compiler directives at the beginning of the
                    # file.
                    starting_text = re.findall(
                        r".*?#endif", text, flags=re.DOTALL
                    )[0]
                    big_file.write(starting_text + '\n\n')
                    # Find the compiler directives at the end of the file.
                    ending_text = re.findall(
                        r"#if.*?#endif", text, flags=re.DOTALL
                    )[-1]
                    first_file = False
                else:
                    # Remove compiler directives at the top of the file
                    copy_text = text
                    # TODO Is there a performance advantage to pulling these
                    # regex strings out of the loop?
                    text = re.sub(
                        r".*?(static const char)",
                        r"\1",
                        text,
                        count=1,
                        flags=re.DOTALL,
                    )
                    if text == copy_text:
                        continue
                    # Remove compiler directives at the end of the file
                    text = re.sub(
                        r"#if.*", r"", text, count=1, flags=re.DOTALL
                    )

                # Deal with each docstring individually.
                docstrings = re.findall(
                    r"static const char.*?\)doc\";", text, flags=re.DOTALL
                )

                for docstring in docstrings:
                    # Pull out variable name containing the docstrings.
                    name = re.findall(r"__doc_\S*", docstring)[0]

                    # If a name appears twice, give it a new name (_X).
                    new_name = disambiguate_names(name, unique_names)
                    if new_name != name:
                        docstring = re.sub(
                            r"{}\b".format(name),
                            r"{}".format(new_name),
                            docstring,
                        )

                    # Add the new names to the bank of unique names.
                    unique_names.append(new_name)
                    big_file.write(docstring + '\n\n')
        # Add the compiler directives to the end of the file.
        big_file.write(ending_text + '\n')

    # Clean out output directory
    rmtree(temp_dir, ignore_errors=True)

    sys.exit(manager.exit_code)


if __name__ == '__main__':
    main()
