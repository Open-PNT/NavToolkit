#!/usr/bin/env python3

import os
from glob import glob
import sys
import re
from typing import List, Dict

import cxxheaderparser.simple as parser

PREFACE = '''\
#pragma once
/*
  This file contains docstrings for use in the Python bindings.
  Do not edit! They were automatically extracted by util/extract_docs.py.
 */

#define __EXPAND(x) x
#define __COUNT(_1, _2, _3, _4, _5, _6, _7, COUNT, ...) COUNT
#define __VA_SIZE(...) __EXPAND(__COUNT(__VA_ARGS__, 7, 6, 5, 4, 3, 2, 1))
#define __CAT1(a, b) a##b
#define __CAT2(a, b) __CAT1(a, b)
#define __DOC1(n1) __doc_##n1
#define __DOC2(n1, n2) __doc_##n1##_##n2
#define __DOC3(n1, n2, n3) __doc_##n1##_##n2##_##n3
#define __DOC4(n1, n2, n3, n4) __doc_##n1##_##n2##_##n3##_##n4
#define __DOC5(n1, n2, n3, n4, n5) __doc_##n1##_##n2##_##n3##_##n4##_##n5
#define __DOC6(n1, n2, n3, n4, n5, n6)     __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6
#define __DOC7(n1, n2, n3, n4, n5, n6, n7)     __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6##_##n7
#define DOC(...)     __EXPAND(__EXPAND(__CAT2(__DOC, __VA_SIZE(__VA_ARGS__)))(__VA_ARGS__))

#ifdef __clang__
#   pragma clang diagnostic push
#   pragma clang diagnostic ignored "-Wunused-variable"
#elif defined(__GNUG__)
#   pragma GCC diagnostic push
#   pragma GCC diagnostic ignored "-Wunused-variable"
#endif

'''  # noqa: E501

POSTFACE = '''\
#ifdef __clang__
#   pragma clang diagnostic pop
#elif defined(__GNUG__)
#   pragma GCC diagnostic pop
#endif
'''


DOCSTRINGS: Dict[str, str] = {}
'''
A mapping of the eventual header variable name of the docstring to its actual
value.
'''

CHILD_CLASSES: Dict[str, str] = {}
'''
A mapping of a child class name to its parent class.
'''

MISSING_METHOD_DOCSTRINGS: Dict[str, List[str]] = {}
'''
A mapping of a class name to the docstring names it is missing.
'''


def increment_string(string: str) -> str:
    '''
    Converts the suffix to an int, increments it, converts it back to a string,
    and returns the original string with the incremented final character.
    '''
    pieces = string.split('_')
    if pieces[-1].isdigit():
        pieces[-1] = str(int(pieces[-1]) + 1)
        return '_'.join(pieces)
    return string


def disambiguate_names(name: str, names: List[str]) -> str:
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


def clean_docstring(docstring):
    '''
    Removes doxygen-style prefixes and suffixes from extracted docstrings.
    Also strips trailing whitespace.
    '''
    if docstring is None:
        return None
    # Remove doxygen-style comment markers
    docstring = docstring.strip('/**')
    docstring = docstring.strip('///')
    docstring = docstring.strip('*/')
    docstring = re.sub(r"\n\*", "\n", docstring, flags=re.DOTALL)
    # Remove leading and trailing whitespace
    docstring = docstring.strip()
    # Replace tabs with spaces for better output in the terminal
    docstring = docstring.replace('\t', '    ')
    return docstring


def process_class(input: parser.ClassScope):
    '''
    Extract docstrings for a class, including any methods, fields, or enums.
    '''
    docstring = clean_docstring(input.class_decl.doxygen)
    name = input.class_decl.typename.segments[0].name
    if docstring is not None:
        DOCSTRINGS[name] = docstring
    for field in input.fields:
        docstring = clean_docstring(field.doxygen)
        if docstring is not None:
            DOCSTRINGS[f'{name}_{field.name}'] = docstring
    for method in input.methods:
        method_name = method.name.segments[0].name
        if method_name.startswith('operator'):
            continue
        if method_name.startswith('~'):
            continue
        docstring = clean_docstring(method.doxygen)
        if docstring is not None:
            combined_name = disambiguate_names(
                f'{name}_{method_name}', DOCSTRINGS
            )
            DOCSTRINGS[combined_name] = docstring
        else:
            if name in MISSING_METHOD_DOCSTRINGS:
                MISSING_METHOD_DOCSTRINGS[name] += [method_name]
            else:
                MISSING_METHOD_DOCSTRINGS[name] = [method_name]
    for enum in input.enums:
        process_enum(enum, name)
    # Map child to parents for inherited docstrings to be processed later.
    bases = input.class_decl.bases
    if len(bases) > 0:
        CHILD_CLASSES[name] = [
            base.typename.segments[0].name for base in bases
        ]


def process_enum(input: parser.EnumDecl, class_scope=None) -> None:
    '''
    Extract docstrings from an enum, including any documentation of the enum
    values.
    '''
    docstring = clean_docstring(input.doxygen)
    if docstring is not None:
        name = input.typename.segments[0].name
        if class_scope is not None:
            name = f'{class_scope}_{name}'
        DOCSTRINGS[name] = docstring
    for value in input.values:
        docstring = clean_docstring(value.doxygen)
        if docstring is not None:
            DOCSTRINGS[f'{name}_{value.name}'] = docstring


def process_function(input: parser.Function):
    '''
    Extract the docstring of a function.
    '''
    docstring = clean_docstring(input.doxygen)
    if docstring is not None:
        name = disambiguate_names(input.name.segments[0].name, DOCSTRINGS)
        if name.startswith('operator'):
            return
        DOCSTRINGS[name] = docstring


def process_variables(input: parser.Variable):
    '''
    Extract the docstring for a variable.
    '''
    docstring = clean_docstring(input.doxygen)
    if docstring is not None:
        name = input.name.segments[0].name
        DOCSTRINGS[name] = docstring


def process_namespace(namespace: parser.NamespaceScope):
    '''
    Extract all classes, enums, functions, and variables from a namespace,
    recursively.
    '''
    for input in namespace.classes:
        process_class(input)
    for input in namespace.enums:
        process_enum(input)
    for input in namespace.functions:
        process_function(input)
    for input in namespace.variables:
        process_variables(input)
    for name in namespace.namespaces:
        process_namespace(namespace.namespaces[name])


# These files don't get bindings.
FILE_BLACKLIST = [
    'not_null.hpp',
    'factory.hpp',
    'inspect.hpp',
    'tensors.hpp',
    'transform.hpp',
    'aspn.hpp',
    'compiler.hpp',
]


def process_inheritance() -> None:
    '''
    Cross references the known inheritance relationships with the missing
    docstrings, filling in missing docstrings with those from the parent
    class.
    '''
    for child in CHILD_CLASSES:
        if child not in MISSING_METHOD_DOCSTRINGS:
            continue
        for missing_method in MISSING_METHOD_DOCSTRINGS[child]:
            for parent in CHILD_CLASSES[child]:
                if f'{parent}_{missing_method}' in DOCSTRINGS:
                    DOCSTRINGS[f'{child}_{missing_method}'] = DOCSTRINGS[
                        f'{parent}_{missing_method}'
                    ]


def expand_lineage() -> None:
    '''
    Adds grandparents to child-parent mapping, recursively.
    '''
    added_grandparent = False
    for child in CHILD_CLASSES:
        for parent in CHILD_CLASSES[child]:
            if parent in CHILD_CLASSES:
                for grandparent in CHILD_CLASSES[parent]:
                    if grandparent not in CHILD_CLASSES[child]:
                        CHILD_CLASSES[child] += [grandparent]
                        added_grandparent = True
    # Check if anything changed
    if added_grandparent:
        # If something changed, iterate again.
        expand_lineage()


def preprocess_file(input: str) -> str:
    # Remove NEED_DOXYGEN_EXHALE_WORKAROUND directives, but leave
    # their wrapped contents.
    input = re.sub(
        r"#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND(.*?)#endif",
        r"\1",
        input,
        flags=re.DOTALL,
    )
    # Remove other compiler directives, including their wrapped
    # contents.
    input = re.sub(r"#\s*ifndef.*?#\s*endif", "", input, flags=re.DOTALL)
    input = re.sub(r"#\s*ifdef.*?#\s*endif", "", input, flags=re.DOTALL)
    input = re.sub(r"#define.*?\n", "", input, flags=re.DOTALL)
    # Replace doxygen-style math directives with plain LaTeX-style.
    input = input.replace('\\f$', '$')
    return input


def main() -> None:
    '''
    Extract docstrings from headers. Makes the following assumptions:

    - Build directory location is `build/src/bindings/python`, or is passed in
      as the first argument.
    - Current working directory is root project directory, or root project
      directory is passed in as the second argument.
    '''
    build_directory = 'build/src/bindings/python'
    project_directory = os.getcwd()

    if len(sys.argv) > 1:
        build_directory = sys.argv[1]
    if len(sys.argv) > 2:
        project_directory = sys.argv[2]

    files = []
    header_paths = [
        f'{project_directory}/src/navtk/**/*.hpp',
        f'{project_directory}/examples/utils/*.hpp',
    ]
    for path in header_paths:
        files.extend(glob(os.path.normpath(path), recursive=True))

    for file in files:
        basename = os.path.basename(file)
        if basename in FILE_BLACKLIST:
            continue
        with open(file, "r", encoding="utf-8") as f_handle:
            f_string = f_handle.read()

            # Remove #ifdef's, since cxxheaderparser is not a preprocessor.
            f_string = preprocess_file(f_string)

            namespace = parser.parse_string(f_string).namespace
            process_namespace(namespace)

    expand_lineage()
    process_inheritance()

    # Build output file from extracted docstrings, sandwiched by the hard-coded
    # preface and postface.
    out_str = PREFACE
    for name in DOCSTRINGS:
        docstring = DOCSTRINGS[name]
        out_str += (
            f'static const char* __doc_{name} = \nR"doc({docstring})doc";\n'
        )
    out_str += POSTFACE

    with open(f'{build_directory}/navtk_generated.hpp', 'w') as output_file:
        output_file.write(out_str)


if __name__ == '__main__':
    main()
