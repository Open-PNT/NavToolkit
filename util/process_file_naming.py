#!/usr/bin/env python3

from collections import defaultdict
from functools import wraps
from itertools import takewhile
from os.path import realpath, commonpath
import re
import sys

from clang.cindex import CursorKind, TranslationUnit

from inflection import camelize, underscore
import mkdoc

NAMING_PATTERNS = {
    'lowercase': re.compile(r'^[a-z][a-z0-9]*$'),
    'camelCase': re.compile(r'^[a-z][a-zA-Z0-9]*$'),
    'PascalCase': re.compile(r'\b[A-Z]\b|^[A-Z](?=.*[a-z])[a-zA-Z0-9]*$'),
    'snake_case': re.compile(r'^[a-z][a-z0-9_]*$'),
    'LOUD_SNAKE_CASE': re.compile(r'^[A-Z][A-Z0-9_]*$'),
}


def snake_case_but_allow_capital_C(word):
    # This is based on underscore from the inflection library.
    # MIT Licensed, (c) 2012-2015 Janne Vanhala
    word = re.sub(r"([A-Z]+)([A-Z][a-z])", r'\1_\2', word)
    word = re.sub(r"([a-z\d])([A-Z])", r'\1_\2', word)
    word = word.replace("-", "_")
    return re.sub(
        r'[^C]|(?<!_)C(?!_)|(?<=_)C(?!$)(?!_)|(?<!_)(?<!^)C(?=_)',
        lambda m: m.group(0).lower(),
        word,
    )


RENAMERS = {
    'lowercase': lambda x: x.replace('_', '').lower(),
    'camelCase': camelize,
    'PascalCase': lambda x: camelize(x, True),
    'snake_case': snake_case_but_allow_capital_C,
    'LOUD_SNAKE_CASE': lambda x: underscore(x).upper(),
}

GIT_ON = True

# Symbols to leave alone because I'm assuming they're mathy
# setUp because gtest
MATH_BLACKLIST = set(
    re.split(
        r'\s+',
        """
    A a F MU W0c W0m Wukf x f expected_H expected_R PI pi tilt_P R0 P1700
""".strip(),
    )
)

# Names of things marked const or constexpr that are specifically allowed to
# not be LOUD_CASE, such as value in error_mode_assert.hpp which is mimicking
# type_traits value usage
CONST_BLACKLIST = set(
    re.split(
        r'\s+',
        """
    value
    """.strip(),
    )
)


def detect_casing(name):
    name = mkdoc.d(name)
    if any(re.finditer(r'^\d|[^\w]', name)):
        return None
    name = name.strip('_')
    return {
        key for key, pattern in NAMING_PATTERNS.items() if pattern.match(name)
    }


def node_error(node, message):
    return f'{node.spelling} {message}'


def tokens(node):
    return (node.get_definition() or node).get_tokens()


def looks_like_math(node):
    matrix_types = {'xt::', 'Vector', 'Matrix', 'eig', 'auto', 'size_t'}
    if (
        node.spelling in MATH_BLACKLIST
        or node.spelling.startswith('C_')
        or '_C_' in node.spelling
    ):
        return True
    if len(node.spelling) < 5:
        for token in tokens(node):
            if any(x in token.spelling for x in matrix_types):
                return True


def is_excepted(node):
    if node.kind == CursorKind.STRUCT_DECL:
        for token in tokens(node):
            # Exception: fmt::formatter
            if token.spelling == 'fmt' and node.spelling == 'formatter':
                return True
    return False


def tokens_to_search(node):
    all_toks = (token.spelling for token in tokens(node))
    # Only tokens appearing before name need considered
    search_toks = list(takewhile(lambda x: x != node.spelling, all_toks))
    if search_toks.count('<') > 0:
        # Strip out anything between outermost <> as consts in type specs don't
        # matter
        joined = ' '.join(search_toks)
        first = joined.find('<')
        last = joined.rfind('>')
        strp = joined[first : last + 1]
        search_toks = joined.replace(strp, '').split(' ')
    return search_toks


def is_namespace_scope(node):
    return node.lexical_parent.kind == CursorKind.NAMESPACE


def is_in_anonymous_namespace(node):
    return is_namespace_scope(node) and not node.lexical_parent.spelling


def marked_constexpr(tokens):
    return 'constexpr' in tokens


def marked_extern(tokens):
    return 'extern' in tokens


def marked_static(tokens):
    return 'static' in tokens


def marked_const(tokens):
    return 'const' in tokens


def parent_is_classy(node):
    parent = node.lexical_parent and node.lexical_parent.kind
    return parent in {
        CursorKind.CLASS_DECL,
        CursorKind.STRUCT_DECL,
        CursorKind.CLASS_TEMPLATE,
    }


def is_forward_decl(node):
    parent = node.lexical_parent and node.lexical_parent.kind
    if not parent:
        return False
    return (
        node.kind in {CursorKind.VAR_DECL, CursorKind.FIELD_DECL}
        and parent == CursorKind.TRANSLATION_UNIT
    )


# Marked static const in class, or
# Any constexpr, or
# Anything #define (TODO), or
# Anything in namespace scope and marked extern
# Anything in namespace scope and marked static
# Anything forward declared in a cpp file
def should_be_shout(node):
    if (
        node.kind not in {CursorKind.VAR_DECL, CursorKind.FIELD_DECL}
        or node.spelling in CONST_BLACKLIST
    ):
        return False
    search_toks = tokens_to_search(node)
    return (
        (
            parent_is_classy(node)
            and marked_static(search_toks)
            and marked_const(search_toks)
        )
        or marked_constexpr(search_toks)
        or (
            is_namespace_scope(node)
            and (
                marked_extern(search_toks)
                or marked_static(search_toks)
                or marked_const(search_toks)
            )
        )
        or (is_forward_decl(node) and marked_const(search_toks))
    )


def pick_naming_rule(node):
    if node.spelling.startswith('operator'):
        return None, None
    # Exempt not_null from default naming rules
    if node.spelling.startswith('not_null'):
        return None, None
    rule = {
        CursorKind.CLASS_TEMPLATE: 'PascalCase',
        CursorKind.FUNCTION_DECL: 'snake_case',
        CursorKind.FUNCTION_TEMPLATE: 'snake_case',
        CursorKind.CXX_METHOD: 'snake_case',
        CursorKind.NAMESPACE: 'snake_case',
        CursorKind.TYPEDEF_DECL: 'PascalCase',
        CursorKind.PARM_DECL: 'snake_case',
        CursorKind.VAR_DECL: 'snake_case',
        CursorKind.FIELD_DECL: 'snake_case',
        CursorKind.STRUCT_DECL: 'PascalCase',
        CursorKind.CLASS_DECL: 'PascalCase',
    }.get(node.kind, None)

    if rule is None:
        return None, None

    # Check for other exceptions
    if is_excepted(node):
        return None, None

    # Ignore functions with names like 'get_C_blah_blah' that work with DCMs
    if node.kind in {
        CursorKind.FUNCTION_DECL,
        CursorKind.CXX_METHOD,
    } and looks_like_math(node):
        return None, None

    if should_be_shout(node):
        return 'LOUD_SNAKE_CASE', {'LOUD_SNAKE_CASE'}

    if node.kind in {
        CursorKind.VAR_DECL,
        CursorKind.PARM_DECL,
        CursorKind.FIELD_DECL,
    }:
        parent = node.lexical_parent and node.lexical_parent.kind
        accept = {rule}
        if parent in {
            CursorKind.CXX_METHOD,
            CursorKind.FUNCTION_DECL,
            CursorKind.FUNCTION_TEMPLATE,
            CursorKind.NAMESPACE,
            CursorKind.CLASS_DECL,
            CursorKind.STRUCT_DECL,
            CursorKind.CONSTRUCTOR,
            CursorKind.CLASS_TEMPLATE,
        }:
            # as a special case, don't try to rename member variables that
            # look math-y
            if looks_like_math(node):
                return None, None

            # Loosen naming rules in anonymous namespaces to allow for
            # file-level global types
            if is_in_anonymous_namespace(node):
                return 'snake_case', {rule, 'LOUD_SNAKE_CASE'}
        return 'snake_case', accept
    # Don't screw with overrides, since they have to match their base class
    if node.kind in {CursorKind.CXX_METHOD, CursorKind.FUNCTION_DECL} and any(
        t.spelling in {'override'} for t in tokens(node)
    ):
        return None, None
    return rule, {rule}


def is_in_folder(path, folder):
    path = realpath(path)
    folder = realpath(folder)
    return commonpath([folder]) == commonpath([folder, path])


def memoize(fn):
    memo = {}

    @wraps(fn)
    def decorator(*a, **kw):
        key = a + tuple(sorted(kw.items()))
        if key in memo:
            return memo[key]
        else:
            out = fn(*a, **kw)
            memo[key] = out
            return out

    return decorator


@memoize
def is_project_source_file(path):
    return is_in_folder(path, '.') and not is_in_folder(path, 'subprojects')


def node_is_in_our_code(node):
    if node.location and node.location.file:
        path = node.location.file.name
        return is_project_source_file(path)


def tolist(x):
    if isinstance(x, dict):
        x = list(x.values())
    return x if isinstance(x, (list, tuple)) else [x]


def vectorize(fn):
    @wraps(fn)
    def decorator(*a, **kw):
        if any(isinstance(x, (list, tuple, dict)) for x in a):
            aarr = [tolist(x) for x in a]
            return [fn(*row, **kw) for row in zip(*aarr)]
        return fn(*a, **kw)

    return decorator


@vectorize
def whereis(node):
    if node.location and node.location.file:
        return f'{node.location.file.name}:{node.location.line}'


@vectorize
def whatis(node):
    return '%s: %r %s: %s' % (
        whereis(node),
        node.kind,
        node.spelling,
        ' '.join(it.spelling for it in node.get_tokens()),
    )


def _rule_error(message, node, cursors):
    previous = whatis(cursors[node.spelling])
    msg = f'{message}\n  Current Token: {whatis(node)}\n  Previous Tokens:\n\t'
    raise ValueError(msg + '\n\t'.join(previous))


def _apply_rule(node, renames, cursors, collisions, preserved):
    # filter out implicit/macro-defined stuff
    if not any(t.spelling.strip() for t in node.get_tokens()):
        return
    rule, accept = pick_naming_rule(node)
    if rule is not None:
        renamer = RENAMERS[rule]
        if not accept & (detect_casing(node.spelling) or set()):
            rename = renamer(node.spelling)
            if node.spelling != rename:
                old = renames.get(node.spelling, rename)
                if old != rename:
                    _rule_error(
                        f"Trying to rename {node.spelling} to {rename}"
                        f" but it's already been renamed to {old}",
                        node,
                        cursors,
                    )
                    collisions.add(node.spelling)
                elif node.spelling in preserved:
                    _rule_error(
                        f"Trying to rename {node.spelling} to {rename} but "
                        f"a previous instance of the name was preserved",
                        node,
                        cursors,
                    )
                    collisions.add(node.spelling)
                else:
                    renames[node.spelling] = rename
                return
        rename = renames.get(node.spelling, None)
        if rename:
            _rule_error(
                f"Would preserve {node.spelling} but it has previously been "
                f"renamed to {rename}",
                node,
                cursors,
            )
        preserved.add(node.spelling)


def extract(filename, node, prefix, output):
    if not prefix:
        prefix = ()
    if not output:
        output.append(({}, defaultdict(dict), set(), set()))
    cursors = output[0][1]
    if node_is_in_our_code(node) is False:  # "None" means we don't know
        return
    if node.kind in mkdoc.RECURSE_LIST:
        spelling = mkdoc.d(node.spelling)
        if node.kind == CursorKind.TRANSLATION_UNIT:
            subfix = prefix
        else:
            subfix = prefix + (spelling,)
        for it in node.get_children():
            extract(filename, it, subfix, output)
    _apply_rule(node, *output[0])
    cursors[node.spelling][whereis(node)] = node


# This monkey-patch lets us use extract_all with our own implementation of
# extract, letting mkdoc configure clang and handle threading and file IO.
mkdoc.extract = extract
mkdoc.RECURSE_LIST.extend(
    [
        CursorKind.CXX_METHOD,
        CursorKind.FUNCTION_DECL,
        CursorKind.FUNCTION_TEMPLATE,
        CursorKind.COMPOUND_STMT,
        CursorKind.DECL_STMT,
        CursorKind.FOR_STMT,
        CursorKind.CONSTRUCTOR,
    ]
)

# Configure mkdoc's call to cindex to PARSE_INCOMPLETE, which removes the
# misleading "#pragma once in header file" warning
mkdoc.PARSE_OPTIONS = TranslationUnit.PARSE_INCOMPLETE

# uncomment next line to get a real error message in the event of a hardexit.
# mkdoc.ExtractionThread.start = mkdoc.ExtractionThread.run


def main(args):
    renames, cursors, *_ = mkdoc.extract_all(args)[0]
    exit_status = 0
    for current, should in renames.items():
        exit_status = 1
        for item in whatis(cursors[current]):
            print(f'{item}: should be {should}')
    sys.exit(exit_status)


if __name__ == '__main__':
    sys.argv.append('-hardexit')  # awful, awful hack
    main(sys.argv[1:])
