#!/usr/bin/env python3
"""
This program reads C++ code snippets containing matrix literals and reformats
them into space-aligned rows/columns.

For example, `Matrix foo{{10.2,4},{-30,1.34}};` becomes:

    // clang-format off
    Matrix foo{{ 10.2 ,   4   },
               {-30   ,   1.34}};
    // clang-format on

Enter your matrix literals on stdin and press Ctrl+D when you're done.
"""


import sys
from buildkit.parsing import (
    advance_to,
    indent_of,
    split_carefully,
    is_number,
    decimal_places,
    left_of_decimal,
)


def non_number_length(text):
    '''
    Return len(text.strip()) if text does not represent a number; otherwise 0.
    '''

    text = text.strip()
    return 0 if is_number(text) else len(text)


def compute_cell_size(term, count=len):
    '''
    If the given term introduces nested braces, return the largest value
    of `count(it)` for each of the comma-separated elements within the nested
    braces. Otherwise return the value of `count(term.strip())`
    '''

    term = term.strip()
    if term.startswith('{') and term.endswith('}'):
        subterms = [a for a, b in split_carefully(term[1:-1], ',')]
        return max(compute_cell_size(t, count) for t in subterms)
    return count(term)


def compute_cell_sizes(terms):
    '''
    Given a list of matrix cells, compute the minimum size necessary for
    each cell, returning a 3-tuple of (characters left of the decimal,
    characters right of the decimal, total characters).
    '''

    left, right, cell = [
        max((compute_cell_size(t, x) for t in terms), default=0)
        for x in (left_of_decimal, decimal_places, compute_cell_size)
    ]
    cell = max(cell, left + right + (1 if right else 0))
    return left, right, cell


def beautify_brace_init(expr, min_sizes=(0, 0, 0)):
    '''
    Reformat a single matrix initializer.
    '''

    itr = iter(expr)
    preamble = ''.join(advance_to(itr, '{'))
    linebreak = "\n%s" % ''.join(
        '\t' if c == '\t' else ' ' for c in preamble.split("\n")[-1]
    )
    terms = []
    epilogue = ''
    for term, close in split_carefully(itr, ',}'):
        terms.append(term)
        if close == '}':
            epilogue = '}'
            break
    epilogue += ''.join(itr)
    sizes = compute_cell_sizes(terms)
    if not sizes[-1]:
        # nothing to align with so bail
        return expr
    sizes = [max(*it) for it in zip(min_sizes, sizes)]
    nesting = False
    new_terms = []
    for term in terms:
        term = term.strip()
        if term.startswith('{') and term.endswith('}'):
            nesting = True
            term = beautify_brace_init(term, sizes)
        elif is_number(term):
            places = decimal_places(term)
            if sizes[1] and '.' not in term:
                term += " "
            term += " " * (sizes[1] - places)
            term = " " * (sizes[2] - len(term)) + term
        else:
            term += " " * (sizes[2] - len(term))
        new_terms.append(term)
    main = (f',{linebreak}' if nesting else ", ").join(new_terms)
    return preamble + main + epilogue


def pretty_literal(text):
    '''
    Yield a series of reformatted, space-aligned matrix literals based on
    the input text.
    '''

    yielded_clang_tag = False
    need_newline = False
    indent = ''
    all_out_thusfar = ''
    for command, terminator in split_carefully(text, ';'):
        chunk = command + terminator
        out = beautify_brace_init(chunk)
        all_out_thusfar += out
        if out != chunk and not yielded_clang_tag:
            if need_newline:
                yield '\n'
            indent = indent_of(all_out_thusfar)
            yield f'{indent}// clang-format off\n'
            yielded_clang_tag = True
        yield out
        need_newline = not out.rstrip(' \t').endswith('\n')
    if yielded_clang_tag:
        if need_newline:
            yield '\n'
        yield f'{indent}// clang-format on\n'


def lines_to_chars(src):
    '''Given an array of strings, yield each character in each string.'''
    for element in src:
        yield from element


if __name__ == '__main__':
    if sys.stdin.isatty():
        sys.stderr.writelines(
            [sys.modules[__name__].__doc__.lstrip(), "=" * 79 + "\n"]
        )
        sys.stderr.flush()
    for literal in pretty_literal(lines_to_chars(sys.stdin)):
        sys.stdout.write(literal)
        sys.stdout.flush()
