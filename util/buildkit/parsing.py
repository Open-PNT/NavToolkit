'''
Utility/helper functions for parsing code.
'''


def indent_of(code):
    r'''
    Return a string that matches the leading indentation of a given
    piece of code, defined as the leading whitespace common to all
    lines in the code.

    >>> indent_of('\tif (x) {\n\t\treturn;\n\t}')
    '\t'
    '''
    shortest = None
    for line in code.split("\n"):
        indent = ''
        for char in line:
            if char in "\t ":
                indent += char
            elif shortest is None or len(indent) < len(shortest):
                shortest = indent
                break
    return shortest or ''


def escaped_chars(text):
    r'''
    Yield each character in the given text, one at a time, except treat
    escape sequences as a single character.

    >>> tuple(escaped_chars(r'a\nb'))
    ('a', '\\n', 'b')

    This function is implemented as a lazy generator. If the text passed
    in is an iterator, it will be consumed one character at a time.

    >>> itr = iter('a\nb\nc')
    >>> ec = iter(escaped_chars(itr))
    >>> next(ec)
    'a'
    >>> next(ec)
    '\n'
    >>> ''.join(itr)
    'b\nc'
    '''

    char_escaped = False
    for char in text:
        if char_escaped:
            char_escaped = False
            yield f'\\{char}'
        elif char == '\\':
            char_escaped = True
        else:
            yield char
    if char_escaped:
        yield '\\'


BALANCE = dict(["{}", "()", "[]", '""', "''"])


def advance_to(text, token):
    '''
    Iterate through the given sequence of characters looking for the given
    token, yielding each character up to and including the token. Use the
    global BALANCE dict to do paren-balancing, and only stop when the token
    is reached outside of a quote, brace, paren, or bracket.

    >>> ''.join(advance_to('foo (bar "baz)" bad) bah', 'a'))
    'foo (bar "baz)" bad) ba'

    This function is implemented as a lazy generator. If the text passed
    in is an iterator, it will be consumed one character at a time.
    '''

    stack = [token]
    for tok in escaped_chars(text):
        yield tok
        closing_char = BALANCE.get(tok, None)
        if tok == stack[-1]:
            stack.pop()
            if not stack:
                break
        elif closing_char is not None:
            stack.append(closing_char)


def split_carefully(text, separators):
    '''
    Yield (substring, separator) for substrings of the given text, delimited
    by any character in the list of possible separators, but treating
    expressions in parens, brackets, braces or quotes as atomic. Each
    substring yielded will be accompanied by its trailing separator, or the
    empty string if it's the last substring in the input text.

    >>> list(split_carefully('foo. bar! baz', {'.', '!'}))
    [('foo', '.'), (' bar', '!'), (' baz', '')]
    >>> list(split_carefully('foo. (bar. baz.) bah', '.'))
    [('foo', '.'), (' (bar. baz.) bah', '')]

    This function is implemented as a lazy generator. If the text passed in
    is an iterator it will be consumed one character at a time.
    '''
    buf = []
    itr = iter(escaped_chars(text))
    for char in itr:
        buf.append(char)
        close = BALANCE.get(char, None)
        if char in separators:
            yield ''.join(buf[:-1]), char
            buf = []
        elif close:
            buf.extend(advance_to(itr, close))
    if buf:
        yield ''.join(buf), ''


def is_number(text):
    '''
    Return True iff the given text looks like it's a number. This function
    predicts whether `float(text.strip())` will return an error.
    '''
    return not set(text.strip()) - set("-0987654321.eE")


def decimal_places(text):
    '''
    If the given value looks like a number, return the count of characters
    after the decimal point. Otherwise return 0.

    >>> decimal_places('1')
    0
    >>> decimal_places('1.0')
    1
    >>> decimal_places('1.0e4')
    3
    >>> decimal_places('foo.bar')
    0
    '''
    text = str(text).strip()
    if is_number(text) and "." in text:
        return len(text.split(".")[-1])
    return 0


def left_of_decimal(text):
    '''
    If the given value looks like a number, return the count of characters
    before the decimal point. Otherwise return 0.

    >>> left_of_decimal('1')
    1
    >>> left_of_decimal('.1')
    0
    >>> left_of_decimal('10.2e4')
    2
    >>> left_of_decimal('foo.bar')
    0
    '''
    text = str(text).strip()
    if is_number(text):
        return len(text.rsplit(".", 1)[0])
    return 0
