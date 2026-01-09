'''
Routines and classes for interacting with the user from our Python scripts.
'''

# pylint: disable=too-few-public-methods,unused-argument,no-self-use

import sys
from os import environ, fstat
import re
import stat
from shutil import get_terminal_size

# Detects whether we're running from within a meson subprocess based on whether
# the run_command() env vars are present.
IN_MESON = (
    bool({'MESON_SOURCE_ROOT', 'MESON_BUILD_ROOT'} & set(environ))
    and not sys.stdout.isatty()
)

# ANSI four-bit colors by number
ANSI_COLOR_CODES = {
    c: n
    for n, c in enumerate(
        ('black', 'red', 'green', 'yellow', 'blue', 'magenta', 'cyan', 'white')
    )
}

# The ansi color codes above get added to one of these to decide foreground
# vs background
FOREGROUND_COLOR_BASE = 30
BACKGROUND_COLOR_BASE = 40

# How to indent inner ViewFrames
INDENTATION = '   '

# Which ANSI color name to use for different kinds of message
MESSAGE_TYPE_COLORS = {
    v: k
    for k, vv in {
        'red': ('fail', 'error'),
        'yellow': ('warn', 'warning'),
        'green': ('ok',),
        'cyan': ('info', 'skip'),
    }.items()
    for v in vv
}


def _b(target, text):
    '''
    Convert the given text to/from bytes to match the type of the target.

    Used below to match unicode vs bytes on a lot of string manipulation
    routines.
    '''
    if isinstance(text, int):
        text = chr(text)
    if isinstance(target, bytes):
        return text.encode('u8')
    if isinstance(text, bytes):
        return text.decode('u8', 'ignore')
    return text


def _compose_ansi_color(color, base=FOREGROUND_COLOR_BASE):
    code = int(ANSI_COLOR_CODES.get(color, color))
    if code <= 10:
        code += base
    return str(code)


def compose_ansi(
    foreground=None, bold=False, background=None, title=None, underline=False
):
    '''
    Return an ANSI escape sequence corresponding to the requested formatting.

    Called with no arguments, return the ANSI reset escape sequence.
    '''

    terms = []
    if foreground is not None:
        terms.append(_compose_ansi_color(foreground))
    if background is not None:
        terms.append(_compose_ansi_color(background, BACKGROUND_COLOR_BASE))
    if bold:
        terms.append('1')
    if underline:
        terms.append('4')
    color = f'\033[{";".join(terms or ["0"])}m'
    if title is not None:
        title = f'\033]0;{title}\007'
    if title and not terms:
        return title
    return (title or '') + color


class ConsoleOutput:
    '''
    Wrapper for a python output stream, such as sys.stderr or sys.stdout,
    that provides special features like ANSI colors, folding, auto-indent,
    and word wrap.

    The module constant UI is an instance of this. Use UI.write to
    communicate with the user.

    This class works by sending all incoming calls to `write` to a series
    of filters (self.filters) which are defined as separate classes.
    '''

    column_count = 0
    visible = False

    def __init__(self, stream):
        self.stream = stream
        try:
            self.is_tty = stream.isatty()
        except AttributeError:
            self.is_tty = False
        try:
            self.is_pipe = fstat(stream.fileno()).st_mode & stat.S_IFIFO
        except AttributeError:
            self.is_pipe = None
        self.visible = self.is_tty and not (IN_MESON or self.is_pipe)
        self.filters = [
            ListExpander(),
            FoldingFilter(),
            Indenter(),
            CRColumnLimiter(),
            OptionsAsFormat(),
        ]

    def write(self, *data, **kw):
        '''
        Run the given data through the view filters, then write the
        modified result to the underlying stream.

        Each item in the data stream can be text (in which case that
        text is written to the console) or a dict with formatting
        options (such as `{'bold': True}`) which will be converted into
        ANSI format codes. See the various filter classes (defined below)
        for examples of the kinds of things you can write.
        '''
        for console_filter in self.filters:
            data = console_filter.filter(self, data, kw)
        data = list(data)
        for datum in data:
            if not isinstance(datum, dict):
                self.stream.write(_b('', datum))
        self.stream.flush()

    def write_status(self, text, status=None):
        '''
        Write a standard status line
        '''
        if IN_MESON:
            return
        self.clearline()
        if status:
            color = MESSAGE_TYPE_COLORS[status]
            if status == 'ok':
                status = ' ok '
            writing = [
                {'bold': True},
                '[',
                {'foreground': color},
                status,
                {},
                {'bold': True},
                '] ',
            ]
        else:
            writing = [{'bold': True}, '[....] ']
        writing.extend(
            [
                ({} if status else {'bold': True, 'underline': True}),
                text,
                {},
                ('\n' if status else '\r'),
            ]
        )
        self.write(*writing, constrain=not status)

    def message(self, kind, text, **kw):
        '''
        Write a message using a colorful redhat-style [ ok ] status field.
        '''
        foreground_color = kw.pop(
            'foreground', MESSAGE_TYPE_COLORS.get(kind.lower(), 'red')
        )
        leader = kind.upper()
        if IN_MESON:
            leader += '\0'
        else:
            leader += ' ' * max(0, 8 - len(leader))
        self.write(leader, foreground=foreground_color, bold=True)
        self.write(f'{text!s}\n', foreground=foreground_color)

    def clearline(self):
        '''
        If this is a visible terminal, use whitespace and \r to clear the
        current output line. Otherwise, write a \n.
        '''
        self.write(f'\r{" " * 1000}\r')


class AtomicFilter:
    '''
    Base class for output filters that can process ConsoleOutput#write
    arguments one-at-a-time, rather than needing to handle the entire
    write call at once.
    '''

    def filter(self, console, data, options):
        '''
        Modifies the incoming stream by passing each of its elements to
        either filter_format or filter_text based on its type.
        '''
        for datum in data:
            # pylint: disable=assignment-from-no-return
            if isinstance(datum, dict):
                stream = self.filter_format(console, datum, options)
            else:
                stream = self.filter_text(console, datum, options)
            yield from stream or [datum]

    def filter_format(self, console, datum, options):
        '''Transform a single formatting dict in a write call.'''

    def filter_text(self, console, datum, options):
        '''Transform a single piece of text in a write call.'''


class CRColumnLimiter(AtomicFilter):
    r'''
    ConsoleOutput filter that prevents any `\r`-terminated message from
    exceeding the width of the console, to prevent line wrapping from
    resulting in duplicate/broken `\r` replacements.
    '''

    column_count = 0
    column_limit = get_terminal_size().columns - 1

    def filter_text(self, console, datum, options):
        r'''
        Maintain a running column count and prevent any line ending with `\r`
        from exceeding the width of the terminal.
        '''
        if not console.visible:
            if datum.strip(_b(datum, '\r ')):
                datum = re.sub(_b(datum, r'\r\s*'), '...\n', datum)
                if _b(datum, '\n') in datum:
                    self.column_count = len(datum.splitlines()[-1])
                else:
                    self.column_count += len(datum)
                yield datum
            elif self.column_count and _b(datum, '\r') in datum:
                yield '\n'
                self.column_count = 0
            return
        for line in datum.splitlines(keepends=True):
            eol = _b(
                line, line[-1] if line and line[-1] in _b(line, '\r\n') else ''
            )
            line = line.rstrip(eol)
            new_count = self.column_count + len(line)
            limiting = eol == _b(eol, '\r') or options.get('constrain', 0)
            if limiting and new_count > self.column_limit:
                split = self.column_limit - self.column_count - 2
                line = line[:split] + _b(
                    line, " »" if line[split:].strip() else ''
                )
                new_count = self.column_count + len(line)
            if limiting and new_count > self.column_limit:
                new_count = self.column_limit
                line = _b(line, '')
            yield line + eol
            if eol:
                self.column_count = 0
            else:
                self.column_count = new_count


class Indenter(AtomicFilter):
    '''
    ConsoleOutput filter that adds space after all incoming newlines,
    allowing you to automatically indent the information you're writing.
    '''

    indent = 0

    def filter_format(self, console, datum, options):
        '''Consume the 'indent' format parameter.'''
        self.indent += datum.pop('indent', 0)

    def filter_text(self, console, datum, options):
        '''
        Append whitespace after newlines, corresponding to the indent level.
        '''
        indent = self.indent + options.get('indent', 0)
        if not IN_MESON:
            for char in '\r\n':
                datum = datum.replace(
                    _b(datum, char), _b(datum, f'{char}{INDENTATION * indent}')
                )
        yield datum


class OptionsAsFormat:
    '''
    ConsoleOutput filter that converts dicts describing formatting to
    ANSI color codes, for example UI.write({'bold': True}) will, if the
    output is a TTY, write the ASCII color code for bold.

    Allowable keys to the dicts correspond to keyword arguments for the
    compose_ansi function.
    '''

    def filter(self, console, data, options):
        '''Modify the data stream, converting dicts to ansi format codes'''
        format_keys = {
            'foreground',
            'background',
            'bold',
            'title',
            'underline',
        }
        changed_format = False
        for source in [[options], data]:
            for item in source:
                if isinstance(item, dict):
                    formatting = {
                        k: v for k, v in item.items() if k in format_keys
                    }
                    if (formatting or not item) and console.visible:
                        yield compose_ansi(**formatting)
                        changed_format = True
                yield item
        if changed_format:
            yield compose_ansi()


class FoldingFilter:
    '''
    ConsoleOutput filter that listens for a `fold` keyword argument.

        UI.write(fold='push') suppresses any further output until...
        UI.write(fold='pop') dumps all the output that was hidden or...
        UI.write(fold='drop') deletes the cache.
    '''

    def __init__(self):
        self.buf = []

    def filter(self, console, data, options):
        '''Modify the data stream, possibly hiding its content.'''
        action = options.get('fold', '')
        if not data and {'title'} == set(options):
            yield from data
        elif action == 'push':
            self.buf.append([])
        elif action == 'pop':
            for cached_data, cached_options in self.buf.pop():
                console.write(*cached_data, **cached_options)
        elif action == 'drop':
            self.buf.pop()
        elif self.buf and self.buf[-1] and self.buf[-1][-1][1] == options:
            self.buf[-1][-1][0].extend(data)
        elif self.buf:
            self.buf[-1].append((list(data), options))
        else:
            yield from data


class ListExpander:
    '''
    ConsoleOutput filter that flattens lists, such that

        UI.write(['a', 'b'], 'c')

    is equivalent to

        UI.write('a', 'b', 'c')
    '''

    def filter(self, console, data, options):
        '''Modify the data stream, flattening lists.'''
        for item in data:
            if isinstance(item, (list, tuple)):
                yield from item
            else:
                yield item


UI = ConsoleOutput(sys.stderr)


def write_title(frame):
    '''
    Update the ANSI console title to a |-separated list of ViewFrames,
    starting with the given frame and including all the containing frames.

    If frame is null, clear the ANSI console title.
    '''
    title = []
    while frame is not None:
        if frame.hide_inner:
            title.clear()
        if frame.title:
            title.append(frame.title)
        frame = frame.parent
    UI.write(title=' | '.join(reversed(title)))


class ViewFrame:
    '''
    A context manager that writes its status to the console using
    redhat-style [ ok ] / [fail] markers and ANSI colors.
    '''

    current = None
    parent = None
    hide_inner = False
    is_open = False
    action = None
    title = None
    catch = ()
    allow = ()
    indent = 1
    finish_status = 'ok'
    _unhide = False
    _errors_written = set()

    def __init__(self, **kw):
        for key, value in kw.items():
            setattr(self, key, value)
        if 'title' not in kw:
            self.title = self.action

    def __enter__(self):
        self.parent = ViewFrame.current
        ViewFrame.current = self
        if self.action:
            UI.write_status(self.action)
            UI.write({'indent': self.indent})
        if self.hide_inner:
            UI.write(fold='push')
            self._unhide = 'drop'
        if not IN_MESON and self.action:
            write_title(self)
            UI.write('\n')
        return self

    def __exit__(self, exception_type, value, traceback):
        if exception_type:
            msg = 'warning'
            if not isinstance(value, self.allow):
                self.finish_status = 'fail'
                msg = 'error'
            if value not in self._errors_written:
                UI.message(msg, value)
                self._errors_written.add(value)
        notok = self.finish_status != 'ok'
        self.unhide_immediately(notok and 'pop')
        if self.parent and notok:
            self.parent.unhide_immediately('pop')
        if self.action:
            UI.write({'indent': -self.indent})
        UI.write_status(self.action, self.finish_status)
        ViewFrame.current = self.parent
        if not IN_MESON:
            write_title(ViewFrame.current)
        return True if self.catch is True else isinstance(value, self.catch)

    def unhide_immediately(self, unhide_action=None):
        '''
        If this ViewFrame is suppressing console output, immediately stop
        doing that. Called automatically by __exit__.
        '''
        if self._unhide:
            UI.write(fold=unhide_action or self._unhide)
            self._unhide = False


def warn(message, **kw):
    '''
    Write a beautified version of the message to the console and mark
    the current ViewFrame (if any) as finish_status='warn'
    '''
    if ViewFrame.current and ViewFrame.current.finish_status == 'ok':
        ViewFrame.current.finish_status = 'warn'
    return UI.message('warning', message, **kw)


def error(message, **kw):
    '''
    Write a beautified version of the message to the console and mark
    the current ViewFrame (if any) as finish_status='fail'
    '''
    if ViewFrame.current and ViewFrame.current.finish_status != 'fail':
        ViewFrame.current.finish_status = 'fail'
    return UI.message('error', message, **kw)
