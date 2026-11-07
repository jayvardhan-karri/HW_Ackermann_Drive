# #############################################################################
# [FILE]   launch_prompts.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/launch_prompts.py
# ROLE  : Asks for the crucial settings of a launch file in the terminal, so
#         nobody has to know the command-line arguments. Used by the entry
#         launch files (record_lap, compare_lap, obstacle_lap, r05_lap,
#         hw_tracking, hw_check, limo_test).
#
# Rules   : a value given on the command line (name:=value) is used, never asked.
#           A missing value is asked, with numbered choices and Enter = default.
#           No terminal (scripted run): the default is used without asking.
#           Then a heading lists every setting and where it came from.
# How     : ask_first(...) returns an OpaqueFunction placed FIRST in the launch
#           description. It writes the answers into the launch configurations, so
#           everything after it (nodes, included launch files) sees them as if
#           they had been typed on the command line.
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
#   _name    a leading underscore = internal helper, only used inside this file/class
# #############################################################################

import sys

from launch.actions import OpaqueFunction

CONTROLLERS = [('pid', 'PID'), ('lqr', 'LQR'), ('mpc', 'LTV-MPC'), ('nmpc', 'NMPC')]
START_SIDES = [('auto', 'auto: top, or bottom if the robot is nearer it'), ('top', 'top straight'),
               ('bottom', 'bottom straight')]
SIM = [('false', 'real robot and mocap'), ('true', 'rehearsal with the stand-in robot (no lab)')]


# =============================================================================
# [UTIL]   class Ask
# -----------------------------------------------------------------------------
# One question (plain data holder, no behaviour). ask_first() gets a list of them.
#   name     launch argument it fills (declare it with default '' in the file)
#   label    text in the prompt and the heading
#   default  used on Enter and when there is no terminal; None = must be given
#   choices  [(value, text), ...] for a pick list, or None for a typed value
#   kind     'choice', 'many' (several of the choices, e.g. controllers),
#            'int' or 'float'
#   skip     function(configurations) -> True to leave this question out
#   strict   False: a command-line value is taken as given, even if not in the
#            choices (e.g. track_radius:=0.5, controllers:=mpc:constraint)
# =============================================================================
class Ask:
    # =========================================================================
    # [LIFE]   __init__(name, label, default, choices=None, kind, skip, strict)
    # -------------------------------------------------------------------------
    # Stores the fields described in the class banner.
    # =========================================================================
    def __init__(self, name, label, default, choices=None, kind='choice', skip=None, strict=True):
        self.name, self.label, self.default = name, label, default
        self.choices, self.kind, self.skip, self.strict = choices, kind, skip, strict


# =============================================================================
# [LIFE]   ask_first(title, questions) -> OpaqueFunction
# -----------------------------------------------------------------------------
# The action to put first in a LaunchDescription (see the file header).
# title: heading of the printed summary; questions: list of Ask.
# HOW: wraps _ask_all in an OpaqueFunction, which launch runs before the nodes.
# =============================================================================
def ask_first(title, questions):
    return OpaqueFunction(function=lambda context: _ask_all(context, title, questions))


# =============================================================================
# [UI]     _ask_all(context, title, questions)  (helper of ask_first)
# -----------------------------------------------------------------------------
# For each question: command line, else prompt, else default. Checks the value,
# writes it into the launch configurations, prints the heading.
# HOW: a question with a given value is not asked (checked only when strict);
# with no terminal and no default it raises RuntimeError. The summary lists each
# setting with its source: command line / chosen / default. Returns [] (no
# further launch actions).
# =============================================================================
def _ask_all(context, title, questions):
    cfg = context.launch_configurations
    interactive = sys.stdin is not None and sys.stdin.isatty()
    rows = []
    for q in questions:
        if q.skip is not None and q.skip(cfg):
            continue
        given = str(cfg.get(q.name, '')).strip()
        if given:
            value, source = (_check(q, given) if q.strict else given), 'command line'
        elif interactive:
            value, source = _prompt(q), 'chosen'
        elif q.default is not None:
            value, source = q.default, 'default'
        else:
            raise RuntimeError(f'{q.name}:=... is required ({q.label}); no terminal to ask in')
        cfg[q.name] = value
        rows.append((q.label, _shown(q, value), source))
    width = max(len(r[0]) for r in rows) if rows else 0
    lines = ['', '=' * 64, f'  {title}', '-' * 64]
    lines += [f'  {label:<{width}} : {shown}   ({source})' for label, shown, source in rows]
    lines += ['=' * 64, '']
    print('\n'.join(lines), flush=True)
    return []


# =============================================================================
# [UI]     _prompt(q) -> value  (helper of _ask_all)
# -----------------------------------------------------------------------------
# Shows the question until the answer is valid. Enter = default. End of input
# (Ctrl-D) = default, or an error when there is none.
# HOW: prints the numbered choices, reads a line, and gives it to _check(); a
# wrong answer prints the reason and asks again.
# =============================================================================
def _prompt(q):
    if q.choices:
        print(f'\n{q.label}:', flush=True)
        for i, (value, text) in enumerate(q.choices, 1):
            print(f'  {i}) {text}', flush=True)
    hint = ' (numbers or names, e.g. "1 4" or "pid,nmpc")' if q.kind == 'many' else ''
    default = f' [Enter = {_shown(q, q.default)}]' if q.default is not None else ''
    while True:
        try:
            answer = input(f'{q.label}{hint}{default}: ').strip()
        except EOFError:
            answer = ''
            if q.default is None:
                raise RuntimeError(f'{q.name}:=... is required ({q.label})')
        if not answer and q.default is not None:
            return q.default
        try:
            return _check(q, answer)
        except RuntimeError as e:
            print(f'  {e}', flush=True)


# =============================================================================
# [UTIL]   _check(q, text) -> value  (helper of _ask_all, _prompt)
# -----------------------------------------------------------------------------
# Accepts a choice by number or by value; for 'many', a list of them (kept in
# the order of the choices, duplicates removed). Raises RuntimeError if invalid.
# 'int' / 'float': a number more than 0. Returns text, as launch arguments are.
# =============================================================================
def _check(q, text):
    if q.kind in ('int', 'float'):
        try:
            number = int(text) if q.kind == 'int' else float(text)
        except ValueError:
            raise RuntimeError(f'{q.label}: "{text}" is not a number') from None
        if number <= 0:
            raise RuntimeError(f'{q.label}: must be more than 0')
        return str(number)
    values = [v for v, _ in q.choices]
    picked = []
    for token in text.replace(',', ' ').split() if q.kind == 'many' else [text]:
        token = token.strip().lower()
        if token.isdigit() and 1 <= int(token) <= len(values):
            token = values[int(token) - 1]
        if token not in values:
            raise RuntimeError(f'{q.label}: "{token}" is not one of {", ".join(values)}')
        picked.append(token)
    if q.kind == 'many':
        return ','.join(v for v in values if v in picked)
    return picked[0]


# =============================================================================
# [UTIL]   _shown(q, value) -> text for people  (helper of _ask_all, _prompt)
# -----------------------------------------------------------------------------
# Choice values become their names ('pid' -> 'PID'); only the part of the name
# before a ':' is shown. Typed values are shown as they are.
# =============================================================================
def _shown(q, value):
    if not q.choices:
        return value
    names = dict(q.choices)
    return ', '.join(names.get(v, v).split(':')[0] for v in str(value).split(','))
