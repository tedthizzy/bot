"""I-17's checker: no host timestamp is ever compared against the MCU clock.

ARCHITECTURE principle 5 states it absolutely -- "No host clock is ever compared
against the MCU clock. The one host value that crosses the link
(``P.pi_mono_us``) is stored and echoed as an opaque token, never read as a
time."  §5.1 allow-lists that one echo field.

The check this replaces was two regexes over single lines.  It required the two
clock tokens to sit next to each other with at most a couple of operator
characters between them, so it missed every spelling with a call, a cast, a
member access or a unit conversion in the way -- which is every natural
spelling::

    age_ms = (time.monotonic_ns() - frame.mcu_us * 1000) / 1e6
    latency = now_mono_ns - telemetry.mcu_us
    skew = self._clock() - frame.mcu_us
    int64_t skew = (int64_t)now_us - (int64_t)host_stamp_us;

and its allow-list exempted every line containing the substring ``pi_mono_us``,
which exempts ``drift = frame.mcu_us - pi_mono_us`` -- a direct comparison of
the two clocks and exactly the violation the invariant exists to catch.

So this does not match adjacency.  It finds the *operands* of each arithmetic or
comparison operator -- through ``ast`` in Python, through a token scan in C --
and flags any operator one of whose operands carries an MCU-clock name while
another carries a host-clock name, at any nesting.
"""

from __future__ import annotations

import ast
import re
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

__all__ = ["Crossing", "check_c", "check_python", "check_tree"]

MCU_CLOCK = re.compile(r"mcu_us|esp_timer_get_time")
"""Names carrying the MCU's own clock.  ``now_us`` joins this set in C only
(see :func:`check_c`): in firmware it is ``esp_timer_get_time()``, and in Python
it is an ordinary local."""

HOST_CLOCK = re.compile(
    r"mono_ns|mono_us|monotonic|perf_counter|clock_gettime|_clock\b|host_now"
    r"|host_\w*(?:stamp|time|clock|mono|_us|_ns|_ms)"
)
"""Names carrying a Pi clock.  ``pi_mono_us`` and ``echo_pi_mono_us`` match
here on purpose: §5.1 makes that field a host value both sides of the link, so
pairing it with an MCU stamp is the violation, not the allowance.
``host_boot_id`` deliberately does not match -- it is an identity, not a time."""

ALLOW: frozenset[tuple[str, str]] = frozenset(
    {("packages/rover_robotd/link.py", "Link._note_pong")}
)
"""The one allowance of §5.1, keyed by repo-relative file and qualified symbol
rather than by a substring anywhere on a line.  ``_note_pong`` is where the
``O`` frame's echo is turned back into an RTT, entirely on the Pi's own clock."""

_ARITH = (ast.Add, ast.Sub, ast.Mult, ast.Div, ast.FloorDiv, ast.Mod, ast.Pow)


@dataclass(frozen=True)
class Crossing:
    """One operator with an MCU operand and a host operand."""

    path: str
    line: int
    text: str
    mcu: str
    host: str

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: {self.mcu} against {self.host}: {self.text}"


def _classify(names: set[str], mcu_extra: re.Pattern[str] | None) -> tuple[str, str]:
    """The first MCU name and the first host name in a set, or empty strings."""
    mcu = sorted(
        n for n in names if MCU_CLOCK.search(n) or (mcu_extra and mcu_extra.search(n))
    )
    host = sorted(n for n in names if HOST_CLOCK.search(n))
    return (mcu[0] if mcu else ""), (host[0] if host else "")


# ---------------------------------------------------------------------------
# Python
# ---------------------------------------------------------------------------


def _py_names(node: ast.AST) -> set[str]:
    """Every identifier in a subtree, attributes included.

    ``self._clock()`` yields ``{"self", "_clock"}`` and ``frame.mcu_us`` yields
    ``{"frame", "mcu_us"}``, so a call, a cast or a member access between the
    two clocks costs the check nothing.
    """
    out: set[str] = set()
    for sub in ast.walk(node):
        if isinstance(sub, ast.Name):
            out.add(sub.id)
        elif isinstance(sub, ast.Attribute):
            out.add(sub.attr)
    return out


class _PyScan(ast.NodeVisitor):
    def __init__(self, path: str, lines: list[str]) -> None:
        self.path = path
        self.lines = lines
        self.scope: list[str] = []
        self.found: dict[int, Crossing] = {}

    def _scoped(self, node: ast.AST) -> None:
        self.scope.append(getattr(node, "name", "?"))
        self.generic_visit(node)
        self.scope.pop()

    visit_ClassDef = _scoped
    visit_FunctionDef = _scoped
    visit_AsyncFunctionDef = _scoped

    def _operands(self, node: ast.AST) -> list[ast.AST]:
        if isinstance(node, ast.BinOp) and isinstance(node.op, _ARITH):
            return [node.left, node.right]
        if isinstance(node, ast.Compare):
            return [node.left, *node.comparators]
        return []

    def _record(self, node: ast.AST) -> None:
        operands = self._operands(node)
        if not operands:
            return
        if (self.path, ".".join(self.scope)) in ALLOW:
            return
        names: set[str] = set()
        for operand in operands:
            names |= _py_names(operand)
        mcu, host = _classify(names, None)
        if not mcu or not host:
            return
        line = getattr(node, "lineno", 0)
        text = self.lines[line - 1].strip() if 0 < line <= len(self.lines) else ""
        self.found.setdefault(line, Crossing(self.path, line, text[:90], mcu, host))

    def visit_BinOp(self, node: ast.BinOp) -> None:
        self._record(node)
        self.generic_visit(node)

    def visit_Compare(self, node: ast.Compare) -> None:
        self._record(node)
        self.generic_visit(node)


def check_python(source: str, path: str = "<memory>") -> list[Crossing]:
    """Every clock crossing in one Python source, ordered by line."""
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return []
    scan = _PyScan(path, source.splitlines())
    scan.visit(tree)
    return [scan.found[line] for line in sorted(scan.found)]


# ---------------------------------------------------------------------------
# C
# ---------------------------------------------------------------------------

_C_NOISE = re.compile(
    r"/\*.*?\*/|//[^\n]*|\"(?:\\.|[^\"\\])*\"|'(?:\\.|[^'\\])*'", re.S
)
_C_MCU_EXTRA = re.compile(r"\bnow_us\b")
_C_TOKEN = re.compile(
    r"->|\+\+|--|<<=?|>>=?|[-+*/%<>=!]=|&&|\|\||[-+*/%<>]|[A-Za-z_][A-Za-z0-9_]*"
)
_C_ARITH = frozenset({"+", "-", "*", "/", "%", "<", ">", "<=", ">=", "==", "!="})


def _blank(match: re.Match[str]) -> str:
    """Replace a comment or literal with spaces, keeping every newline so line
    numbers survive."""
    return re.sub(r"[^\n]", " ", match.group(0))


def _c_fragments(text: str) -> Iterator[tuple[int, str]]:
    """``(offset, fragment)`` for each statement fragment.

    Split on ``;``, braces and *every* comma, at any depth: an argument list and
    an initialiser list are not operator operands, and splitting them keeps
    ``f(a - 1, b + 2)`` from reading as one expression.
    """
    start = 0
    for index, char in enumerate(text):
        if char in ";{},":
            yield start, text[start:index]
            start = index + 1
    yield start, text[start:]


def check_c(source: str, path: str = "<memory>") -> list[Crossing]:
    """Every clock crossing in one C translation unit, ordered by line."""
    text = _C_NOISE.sub(_blank, source)
    found: dict[int, Crossing] = {}
    for offset, fragment in _c_fragments(text):
        names: set[str] = set()
        has_operator = False
        for token in _C_TOKEN.finditer(fragment):
            value = token.group(0)
            if value in _C_ARITH:
                has_operator = True
            elif value[:1].isalpha() or value.startswith("_"):
                names.add(value)
        if not has_operator:
            continue
        mcu, host = _classify(names, _C_MCU_EXTRA)
        if not mcu or not host:
            continue
        line = text.count("\n", 0, offset) + 1
        found.setdefault(
            line, Crossing(path, line, " ".join(fragment.split())[:90], mcu, host)
        )
    return [found[line] for line in sorted(found)]


# ---------------------------------------------------------------------------
# trees
# ---------------------------------------------------------------------------


def check_tree(root: Path, repo: Path) -> list[Crossing]:
    """Every crossing under ``root``, with repo-relative paths."""
    out: list[Crossing] = []
    for path in sorted([*root.rglob("*.py"), *root.rglob("*.[ch]")]):
        relative = str(path.relative_to(repo))
        source = path.read_text(encoding="utf-8", errors="replace")
        checker = check_python if path.suffix == ".py" else check_c
        out.extend(checker(source, relative))
    return out
