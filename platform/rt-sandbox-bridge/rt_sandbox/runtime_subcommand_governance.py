"""Runtime subcommand registry governance lint (PLAT-RT-R3c).

See docs/evaluation/rt_runtime_subcommand_registry_v1.md.
"""

from __future__ import annotations

import ast
from pathlib import Path

from rt_sandbox.audit_vocabulary import EVENT_KIND_ADAPTER, classify_event_kind
from rt_sandbox.governance import (
    RUNTIME_SUBCOMMAND_AUDIT_EXCEPTIONS,
    RUNTIME_SUBCOMMANDS,
    RUNTIME_SUBCOMMANDS_RESERVED,
)

_HANDLER_MODULE = Path(__file__).resolve().parent / "session_runtime_commands.py"
_HANDLER_FUNCTION = "handle_runtime_command"


def discover_runtime_subcommand_handlers(
    source_path: Path | None = None,
) -> frozenset[str]:
    """Return subcommand names implemented via ``if sub == "..."`` in handle_runtime_command."""
    path = source_path or _HANDLER_MODULE
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    handlers: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef) or node.name != _HANDLER_FUNCTION:
            continue
        for child in ast.walk(node):
            if not isinstance(child, ast.If):
                continue
            test = child.test
            if not isinstance(test, ast.Compare):
                continue
            if len(test.ops) != 1 or not isinstance(test.ops[0], ast.Eq):
                continue
            if len(test.comparators) != 1:
                continue
            left = test.left
            if not isinstance(left, ast.Name) or left.id != "sub":
                continue
            comparator = test.comparators[0]
            if isinstance(comparator, ast.Constant) and isinstance(comparator.value, str):
                handlers.add(comparator.value)
    return frozenset(handlers)


def lint_runtime_subcommands(
    *,
    source_path: Path | None = None,
) -> list[str]:
    """Return governance drift issues for runtime subcommand registry vs handlers."""
    issues: list[str] = []
    handlers = discover_runtime_subcommand_handlers(source_path)

    for name in sorted(RUNTIME_SUBCOMMANDS - handlers):
        issues.append(f"declared but unimplemented: {name}")
    for name in sorted(handlers - RUNTIME_SUBCOMMANDS):
        issues.append(f"implemented but not in RUNTIME_SUBCOMMANDS: {name}")

    overlap = RUNTIME_SUBCOMMANDS & RUNTIME_SUBCOMMANDS_RESERVED
    for name in sorted(overlap):
        issues.append(f"reserved subcommand incorrectly allowed: {name}")

    for name in sorted(RUNTIME_SUBCOMMANDS):
        if classify_event_kind(name) != EVENT_KIND_ADAPTER:
            issues.append(
                f"subcommand not classified as adapter in audit vocabulary: {name}"
            )

    for subcommand in RUNTIME_SUBCOMMAND_AUDIT_EXCEPTIONS:
        if subcommand not in RUNTIME_SUBCOMMANDS:
            issues.append(
                f"audit exception for unknown subcommand: {subcommand}"
            )

    return issues
