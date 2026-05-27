#!/usr/bin/env python3
"""Compile rt_experiment_spec_v1 to rt_experiment_batch_v1 (PLAT-RT-F5 P0).

Maintainer-only; explanatory experiment planning — not operational authority.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from copy import deepcopy
from itertools import product
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from rt_sandbox.isolation import assert_template_ref_blocked  # noqa: E402
from rt_sandbox.template_catalog import get_template  # noqa: E402

SPEC_GOVERNANCE_BANNER = (
    "RT EXPERIMENT — local maintainer planning only; not operational authority"
)

ALLOWED_MATRIX_AXIS_IDS = frozenset(
    {
        "template_id",
        "tactical_mode_hint",
        "dwell_s",
        "terrain_profile_ref",
        "f4_layer_preset",
    }
)

CLASS_STRATEGY = {
    "terrain_comparison": "explicit_list",
    "sensor_range_comparison": "explicit_list",
    "tactical_mode_comparison": "explicit_list",
    "repeatability_sweep": "repeat_expand",
    "parameter_matrix": "cartesian",
}


def _load_spec(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != "rt_experiment_spec_v1":
        raise ValueError("spec must be rt_experiment_spec_v1")
    return data


def _sort_keys_deep(value: Any) -> Any:
    if isinstance(value, list):
        return [_sort_keys_deep(v) for v in value]
    if isinstance(value, dict):
        return {k: _sort_keys_deep(value[k]) for k in sorted(value)}
    return value


def compute_spec_fingerprint(spec: dict[str, Any]) -> str:
    body = deepcopy(spec)
    body.pop("experiment_id", None)
    canonical = json.dumps(
        _sort_keys_deep(body),
        separators=(",", ":"),
        sort_keys=True,
        ensure_ascii=False,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]


def _assert_builtin_template(template_id: str) -> None:
    assert_template_ref_blocked(template_id)
    if get_template(template_id) is None:
        raise ValueError(f"unknown template_id: {template_id}")


def _assert_allowed_axis(axis_id: str) -> None:
    if axis_id not in ALLOWED_MATRIX_AXIS_IDS:
        raise ValueError(f"forbidden matrix axis: {axis_id}")


def assert_compileable_spec(spec: dict[str, Any]) -> None:
    cls = spec.get("experiment_class")
    strategy = spec.get("compile_strategy")
    if CLASS_STRATEGY.get(cls) != strategy:
        raise ValueError(f"{cls} requires {CLASS_STRATEGY.get(cls)} compile_strategy")

    if strategy == "cartesian":
        axes = spec.get("matrix_axes") or []
        if not axes:
            raise ValueError("cartesian requires matrix_axes")
        for axis in axes:
            _assert_allowed_axis(str(axis["axis_id"]))
            for val in axis.get("values") or []:
                s = str(val)
                if axis["axis_id"] == "template_id":
                    _assert_builtin_template(s)
                else:
                    assert_template_ref_blocked(s)

    if strategy == "repeat_expand":
        rc = spec.get("repeat_config") or {}
        if rc.get("jitter_s", 0) != 0:
            raise ValueError("repeat_config.jitter_s must be 0")
        if int(rc.get("count", 0)) < 2:
            raise ValueError("repeat_config.count must be >= 2")
        _assert_builtin_template(str(rc["base_entry"]["template_id"]))

    if strategy == "explicit_list":
        entries = spec.get("spec_entries") or []
        if not entries:
            raise ValueError("explicit_list requires spec_entries")
        for entry in entries:
            _assert_builtin_template(str(entry["template_id"]))


def _slugify_run_id(parts: list[str]) -> str:
    raw = "m-" + "-".join(parts)
    raw = re.sub(r"[^a-z0-9]+", "-", raw.lower()).strip("-")
    if len(raw) <= 64:
        return raw
    digest = hashlib.sha256(raw.encode("utf-8")).hexdigest()[:8]
    return f"{raw[:55]}-{digest}"


def _axis_signature(coords: dict[str, str]) -> str:
    return ";".join(f"{k}={coords[k]}" for k in sorted(coords))


def _build_run(
    spec: dict[str, Any],
    fingerprint: str,
    *,
    run_id: str,
    label: str,
    template_id: str | None = None,
    dwell_s: float | None = None,
    tactical_mode_hint: str | None = None,
    matrix_coords: dict[str, str] | None = None,
    repeat_index: int | None = None,
    repeat_group_id: str | None = None,
    terrain_profile_ref: str | None = None,
    f4_layer_preset: str | None = None,
) -> dict[str, Any]:
    row: dict[str, Any] = {
        "run_id": run_id,
        "label": label,
        "experiment_class": spec["experiment_class"],
        "spec_fingerprint": fingerprint,
    }
    if template_id:
        row["template_id"] = template_id
    if dwell_s is not None:
        row["dwell_s"] = dwell_s
    if tactical_mode_hint:
        row["tactical_mode_hint"] = tactical_mode_hint
    if matrix_coords is not None:
        row["matrix_coords"] = matrix_coords
    if repeat_index is not None:
        row["repeat_index"] = repeat_index
    if repeat_group_id:
        row["repeat_group_id"] = repeat_group_id
    if terrain_profile_ref:
        row["terrain_profile_ref"] = terrain_profile_ref
    if f4_layer_preset:
        row["f4_layer_preset"] = f4_layer_preset
    return row


def compile_experiment_spec(spec: dict[str, Any]) -> dict[str, Any]:
    assert_compileable_spec(spec)
    fingerprint = compute_spec_fingerprint(spec)
    default_dwell = float(spec.get("default_dwell_s") or 2.0)
    strategy = spec["compile_strategy"]
    runs: list[dict[str, Any]] = []

    if strategy == "explicit_list":
        for entry in spec.get("spec_entries") or []:
            runs.append(
                _build_run(
                    spec,
                    fingerprint,
                    run_id=str(entry["entry_id"]),
                    label=str(entry["label"]),
                    template_id=str(entry["template_id"]),
                    dwell_s=float(entry.get("dwell_s") or default_dwell),
                    tactical_mode_hint=entry.get("tactical_mode_hint"),
                    terrain_profile_ref=entry.get("terrain_profile_ref"),
                    f4_layer_preset=entry.get("f4_layer_preset"),
                )
            )

    elif strategy == "cartesian":
        axes = spec.get("matrix_axes") or []
        axis_names = [str(a["axis_id"]) for a in axes]
        value_lists = [[str(v) for v in a["values"]] for a in axes]
        for combo in product(*value_lists):
            coords = dict(zip(axis_names, combo, strict=True))
            template_id = coords.get("template_id")
            if not template_id:
                raise ValueError("cartesian product missing template_id")
            _assert_builtin_template(template_id)
            dwell = float(coords["dwell_s"]) if "dwell_s" in coords else default_dwell
            runs.append(
                _build_run(
                    spec,
                    fingerprint,
                    run_id=_slugify_run_id(list(coords.values())),
                    label=_axis_signature(coords),
                    template_id=template_id,
                    dwell_s=dwell,
                    tactical_mode_hint=coords.get("tactical_mode_hint"),
                    matrix_coords=coords,
                    terrain_profile_ref=coords.get("terrain_profile_ref"),
                    f4_layer_preset=coords.get("f4_layer_preset"),
                )
            )

    elif strategy == "repeat_expand":
        rc = spec["repeat_config"]
        base = rc["base_entry"]
        base_id = str(base["template_id"]).replace("_v1", "")[:20] or "repeat"
        for index in range(int(rc["count"])):
            runs.append(
                _build_run(
                    spec,
                    fingerprint,
                    run_id=f"{base_id}-r{index}",
                    label=f"{base['label']} #{index}",
                    template_id=str(base["template_id"]),
                    dwell_s=float(base.get("dwell_s") or default_dwell),
                    repeat_index=index,
                    repeat_group_id=str(rc["repeat_group_id"]),
                )
            )

    return {
        "schema": "rt_experiment_batch_v1",
        "experiment_id": spec["experiment_id"],
        "default_dwell_s": default_dwell,
        "runs": runs,
    }


def batch_to_yaml(batch: dict[str, Any]) -> str:
    if yaml is None:
        raise RuntimeError("PyYAML required for YAML output")
    return yaml.safe_dump(batch, sort_keys=False, default_flow_style=False)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compile RT experiment spec to batch spec")
    parser.add_argument("--spec", required=True, type=Path, help="rt_experiment_spec_v1 JSON")
    parser.add_argument("--out", type=Path, help="write batch YAML/JSON")
    parser.add_argument(
        "--format",
        choices=("yaml", "json"),
        default="yaml",
        help="output format when --out set or stdout",
    )
    args = parser.parse_args(argv)

    spec = _load_spec(args.spec.resolve())
    if spec.get("governance_banner") != SPEC_GOVERNANCE_BANNER:
        print("warning: governance_banner mismatch", file=sys.stderr)
    batch = compile_experiment_spec(spec)

    if args.format == "json":
        text = json.dumps(batch, indent=2, sort_keys=True) + "\n"
    else:
        text = batch_to_yaml(batch)

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
