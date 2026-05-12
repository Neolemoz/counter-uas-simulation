"""Governance contract for evaluation_row lineage fields."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_evaluation_row():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'evaluation_row.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('evaluation_row', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_evaluation_row_preserves_cohort_and_git_lineage(tmp_path: Path) -> None:
    mod = _load_evaluation_row()
    log = tmp_path / 'run.log'
    log.write_text(
        '\n'.join(
            [
                '[LAYER] detect',
                '[HIT] interceptor_0 layer=engage min_miss=0.42 m',
            ],
        )
        + '\n',
        encoding='utf-8',
    )
    meta = tmp_path / 'run.meta.json'
    meta.write_text(
        json.dumps(
            {
                'cohort': 'cohort_a',
                'git_commit': 'abc123',
                'git_dirty': False,
                'notes': 'arm=A seed=101',
            },
        ),
        encoding='utf-8',
    )

    row = mod.evaluation_row(log, meta_path=meta)

    assert row['run_id'] == 'run'
    assert row['cohort'] == 'cohort_a'
    assert row['git_commit'] == 'abc123'
    assert row['git_dirty'] is False
    assert row['notes_meta'] == 'arm=A seed=101'
    assert row['hit'] is True
    assert row['layer_at_hit'] == 'engage'
