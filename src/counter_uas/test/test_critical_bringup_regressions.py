from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_run_capture_module():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'run_capture.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('run_capture_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _parse_scalar(raw: str):  # noqa: ANN202
    value = raw.split('#', 1)[0].strip()
    if value.lower() == 'true':
        return True
    if value.lower() == 'false':
        return False
    try:
        if any(ch in value for ch in '.eE'):
            return float(value)
        return int(value)
    except ValueError:
        return value


def _top_level_block(path: Path, name: str) -> str:
    lines = path.read_text(encoding='utf-8').splitlines()
    start = next(i for i, line in enumerate(lines) if line.strip() == f'{name}:')
    out: list[str] = []
    for line in lines[start + 1:]:
        if line and not line.startswith((' ', '#')):
            break
        out.append(line)
    return '\n'.join(out)


def _tracking_params(path: Path) -> dict[str, object]:
    block = _top_level_block(path, 'tracking_node')
    params: dict[str, object] = {}
    for line in block.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith('#') or ':' not in stripped:
            continue
        key, value = stripped.split(':', 1)
        if value.strip():
            params[key.strip()] = _parse_scalar(value)
    return params


def test_default_config_keeps_gazebo_scale_tracking_gates() -> None:
    default_cfg = _REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config.yaml'
    gazebo_cfg = _REPO_ROOT / 'src' / 'counter_uas' / 'config' / 'config_gazebo_counter_uas.yaml'
    required = {
        'candidate_match_gate_m',
        'candidate_predictive_gate',
        'association_gate_m',
        'confirmation_hits',
        'candidate_max_missed_frames',
        'max_track_speed_mps',
        'max_update_jump_m',
    }

    default_params = _tracking_params(default_cfg)
    gazebo_params = _tracking_params(gazebo_cfg)

    missing = sorted(required - set(default_params))
    assert not missing, f'default bringup config is missing km-scale tracking params: {missing}'
    for key in sorted(required):
        assert default_params[key] == gazebo_params[key], key


def test_interceptor_controller_uses_existing_norm_helper_and_one_shot_origin_timer() -> None:
    src = (
        _REPO_ROOT
        / 'src'
        / 'gazebo_target_sim'
        / 'gazebo_target_sim'
        / 'interceptor_controller_node.py'
    ).read_text(encoding='utf-8')

    assert 'self._norm3(' not in src
    assert 'norm3((vx, vy, vz))' in src
    assert 'self._origin_reset_timer = self.create_timer' in src
    assert 'destroy_timer(self._origin_reset_timer)' in src
    assert 'if not self._idle or self._impact_hidden:' in src


def test_run_capture_marks_coreutils_timeout_in_log(tmp_path, monkeypatch) -> None:  # noqa: ANN001
    run_capture = _load_run_capture_module()
    install_setup = tmp_path / 'install' / 'setup.bash'
    install_setup.parent.mkdir()
    install_setup.write_text('', encoding='utf-8')
    logs_dir = tmp_path / 'runs' / 'logs'

    def fake_run(cmd, **kwargs):  # noqa: ANN001, ANN202
        if kwargs.get('stdout') is not None:
            return types.SimpleNamespace(returncode=124)
        return types.SimpleNamespace(returncode=0, stdout='')

    monkeypatch.setattr(run_capture, 'INSTALL_SETUP', install_setup)
    monkeypatch.setattr(run_capture, 'RUNS_DIR', logs_dir)
    monkeypatch.setattr(run_capture, '_git_commit', lambda: None)
    monkeypatch.setattr(run_capture, '_git_dirty', lambda: False)
    monkeypatch.setattr(run_capture.subprocess, 'run', fake_run)

    log_path, _meta_path, _meta, rc = run_capture.run_capture(
        scenario='single',
        timeout_s=1.0,
        notes=None,
        launch_args=None,
    )

    assert rc == 124
    assert '=== TIMEOUT ===' in Path(log_path).read_text(encoding='utf-8')
