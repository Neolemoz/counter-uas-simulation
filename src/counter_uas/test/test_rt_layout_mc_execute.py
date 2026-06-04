from __future__ import annotations

import argparse
import importlib.util
import json
import subprocess
from pathlib import Path
from unittest.mock import patch

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
_GOLDEN_HANDOFF = _REPO_ROOT / "fixtures" / "rt_sandbox" / "rt_layout_mc_handoff_golden_v1.json"
_GOLDEN_RESULT_SUMMARY = (
    _REPO_ROOT / "fixtures" / "rt_sandbox" / "rt_layout_mc_result_summary_golden_v1.json"
)
_GOLDEN_RESULT_LINK = _REPO_ROOT / "fixtures" / "rt_sandbox" / "rt_layout_mc_result_link_golden_v1.json"
_GOLDEN_PLANNING_REF = (
    _REPO_ROOT / "fixtures" / "rt_sandbox" / "rt_layout_mc_planning_result_ref_golden_v1.json"
)


def _load_mod():  # noqa: ANN201
    path = _REPO_ROOT / "scripts" / "evaluation" / "rt_layout_mc_execute.py"
    assert path.is_file(), f"missing {path}"
    spec = importlib.util.spec_from_file_location("rt_layout_mc_execute", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _load_planning_ref_mod():  # noqa: ANN201
    path = _REPO_ROOT / "scripts" / "evaluation" / "rt_layout_mc_planning_ref.py"
    spec = importlib.util.spec_from_file_location("rt_layout_mc_planning_ref", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_prepare_validates_handoff_and_writes_manifest_status_command(tmp_path: Path) -> None:
    mod = _load_mod()

    result = mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        seed_base=7001,
        cohort="rt_layout_mc_test",
        job_id="golden-job",
    )

    job_dir = tmp_path / "golden-job"
    manifest = json.loads((job_dir / "manifest.json").read_text(encoding="utf-8"))
    status = json.loads((job_dir / "status.json").read_text(encoding="utf-8"))
    command = (job_dir / "command.txt").read_text(encoding="utf-8").strip()

    assert result["job_dir"] == str(job_dir)
    assert manifest["schema_version"] == "rt_layout_mc_execution_manifest_v1"
    assert manifest["job_id"] == "golden-job"
    assert manifest["geometry_id"] == "rt_layout:sha256:4b965e63718a311b"
    assert manifest["source_layout_id"] == "rt_layout_mc_golden"
    assert manifest["run_count"] == 10
    assert manifest["seed_base"] == 7001
    assert manifest["scenario"] == "single"
    assert manifest["cohort"] == "rt_layout_mc_test"
    assert manifest["source_handoff_ref"] == str(_GOLDEN_HANDOFF)

    assert status == {
        "schema_version": "rt_layout_mc_job_status_v1",
        "job_id": "golden-job",
        "status": "prepared",
        "progress_current": 0,
        "progress_total": 10,
        "output_paths": None,
        "command_preview": command,
    }
    assert command == mod.render_command(manifest)


@pytest.mark.parametrize(
    ("label", "scenario"),
    [
        ("single-target", "single"),
        ("multi-target", "multi"),
        ("bringup", "bringup"),
    ],
)
def test_scenario_label_mapping(label: str, scenario: str) -> None:
    mod = _load_mod()

    assert mod._scenario_from_label(label) == scenario


def test_render_command_uses_monte_carlo_run_without_execution(tmp_path: Path) -> None:
    mod = _load_mod()
    result = mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        seed_base=11,
        cohort="cohort-a",
        job_id="cmd-job",
    )

    command = result["status"]["command_preview"]

    assert command.startswith("python3 scripts/monte_carlo.py run")
    assert "--n 10" in command
    assert "--seed-base 11" in command
    assert "--geometry-id rt_layout:sha256:4b965e63718a311b" in command
    assert "--scenario single" in command
    assert "--label cmd-job" in command
    assert "--cohort cohort-a" in command
    assert "target_start_x_m:=-1500" in command


def test_prepare_accepts_job_preview_without_layout(tmp_path: Path) -> None:
    mod = _load_mod()
    handoff = json.loads(_GOLDEN_HANDOFF.read_text(encoding="utf-8"))
    job_path = tmp_path / "job_preview.json"
    job_path.write_text(json.dumps(handoff["mc_job_preview"]), encoding="utf-8")

    result = mod.prepare_job(
        input_path=job_path,
        jobs_root=tmp_path / "jobs",
        job_id="preview-only",
    )

    assert result["manifest"]["schema_version"] == "rt_layout_mc_execution_manifest_v1"
    assert result["manifest"]["scenario"] == "single"
    assert result["manifest"]["source_handoff_ref"] == str(job_path)


def test_prepare_path_does_not_execute_monte_carlo(tmp_path: Path) -> None:
    mod = _load_mod()

    with patch("subprocess.run") as run:
        mod.prepare_job(
            input_path=_GOLDEN_HANDOFF,
            jobs_root=tmp_path,
            job_id="no-exec",
        )

    run.assert_not_called()


def test_prepare_rejects_unknown_schema(tmp_path: Path) -> None:
    mod = _load_mod()
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"schema_version": "unexpected"}), encoding="utf-8")

    with pytest.raises(ValueError, match="schema_version must be"):
        mod.prepare_job(input_path=bad, jobs_root=tmp_path / "jobs")


def _write_mc_outputs(
    mc_dir: Path,
    job_id: str,
    *,
    label: str | None = None,
    include_csv: bool = True,
    n_runs: int = 10,
) -> None:
    mc_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "label": label if label is not None else job_id,
        "n_runs": n_runs,
        "n_success": 8,
        "success_rate": 0.8,
        "miss_distance_m": {"mean": 1.2, "p95": 2.5},
        "intercept_time_s": {"mean": 4.0, "p95": 5.1},
    }
    (mc_dir / f"{job_id}.json").write_text(json.dumps(summary), encoding="utf-8")
    if include_csv:
        (mc_dir / f"{job_id}.csv").write_text("run_id,success\n", encoding="utf-8")


def test_execute_dry_run_prints_command_without_status_change(tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="dry-job",
    )
    job_dir = tmp_path / "dry-job"
    before = json.loads((job_dir / "status.json").read_text(encoding="utf-8"))

    with patch("subprocess.run") as run:
        result = mod.execute_job(job_ref=job_dir, dry_run=True)

    run.assert_not_called()
    assert result["dry_run"] is True
    assert result["command"] == before["command_preview"]
    after = json.loads((job_dir / "status.json").read_text(encoding="utf-8"))
    assert after == before


def test_execute_success_transitions_and_output_paths(tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="ok-job",
    )
    job_dir = tmp_path / "ok-job"
    mc_dir = tmp_path / "mc"
    _write_mc_outputs(mc_dir, "ok-job")

    def fake_run(*_args, **_kwargs):  # noqa: ANN001
        return subprocess.CompletedProcess(args=[], returncode=0, stdout="", stderr="")

    result = mod.execute_job(
        job_ref=job_dir,
        repo_root=_REPO_ROOT,
        mc_out_dir=mc_dir,
        run_subprocess=fake_run,
    )

    assert result["return_code"] == 0
    assert result["status"]["status"] == "completed"
    assert result["output_paths"] == {
        "summary_json": str(mc_dir / "ok-job.json"),
        "summary_csv": str(mc_dir / "ok-job.csv"),
    }
    assert result["status"]["progress_current"] == 10
    assert result["status"]["return_code"] == 0
    assert "completed_utc" in result["status"]

    manifest = json.loads((job_dir / "manifest.json").read_text(encoding="utf-8"))
    on_disk = json.loads((job_dir / "status.json").read_text(encoding="utf-8"))
    assert on_disk["status"] == "completed"
    assert on_disk["output_paths"] == result["output_paths"]
    assert on_disk["result_summary_path"] == str(job_dir / "result_summary.json")
    assert on_disk["result_link_path"] == str(job_dir / "result_link.json")
    assert on_disk["geometry_id"] == manifest["geometry_id"]
    assert on_disk["source_layout_id"] == manifest["source_layout_id"]
    assert on_disk["cohort"] == manifest["cohort"]

    result_summary = json.loads((job_dir / "result_summary.json").read_text(encoding="utf-8"))
    assert result_summary["schema_version"] == "rt_layout_mc_result_summary_v1"
    assert result_summary["job_id"] == "ok-job"
    assert result_summary["geometry_id"] == "rt_layout:sha256:4b965e63718a311b"
    assert result_summary["source_layout_id"] == "rt_layout_mc_golden"
    assert result_summary["run_count"] == 10
    assert result_summary["mc_metrics"]["miss_distance_m_mean"] == 1.2
    assert result_summary["mc_metrics"]["n_runs"] == 10

    result_link = json.loads((job_dir / "result_link.json").read_text(encoding="utf-8"))
    assert result_link["schema_version"] == "rt_layout_mc_result_link_v1"
    assert result_link["manifest_path"] == str(job_dir / "manifest.json")
    assert result_link["result_summary_path"] == str(job_dir / "result_summary.json")
    assert result_link["output_paths"] == result["output_paths"]


def test_execute_subprocess_failure_marks_failed(tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="fail-job",
    )
    job_dir = tmp_path / "fail-job"

    def fake_run(*_args, **_kwargs):  # noqa: ANN001
        return subprocess.CompletedProcess(
            args=[],
            returncode=2,
            stdout="",
            stderr="sim launch failed\n",
        )

    result = mod.execute_job(
        job_ref=job_dir,
        repo_root=_REPO_ROOT,
        mc_out_dir=tmp_path / "mc",
        run_subprocess=fake_run,
    )

    assert result["return_code"] == 2
    assert result["status"]["status"] == "failed"
    assert result["status"]["return_code"] == 2
    assert "sim launch failed" in result["status"]["stderr_summary"]
    assert "exited with code 2" in result["status"]["failure_reason"]
    assert result["status"]["output_paths"] is None
    assert not (job_dir / "result_summary.json").exists()
    assert not (job_dir / "result_link.json").exists()


def test_execute_validation_failure_when_outputs_missing(tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="val-job",
    )
    job_dir = tmp_path / "val-job"

    def fake_run(*_args, **_kwargs):  # noqa: ANN001
        return subprocess.CompletedProcess(args=[], returncode=0, stdout="", stderr="")

    result = mod.execute_job(
        job_ref=job_dir,
        repo_root=_REPO_ROOT,
        mc_out_dir=tmp_path / "empty_mc",
        run_subprocess=fake_run,
    )

    assert result["status"]["status"] == "failed"
    assert "post-run validation failed" in result["status"]["failure_reason"]
    assert result["validation_issues"]
    assert not (job_dir / "result_summary.json").exists()


def test_execute_validation_failure_when_label_mismatch(tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="label-job",
    )
    job_dir = tmp_path / "label-job"
    mc_dir = tmp_path / "mc"
    _write_mc_outputs(mc_dir, "label-job", label="wrong-label")

    def fake_run(*_args, **_kwargs):  # noqa: ANN001
        return subprocess.CompletedProcess(args=[], returncode=0, stdout="", stderr="")

    result = mod.execute_job(
        job_ref=job_dir,
        repo_root=_REPO_ROOT,
        mc_out_dir=mc_dir,
        run_subprocess=fake_run,
    )

    assert result["status"]["status"] == "failed"
    assert "label" in result["status"]["failure_reason"]


def test_validate_mc_outputs_accepts_matching_label(tmp_path: Path) -> None:
    mod = _load_mod()
    mc_dir = tmp_path / "mc"
    _write_mc_outputs(mc_dir, "job-a")
    assert mod.validate_mc_outputs("job-a", mc_dir) == []


def test_execute_rejects_completed_status(tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="done-job",
    )
    job_dir = tmp_path / "done-job"
    status_path = job_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["status"] = "completed"
    status_path.write_text(json.dumps(status), encoding="utf-8")

    with pytest.raises(ValueError, match="not executable"):
        mod.execute_job(job_ref=job_dir, repo_root=_REPO_ROOT, mc_out_dir=tmp_path / "mc")


def test_cmd_execute_dry_run_cli(capsys, tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="cli-dry",
    )
    job_dir = tmp_path / "cli-dry"
    args = argparse.Namespace(job_ref=job_dir, dry_run=True)

    rc = mod.cmd_execute(args)
    captured = capsys.readouterr()
    assert rc == 0
    assert "python3 scripts/monte_carlo.py run" in captured.out


def test_failed_job_can_retry_execute(tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="retry-job",
    )
    job_dir = tmp_path / "retry-job"
    mc_dir = tmp_path / "mc"

    def fail_run(*_args, **_kwargs):  # noqa: ANN001
        return subprocess.CompletedProcess(args=[], returncode=1, stdout="", stderr="err")

    mod.execute_job(
        job_ref=job_dir,
        repo_root=_REPO_ROOT,
        mc_out_dir=mc_dir,
        run_subprocess=fail_run,
    )
    _write_mc_outputs(mc_dir, "retry-job")

    def ok_run(*_args, **_kwargs):  # noqa: ANN001
        return subprocess.CompletedProcess(args=[], returncode=0, stdout="", stderr="")

    result = mod.execute_job(
        job_ref=job_dir,
        repo_root=_REPO_ROOT,
        mc_out_dir=mc_dir,
        run_subprocess=ok_run,
    )
    assert result["status"]["status"] == "completed"
    assert (job_dir / "result_summary.json").is_file()
    assert (job_dir / "result_link.json").is_file()


def test_build_result_summary_and_link_identifier_consistency(tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="handoff-job",
    )
    job_dir = tmp_path / "handoff-job"
    manifest = json.loads((job_dir / "manifest.json").read_text(encoding="utf-8"))
    mc_dir = tmp_path / "mc"
    _write_mc_outputs(mc_dir, "handoff-job")

    issues, summary = mod.validate_mc_handoff(manifest=manifest, mc_out_dir=mc_dir)
    assert issues == []
    assert summary is not None

    output_paths = mod._output_paths_for_job("handoff-job", mc_dir)
    completed_utc = "2026-06-02T12:00:00Z"
    result_summary = mod.build_result_summary(
        manifest=manifest,
        output_paths=output_paths,
        mc_summary=summary,
        completed_utc=completed_utc,
    )
    assert result_summary["job_id"] == manifest["job_id"]
    assert result_summary["geometry_id"] == manifest["geometry_id"]
    assert result_summary["cohort"] == manifest["cohort"]

    paths = mod.write_result_handoff_artifacts(
        job_dir=job_dir,
        manifest=manifest,
        output_paths=output_paths,
        mc_summary=summary,
        completed_utc=completed_utc,
    )
    result_link = json.loads((job_dir / "result_link.json").read_text(encoding="utf-8"))
    assert paths["result_summary_path"] == str(job_dir / "result_summary.json")
    assert result_link["source_layout_id"] == manifest["source_layout_id"]
    assert result_link["status_path"] == str(job_dir / "status.json")


def test_validate_mc_handoff_rejects_run_count_mismatch(tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="count-job",
    )
    manifest = json.loads((tmp_path / "count-job" / "manifest.json").read_text(encoding="utf-8"))
    mc_dir = tmp_path / "mc"
    _write_mc_outputs(mc_dir, "count-job", n_runs=3)

    issues, summary = mod.validate_mc_handoff(manifest=manifest, mc_out_dir=mc_dir)
    assert summary is not None
    assert any("run_count" in item for item in issues)


def test_execute_does_not_write_handoff_artifacts_on_subprocess_failure(tmp_path: Path) -> None:
    mod = _load_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="no-handoff",
    )
    job_dir = tmp_path / "no-handoff"
    mc_dir = tmp_path / "mc"
    _write_mc_outputs(mc_dir, "no-handoff")

    def fake_run(*_args, **_kwargs):  # noqa: ANN001
        return subprocess.CompletedProcess(args=[], returncode=3, stdout="", stderr="boom")

    mod.execute_job(
        job_ref=job_dir,
        repo_root=_REPO_ROOT,
        mc_out_dir=mc_dir,
        run_subprocess=fake_run,
    )
    assert not (job_dir / "result_summary.json").exists()
    assert not (job_dir / "result_link.json").exists()


def test_golden_result_summary_and_link_identifier_consistency() -> None:
    planning_ref = _load_planning_ref_mod()
    summary = json.loads(_GOLDEN_RESULT_SUMMARY.read_text(encoding="utf-8"))
    link = json.loads(_GOLDEN_RESULT_LINK.read_text(encoding="utf-8"))
    manifest = {
        "job_id": link["job_id"],
        "geometry_id": link["geometry_id"],
        "source_layout_id": link["source_layout_id"],
        "cohort": link["cohort"],
        "run_count": summary["run_count"],
    }
    issues = planning_ref.audit_layout_identifier_propagation(
        manifest=manifest,
        result_summary=summary,
        result_link=link,
    )
    assert issues == []


def test_planning_result_ref_mapping_from_golden_handoff() -> None:
    planning_ref = _load_planning_ref_mod()
    golden = json.loads(_GOLDEN_PLANNING_REF.read_text(encoding="utf-8"))
    summary = json.loads(_GOLDEN_RESULT_SUMMARY.read_text(encoding="utf-8"))
    link = json.loads(_GOLDEN_RESULT_LINK.read_text(encoding="utf-8"))

    payload = planning_ref.build_planning_mc_result_ref_payload(
        result_link=link,
        result_summary=summary,
        linked_package_id=golden["result_ref"]["linked_package_id"],
        linked_planning_geometry_id=golden["result_ref"]["linked_planning_geometry_id"],
        imported_utc=golden["result_ref"]["imported_utc"],
    )

    assert payload == golden["result_ref"]
    assert payload["mc_run_label"] == link["job_id"]
    assert payload["mc_result_id"] == planning_ref.layout_mc_result_id(link["job_id"])


def test_planning_ref_mapping_does_not_invoke_subprocess_or_bridge() -> None:
    planning_ref = _load_planning_ref_mod()
    summary = json.loads(_GOLDEN_RESULT_SUMMARY.read_text(encoding="utf-8"))
    link = json.loads(_GOLDEN_RESULT_LINK.read_text(encoding="utf-8"))
    golden = json.loads(_GOLDEN_PLANNING_REF.read_text(encoding="utf-8"))

    source = (_REPO_ROOT / "scripts" / "evaluation" / "rt_layout_mc_planning_ref.py").read_text(
        encoding="utf-8"
    )
    assert "subprocess" not in source
    assert "rt_bridge" not in source
    assert "rosbridge" not in source

    with patch("subprocess.run") as run:
        planning_ref.build_planning_mc_result_ref_payload(
            result_link=link,
            result_summary=summary,
            linked_package_id=golden["result_ref"]["linked_package_id"],
            linked_planning_geometry_id=golden["result_ref"]["linked_planning_geometry_id"],
        )
    run.assert_not_called()


def test_audit_layout_identifiers_detects_summary_mismatch() -> None:
    planning_ref = _load_planning_ref_mod()
    summary = json.loads(_GOLDEN_RESULT_SUMMARY.read_text(encoding="utf-8"))
    link = json.loads(_GOLDEN_RESULT_LINK.read_text(encoding="utf-8"))
    manifest = {
        "job_id": link["job_id"],
        "geometry_id": link["geometry_id"],
        "source_layout_id": link["source_layout_id"],
        "cohort": link["cohort"],
        "run_count": summary["run_count"],
    }
    bad_summary = {**summary, "geometry_id": "rt_layout:sha256:deadbeef"}
    issues = planning_ref.audit_layout_identifier_propagation(
        manifest=manifest,
        result_summary=bad_summary,
        result_link=link,
    )
    assert any("geometry_id" in item for item in issues)


def test_execute_success_identifier_propagation_matches_manifest(tmp_path: Path) -> None:
    mod = _load_mod()
    planning_ref = _load_planning_ref_mod()
    mod.prepare_job(
        input_path=_GOLDEN_HANDOFF,
        jobs_root=tmp_path,
        job_id="prop-job",
        cohort="cohort-prop",
    )
    job_dir = tmp_path / "prop-job"
    manifest = json.loads((job_dir / "manifest.json").read_text(encoding="utf-8"))
    mc_dir = tmp_path / "mc"
    _write_mc_outputs(mc_dir, "prop-job")

    def fake_run(*_args, **_kwargs):  # noqa: ANN001
        return subprocess.CompletedProcess(args=[], returncode=0, stdout="", stderr="")

    mod.execute_job(
        job_ref=job_dir,
        repo_root=_REPO_ROOT,
        mc_out_dir=mc_dir,
        run_subprocess=fake_run,
    )
    status = json.loads((job_dir / "status.json").read_text(encoding="utf-8"))
    summary = json.loads((job_dir / "result_summary.json").read_text(encoding="utf-8"))
    link = json.loads((job_dir / "result_link.json").read_text(encoding="utf-8"))
    assert planning_ref.audit_layout_identifier_propagation(
        manifest=manifest,
        result_summary=summary,
        result_link=link,
        status=status,
    ) == []
