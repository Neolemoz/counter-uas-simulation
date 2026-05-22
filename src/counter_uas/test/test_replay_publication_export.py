"""Tests for E2 publication export (PLAT-SA-E2)."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(mod)
    return mod


def test_render_publication_packet_html():
    mod = _load_module("export_presentation_pack", _EVAL / "export_presentation_pack.py")
    sweep_path = _REPO / "fixtures/sa_r0/sweeps/ridge_overlap_sweep/sweep.json"
    manifest = json.loads(sweep_path.read_text(encoding="utf-8"))
    synthesis_path = _REPO / "fixtures/sa_r0/synthesis/cross_sweep_synthesis_v1.json"
    synthesis = json.loads(synthesis_path.read_text(encoding="utf-8"))
    html = mod.render_publication_packet_html(manifest, synthesis=synthesis)
    assert "@media print" in html
    assert "Figure 1" in html or "figure not generated" in html.lower() or "Figures" in html
    assert "[Replay: ridge_overlap_sweep" in html


def test_publication_report_v1_artifact():
    mod = _load_module("export_presentation_pack", _EVAL / "export_presentation_pack.py")
    sweep_path = _REPO / "fixtures/sa_r0/sweeps/delayed_detection_sweep/sweep.json"
    manifest = json.loads(sweep_path.read_text(encoding="utf-8"))
    report = mod.render_publication_report_v1(manifest)
    assert report["artifact_type"] == "replay_publication_report_v1"
    assert report["sweep_id"] == "delayed_detection_sweep"


def test_publication_packet_fixture_exists():
    path = _REPO / "fixtures/sa_r0/sweeps/ridge_overlap_sweep/reports/publication_packet.html"
    assert path.is_file()
    text = path.read_text(encoding="utf-8")
    assert "Publication replay review packet" in text
