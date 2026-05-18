# UI / Visualization Layer

## 1. Layer Purpose

The UI layer defines observational interfaces for operator visibility, diagnostics, tactical observability, replay visibility, heatmaps, and system state presentation.

The UI must remain a view over source contracts, logs, topics, and artifacts. UI state must not become authoritative for guidance, tactical decisions, hit truth, safety policy, or evaluation outcomes.

## 2. Current Visualization State

- `viz_node.py` subscribes to `/tracks` as `geometry_msgs/Point` and publishes `/track_markers`.
- `viz_node.py` creates display-only nearest-neighbor track IDs because `/tracks` has no real ID, stamp, velocity, covariance, or frame identity.
- `interception_logic_node.py` publishes dome, intercept prediction, intercept math, trail, hit, and heatmap markers.
- Important marker topics include `/danger_zone/dome`, `/danger_zone/layer`, `/interception/markers`, `/interception/hit_markers`, and optional `/interception/heatmap_prob`.
- Heatmap CSV/SVG/HTML artifacts are visualization and evaluation artifacts, not real-world probability claims.
- `run_capture.py`, `analyze_run.py`, and evaluation scripts own replayable logs and metadata.
- Existing `web/` files are thin demo surfaces, not a stable frontend framework contract.

## 3. UI Ownership Boundaries

- Track markers: `src/visualization/visualization/viz_node.py`.
- Tactical/guidance markers: marker publishers in `interception_logic_node.py`.
- Impact events: `src/gazebo_target_sim_interfaces/msg/ImpactEvent.msg`.
- Heatmap visualization: live marker/export code plus offline heatmap plotting and validation scripts.
- Replay visibility: run logs, `.meta.json`, CSV/JSON/SVG/HTML artifacts, and evaluation reports.
- Frontend contracts: future UIs must consume stable read-only topics/artifacts unless a separate authority contract is designed.
- Evaluation truth: parsers and artifacts, not RViz or browser state.

## 4. Planned UI Workstreams

- Visualization contract inventory:
  - Expected files/modules: `ARCHITECTURE_MAP.md`, `viz_node.py`, `interception_logic_node.py`, RViz config, heatmap scripts.
  - Risks: marker topics drift, UI treated as truth, undocumented topic types.
  - Validation: topic grep, docs review, optional marker contract tests.
  - Non-goals: marker behavior changes.
- RViz observability cleanup:
  - Expected files/modules: `viz_node.py`, RViz config, launch args.
  - Risks: display IDs mistaken for tracker IDs, stale RViz config.
  - Validation: visual smoke after docs/tests if behavior changes.
  - Non-goals: guidance/tactical changes.
- Tactical observability display:
  - Expected files/modules: selected/assigned topics, marker streams, logs.
  - Risks: coupling tactical decisions to marker state.
  - Validation: parser tests if logs change; marker checks if topics change.
  - Non-goals: operator approval UI.
- Replay-safe visualization:
  - Expected files/modules: run capture, analyzer, heatmap CSV/SVG/HTML scripts, rosbag helper.
  - Risks: replay views diverge from logged evidence.
  - Validation: artifact schema tests and heatmap agreement tests.
  - Non-goals: full ROS bag replay requirement.
- Heatmap visualization consistency:
  - Expected files/modules: live heatmap marker/export, offline render/plot/validation scripts.
  - Risks: `P(hit|noise model)` mislabeled as real `P_kill`, schema drift.
  - Validation: CSV column checks, `test_heatmap_agreement.py`.
  - Non-goals: recalibrating the model.
- UI event compatibility:
  - Expected files/modules: `ImpactEvent.msg`, marker publishers, parser/report scripts.
  - Risks: event schema expansion without parser/docs migration.
  - Validation: message and parser fixtures if schema changes.
  - Non-goals: full event bus now.
- Future frontend preparation:
  - Expected files/modules: `web/index.html`, `web/cesium_rosbridge.html`, topic docs.
  - Risks: hardcoded legacy `/tracks` assumptions or write-capable UI without authority model.
  - Validation: read-only frontend contract review.
  - Non-goals: introducing React/Vue/desktop framework.

## 5. Explicit Non-Goals

- No UI state as source of truth.
- No frontend framework introduction.
- No operator approval, deny, hold-fire, or override implementation.
- No guidance, tactical, safety, or launch-default changes.
- No parser/report replacement with visual state.
- No real-world C2 dashboard claims.

## 6. Visualization Philosophy

Visualization should make simulation state easier to inspect without changing that state. Live markers, browser views, screenshots, and heatmaps are useful only when their source contracts are clear.

Replay-safe visualization must point back to logs, sidecars, seeds, launch args, and parser outputs. If a visual display conflicts with a parser-visible artifact, the artifact wins until the contract is corrected.

## 7. UI Contracts

- `/track_markers` is display-only and based on Point inputs.
- `/danger_zone/dome`, `/danger_zone/layer`, `/interception/markers`, `/interception/hit_markers`, and `/interception/heatmap_prob` are observational marker streams.
- `ImpactEvent` is a minimal soft-hit event for visual/effect consumers.
- Heatmap CSV/SVG/HTML files must remain compatible with validation and plotting scripts.
- Logs and `.meta.json` remain replay/evaluation truth.
- Future frontend surfaces must default to read-only.

## 8. AI-Agent Guidance

- Do not couple UI state back into guidance, tactical, safety, or evaluation logic.
- Treat marker topics and namespaces as compatibility contracts when downstream tools use them.
- Preserve parser/report compatibility when adding event or log fields.
- Avoid treating `/tracks` display IDs as real tracker IDs.
- Read `ARCHITECTURE_MAP.md` and relevant layer docs before UI work.
- Run parser, heatmap, and marker-related tests when changing event or artifact schemas.

## 9. UI Hotspots

- `interception_logic_node.py` owns many marker streams in addition to policy and guidance.
- `/tracks` Point lacks stable identity, stamp, velocity, and covariance.
- RViz marker state can be mistaken for truth.
- Heatmap displays can be overinterpreted as real probability.
- Web demo assumptions are not a stable frontend API.
- Parser-visible logs and visual events may drift apart.

## 10. Unresolved UI Ambiguities

- No canonical frontend contract exists.
- Marker schema/versioning is informal.
- Replay visualization does not yet have a single manifest format.
- Operator-facing state is observational, but future authority boundaries are not implemented.
- Track identity for UI remains weak on Point-only paths.

## 11. Recommended First UI Implementation Candidate

Start with a visualization contract inventory: list marker topics, message types, source files, artifact paths, parser dependencies, and read-only assumptions before changing UI behavior.
