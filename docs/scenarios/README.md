# Scenario catalog & VV&A notes

This folder holds **scenario definitions** and **verification** guidance so simulation can be compared against a production path without forking algorithm code.

## Catalog (`scenario_catalog.yaml`)

Each entry describes:

- **scale**: `gazebo_km` vs `lab_m` (which `counter_uas_config` to load).
- **engagement**: `intercept_measurement_source` (`ground_truth` | `fused` | `tracks`).
- **assets**: dome radii / radar range references (documentation only; launch files own authoritative numbers until a single scenario loader exists).
- **interfaces**: topic names for detections, fused points, tracks, threat.

Edit [`scenario_catalog.yaml`](scenario_catalog.yaml) when adding a new world or ROC study; keep names aligned with `ros2 launch` arguments in the root [`README.md`](../README.md).

## Recording & replay (ros2 bag)

**Record** a full bringup session (trim topics to what you need):

```bash
source install/setup.bash
ros2 bag record /drone/position /radar/detections /camera/detections \
  /fused_detections /tracks /threat_level /interceptor/selected_id \
  /interception/markers /tf /tf_static
```

**Replay** into the same stack (sensors off; publish recorded truths first to confirm):

```bash
ros2 bag play your_bag.mcap --clock 100
```

Use `--remap` if your capture used prefixed topic names. For engagement-only trials, record `/drone/position` plus `/interception/*` markers.

## Sim vs production interface

| Layer | Simulation (this repo) | Production-oriented stub |
|-------|----------------------|---------------------------|
| Raw detections | `geometry_msgs/Point` topics | Stamped `Point` / `Detection3D` + sensor id |
| Fusion output | `/fused_detections` `Point` | Track reports + covariance + time |
| Tracks | `/tracks` `Point` (filtered centroid) | Track id, class, NEES-gated state |
| Engagement | `interception_logic_node` subscribes to chosen `Point` stream | Adapter node maps production tracks → single `Point` or native track type |

**Contract**: keep topic *names* stable in YAML (`scenario_catalog.yaml` → `counter_uas` configs) and add **adapter nodes** rather than branching inside `interception_logic_node` for each fielded system.

## Hygiene

Heavy artifacts belong under `runs/` (ignored by git). Commit only short log excerpts or derived CSV if needed for regression; prefer the `counter_uas` pytest regression for parser stability.
