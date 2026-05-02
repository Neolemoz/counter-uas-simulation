# counter-uas-simulation

ROS 2 workspace: Gazebo Sim counter-UAS engagement (`gazebo_target_sim`) plus a separate **sensor → fusion → tracking → visualization** pipeline (`counter_uas` bringup). Engagement logic can consume **ground truth** or **fused / track** topics (see below).

## Packages

| Package | Role |
|--------|------|
| `gazebo_target_sim` | Gz world, target & interceptor controllers, **`interception_logic_node`**, noisy measurement bridge |
| `counter_uas` | Launch + shared YAML configs |
| `radar_sim` | `/drone/position` → `/radar/detections` (range, beam gate, PD, noise) |
| `camera_sim` | `/drone/position` → `/camera/detections` (mount + yaw FOV, range, PD) |
| `fusion` | Radar + camera points → `/fused_detections` |
| `tracking` | `/fused_detections` → `/tracks` (CV Kalman + association) |
| `threat_assessment` | `/tracks` → `/threat_level` (distance bands) |
| `visualization` | RViz markers from `/tracks` + threat state machine |
| `world_sim` | Optional kinematic target (circular path) without Gazebo |

## Build

```bash
cd /path/to/counter-uas-simulation
colcon build --symlink-install
source install/setup.bash
```

## Launches

### Full stack (Gazebo + sensors + fusion + tracking + viz + threat)

Uses shared YAML under `counter_uas/config/` (default `config.yaml`, km-scale, camera aimed toward incoming traffic from \(-X\)).

```bash
source install/setup.bash
ros2 launch counter_uas bringup.launch.py
```

**Launch arguments**

| Argument | Default | Role |
|----------|---------|------|
| `counter_uas_config` | `config.yaml` | Parameter file name inside `share/counter_uas/config/` |
| `intercept_measurement_source` | `tracks` | Engagement feed: `ground_truth`, `fused`, or `tracks` |
| `fused_detections_topic` | `/fused_detections` | Topic when `intercept_measurement_source:=fused` |
| `tracks_topic` | `/tracks` | Topic when `intercept_measurement_source:=tracks` |

Optional: **small-scale** config (12 m radar) for toy worlds:

```bash
ros2 launch counter_uas bringup.launch.py \
  counter_uas_config:=config_lab_toy.yaml
```

### Gazebo + interception only (no radar stack)

```bash
ros2 launch gazebo_target_sim gazebo_target.launch.py
```

## Data paths: ground truth vs fused track

Two parallel paths exist:

1. **Engagement (`interception_logic_node`)** — subscribes to a single `geometry_msgs/Point` stream for target position (default **`/drone/position`** = Gazebo GT).
2. **Perception chain** — `/drone/position` → radar & camera → **`/fused_detections`** → **`/tracks`**.

To drive interception from **tracked** or **fused** measurements, set `intercept_measurement_source` when launching (see bringup arguments) or override at runtime:

```bash
ros2 param set /interception_logic_node intercept_measurement_source tracks
```

`gazebo_target_multi.launch.py` still assigns `/drone_i/position` per target; `intercept_measurement_source` applies only to single-target mode.

## Main topics (single-target)

| Topic | Type | Producer | Notes |
|-------|------|----------|------|
| `/drone/position` | `Point` | `target_controller_node` | Gazebo truth |
| `/radar/detections` | `Point` | `radar_sim_node` | Noisy, range-limited |
| `/camera/detections` | `Point` | `camera_sim_node` | FOV gate |
| `/fused_detections` | `Point` | `fusion_node` | Join or pass-through |
| `/tracks` | `Point` | `tracking_node` | Filtered position (no id in msg) |
| `/threat_level` | `String` | `threat_assessment_node` | HIGH / MEDIUM / LOW |
| `/interception/markers` | `Marker` | `interception_logic_node` | RViz |
| `/danger_zone/dome` | `Marker` | `interception_logic_node` | Dome shells |

## Config files

- [`src/counter_uas/config/config.yaml`](src/counter_uas/config/config.yaml) — **default** `counter_uas_config` (km-scale, `bringup`).
- [`src/counter_uas/config/config_gazebo_counter_uas.yaml`](src/counter_uas/config/config_gazebo_counter_uas.yaml) — same class of scenario, explicit filename.
- [`src/counter_uas/config/config_lab_toy.yaml`](src/counter_uas/config/config_lab_toy.yaml) — ~10 m lab scale.

## Regression tests

```bash
colcon test --packages-select counter_uas --event-handlers console_direct+
```

Parses a small synthetic log with `scripts/analyze_run.py` so metric extraction stays stable.

## Scripts & research

- `scripts/run_capture.py`, `scripts/analyze_run.py` — log capture and metrics.
- `scripts/validate_heatmap_vs_gazebo.py` — heatmap vs Gazebo trials.
- `research/phase*.md` — baseline metrics and experiment notes.
- **Scenario catalog & replay:** see [`docs/scenarios/`](docs/scenarios/) (interface split sim vs production).

## Message types (roadmap)

The perception chain currently uses `geometry_msgs/Point` for compact lab plumbing. For production-style contracts, prefer stamped messages or track messages with **id, stamp, frame_id, covariance** (Phase C+ evolution).
