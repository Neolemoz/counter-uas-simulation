# Sensor realism overlay runbook

**Freeze:** SIM-SR1 (frozen stable, default-off). See [README.md](README.md) § Sensor realism freeze.

Additive, default-off sensor realism for `radar_sim` and `camera_sim`. Topics remain `geometry_msgs/Point` on `/radar/detections`, `/camera/detections`, `/fused_detections`, and `nav_msgs/Odometry` on `/tracks/state`.

## Overlay packs

| File | Effect |
|------|--------|
| [range_dependent_sensing.yaml](range_dependent_sensing.yaml) | Range-dependent PD falloff and measurement σ scaling |
| [sensor_decimation_latency.yaml](sensor_decimation_latency.yaml) | `publish_every_n` decimation + transport delay |

Merged ROS patch: `src/counter_uas/config/config_sensor_realism_overlay.yaml`

## Bringup (default-off)

Lab stack with overlays patched onto an existing config:

```bash
source install/setup.bash
ros2 launch counter_uas bringup.launch.py \
  counter_uas_config:=config_lab_toy.yaml \
  enable_sensor_realism_overlay:=true \
  use_gazebo_gui:=false
```

All-in-one lab config (no separate overlay flag):

```bash
ros2 launch counter_uas bringup.launch.py \
  counter_uas_config:=config_lab_toy_sensor_realism.yaml \
  use_gazebo_gui:=false
```

Km-scale stack with overlay patch on default params:

```bash
ros2 launch counter_uas bringup.launch.py \
  enable_sensor_realism_overlay:=true \
  use_gazebo_gui:=false
```

Optional lifecycle evidence on `/tracks/state`:

```bash
  enable_lifecycle_observer:=true
```

## Verification (offline)

```bash
python3 -m pytest src/counter_uas/test/test_sensor_realism_propagation.py -q
```

The headless harness in `counter_uas.sensor_realism_propagation` checks reduced fused cadence under overlay and non-zero `/tracks/state` publishes without tactical or RT bridge code.
