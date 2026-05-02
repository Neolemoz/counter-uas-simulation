# Cursor agent runbook: Gazebo Sim empty viewport / empty Entity tree

**Thai summary:** ใช้เอกสารนี้เมื่อผู้ใช้รายงานว่า Gazebo เปิดแต่หน้าจอเทาและ **Entity tree ว่าง** — สาเหตุหลักมักไม่ใช่ "build พัง" แต่เป็นการไม่ได้โหลด world ผ่าน `ros2 launch` หรือ GUI cache (`~/.gz/sim`) เสีย

## 1. Classify the symptom

| Case | What user sees | First hypothesis |
|------|----------------|------------------|
| A | No 3D window at all | `use_gazebo_gui:=false` or headless; or no DISPLAY |
| B | Gray viewport + **empty** Entity tree | Gazebo opened **without** world file (app menu, `gz sim` alone, `gz sim -g` only), or broken GUI session |
| C | Entity tree has `ground_plane` / models but viewport still gray | Rendering/Qt/GPU; camera; rare transport |

This repo targets **Case B** most often when users only run `colcon build` or start Gazebo from the desktop.

## 2. Quick start (human): one command

จากรากโปรเจกต์ (หลัง `colcon build`):

```bash
./scripts/run_gazebo_cuas.sh
```

หลายเป้า: `./scripts/run_gazebo_cuas.sh --multi`  
Headless: `./scripts/run_gazebo_cuas.sh use_gazebo_gui:=false`

สคริปต์จะ `source` ROS (jazzy หรือ humble) แล้ว `install/setup.bash` ให้อัตโนมัติ

## 3. Preconditions (agent)

- Source ROS **before** workspace: `source /opt/ros/$ROS_DISTRO/setup.bash` then `source install/setup.bash`.
- After editing Python/launch/config under `gazebo_target_sim`, run:  
  `colcon build --packages-select gazebo_target_sim --symlink-install`

## 4. Verify install artifacts

```bash
ros2 pkg prefix gazebo_target_sim
test -f "$(ros2 pkg prefix gazebo_target_sim)/share/gazebo_target_sim/worlds/target_sphere.sdf" && echo world_ok
test -f "$(ros2 pkg prefix gazebo_target_sim)/share/gazebo_target_sim/gui/cuas_gz_gui.config" && echo gui_ok
gz sdf -k "$(ros2 pkg prefix gazebo_target_sim)/share/gazebo_target_sim/worlds/target_sphere.sdf"
```

## 5. Read launch behavior (codebase)

- [src/gazebo_target_sim/launch/gazebo_target.launch.py](launch/gazebo_target.launch.py) — GUI mode builds `gz sim -r [--gui-config .../cuas_gz_gui.config] <world_sdf>`; headless uses `-s`.
- [src/gazebo_target_sim/launch/gazebo_target_multi.launch.py](launch/gazebo_target_multi.launch.py) — same pattern for `target_sphere_multi.sdf`.

Launch prints **`[gazebo_target_sim] gz sim command:`** — use that line to confirm argv the user should see in terminal when the issue is reported.

## 6. Headless smoke test (agent)

```bash
source /opt/ros/jazzy/setup.bash   # or humble
source install/setup.bash
timeout 15 ./scripts/run_gazebo_cuas.sh use_gazebo_gui:=false 2>&1 | head -40
```

Expect: `[gz-1]: process started`, target_controller log mentioning `set_pose`, no immediate crash.

## 7. User actions (when Case B persists after correct launch)

1. Close any **empty** Gazebo window opened from the app menu.
2. From the same terminal session as ROS sourcing, run **only**:
   - `ros2 launch gazebo_target_sim gazebo_target.launch.py`  
     (or `gazebo_target_multi.launch.py`)
3. Backup and clear stale GUI state:
   - `mv ~/.gz/sim ~/.gz/sim.bak.$(date +%Y%m%d%H%M)`
4. Press **Play** if the sim is paused (orange pause). Note: empty Entity tree usually means **no world**, not only paused.
5. Optional render fallback (user machine only):  
   `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ...` or `QT_QPA_PLATFORM=xcb ros2 launch ...`

## 8. Repo fixes (only if verification fails)

- `gui/*.config` missing from install → [src/gazebo_target_sim/setup.py](setup.py) must install `glob('gui/*.config')`.
- Launch not passing `--gui-config` when intended → check `gz_use_shipped_gui_config` (default true) and `get_package_share_directory('gazebo_target_sim')`.

## 9. Definition of done

- Terminal shows full `gz sim` argv including path to `target_sphere.sdf` (or multi).
- Headless smoke reaches running processes without fatal errors.
- User confirms Entity tree lists world entities (e.g. `ground_plane`, `sphere_target_0`) or provides a new screenshot.

## References

- [gz-sim#2617 — gray 3D scene after loading GUI client configuration](https://github.com/gazebosim/gz-sim/issues/2617)
