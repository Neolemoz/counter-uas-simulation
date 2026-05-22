"""Layer B realism contracts that should not drift silently."""

from __future__ import annotations

from pathlib import Path


REPO = Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    return (REPO / path).read_text(encoding='utf-8')


def test_full_stack_bringup_prefers_tracks_state() -> None:
    text = _read('src/counter_uas/launch/bringup.launch.py')

    assert "'intercept_measurement_source'," in text
    assert "default_value='tracks_state'" in text
    assert "'tracks_state_topic': LaunchConfiguration('tracks_state_topic')" in text
    assert "default_value='/tracks/state'" in text
    assert "'target_start_x_m': LaunchConfiguration('target_start_x_m')" in text
    assert "'target_start_y_m': LaunchConfiguration('target_start_y_m')" in text
    assert "'target_start_z_m': LaunchConfiguration('target_start_z_m')" in text


def test_standalone_single_target_keeps_ground_truth_default_with_state_override() -> None:
    text = _read('src/gazebo_target_sim/launch/gazebo_target.launch.py')

    assert "'intercept_measurement_source'," in text
    assert "default_value='ground_truth'" in text
    assert "default_value='/drone/position'" in text
    assert "'tracks_state_topic'," in text
    assert "Odometry topic when intercept_measurement_source:=tracks_state" in text


def test_multi_target_remains_point_based_ground_truth_by_default() -> None:
    text = _read('src/gazebo_target_sim/launch/gazebo_target_multi.launch.py')

    assert "'intercept_measurement_source'," in text
    assert "default_value='ground_truth'" in text
    assert "ignored when multi_target_enabled" in text
    assert "default_value='/tracks'" in text
    assert "'tracks_state_topic'," not in text


def test_offline_heatmap_rollout_defaults_match_live_launch_limits() -> None:
    heatmap = _read('scripts/render_intercept_heatmap_prob_offline.py')
    launch = _read('src/gazebo_target_sim/launch/gazebo_target.launch.py')

    assert "'interceptor_max_turn_rate_rad_s'" in launch
    assert "default_value='2.5'" in launch
    assert "'interceptor_max_accel_m_s2'" in launch
    assert "default_value='30.0'" in launch
    assert "'--rollout-max-turn-rate-rad-s'" in heatmap
    assert 'default=2.5' in heatmap
    assert "'--rollout-max-accel-m-s2'" in heatmap
    assert 'default=30.0' in heatmap


def test_live_heatmap_rollout_uses_node_plant_limits() -> None:
    node = _read('src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py')
    launch = _read('src/gazebo_target_sim/launch/gazebo_target.launch.py')

    assert "'max_acceleration_m_s2': interceptor_max_accel" in launch
    assert "'max_turn_rate_rad_s': interceptor_turn_rate" in launch
    assert 'self._max_accel = max(float(self.get_parameter(\'max_acceleration_m_s2\').value), 0.1)' in node
    assert 'self._max_turn_rate = max(float(self.get_parameter(\'max_turn_rate_rad_s\').value), 0.0)' in node
    assert 'rollout_max_turn_rate_rad_s=self._max_turn_rate' in node
    assert 'rollout_max_accel_m_s2=self._max_accel' in node


def test_autopilot_tau_and_cmd_delay_are_zero_default_contracts() -> None:
    controller = _read('src/gazebo_target_sim/gazebo_target_sim/interceptor_controller_node.py')
    launch = _read('src/gazebo_target_sim/launch/gazebo_target.launch.py')
    multi_launch = _read('src/gazebo_target_sim/launch/gazebo_target_multi.launch.py')

    assert "self.declare_parameter('autopilot.tau_s', 0.0)" in controller
    assert "self.declare_parameter('autopilot.cmd_delay_s', 0.0)" in controller
    assert "'cmd_timeout_s': 0.75" in launch
    assert "'cmd_timeout_s': 0.75" in multi_launch
    assert "'autopilot.tau_s'" not in launch
    assert "'autopilot.cmd_delay_s'" not in launch
    assert "'autopilot.tau_s'" not in multi_launch
    assert "'autopilot.cmd_delay_s'" not in multi_launch


def test_heatmap_validation_replay_preserves_launch_boundary() -> None:
    validate = _read('scripts/validate_heatmap_vs_gazebo.py')
    run_capture = _read('scripts/run_capture.py')

    assert 'target_start_x_m:={x:.6f}' in validate
    assert 'target_start_y_m:={y:.6f}' in validate
    assert 'target_start_z_m:={z:.6f}' in validate
    assert 'use_gazebo_gui:=false' in validate
    assert 'run_capture(' in validate
    assert 'launch_args=launch_args' in validate
    assert 'if launch_args:' in run_capture
    assert 'ros_cmd = f"{ros_cmd} {launch_args}"' in run_capture
    assert 'cmd=cmd' in run_capture


def test_fusion_point_stream_remains_explicitly_unstamped_and_single_output() -> None:
    fusion = _read('src/fusion/fusion/fusion_node.py')

    assert 'from geometry_msgs.msg import Point' in fusion
    assert 'from nav_msgs.msg import Odometry' not in fusion
    assert 'create_publisher(Point, \'/fused_detections\', 10)' in fusion
    assert "create_subscription(Point, '/radar/detections', self._on_radar, 10)" in fusion
    assert "create_subscription(Point, '/camera/detections', self._on_camera, 10)" in fusion
    assert 'Single-output sensor fusion: emits exactly one ``geometry_msgs/Point`` per fusion event.' in fusion
    assert 'Single-sensor path: only publish when pair-up is disabled (legacy mode).' in fusion


def test_tracking_state_stream_preserves_timestamp_covariance_and_track_id_contract() -> None:
    tracking = _read('src/tracking/tracking/tracking_node.py')

    assert 'self.create_subscription(Point, \'/fused_detections\', self._on_detection, 10)' in tracking
    assert 'self._pub = self.create_publisher(Point, \'/tracks\', 10)' in tracking
    assert 'self._pub_state = self.create_publisher(Odometry, self._tracks_state_topic, 10)' in tracking
    assert 'msg.header.stamp = self.get_clock().now().to_msg()' in tracking
    assert 'msg.header.frame_id = self._tracks_state_frame_id' in tracking
    assert "msg.child_frame_id = f'track_{tr.track_id}'" in tracking
    assert 'msg.pose.covariance = pose_cov' in tracking
    assert 'msg.twist.covariance = twist_cov' in tracking
    assert 'msg.twist.twist.linear.x = float(tr.state[3])' in tracking


def test_interception_tracks_state_uses_odometry_while_point_paths_stay_simplified() -> None:
    node = _read('src/gazebo_target_sim/gazebo_target_sim/interception_logic_node.py')

    assert 'tracks       -> Point on tracks_topic        (legacy; KF velocity dropped)' in node
    assert 'tracks_state -> Odometry on tracks_state_topic (uses KF position + velocity + cov)' in node
    assert "elif _meas_src in ('tracks_state', 'track_state', 'odometry'):" in node
    assert 'self._meas_use_filter_vel = True' in node
    assert 'self.create_subscription(Odometry, tgt_topic, self._on_target_state, 10)' in node
    assert 'self.create_subscription(Point, tgt_topic, self._on_target, 10)' in node
    assert 'self._target_filter_velocity = (' in node
    assert 'self._target = msg' in node
