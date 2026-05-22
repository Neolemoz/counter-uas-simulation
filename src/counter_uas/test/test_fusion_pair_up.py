"""Regression tests for fusion's pair-up gate.

Background
----------
``fusion_node._publish_fused`` historically published once per **callback** — i.e. when
radar arrived it published immediately (with whatever camera was last seen, which might be
hundreds of milliseconds stale at km-scale → a 10–15 m position delta), and again a few
milliseconds later when camera arrived (now both fresh, ~0.5 m delta).  Tracking saw both
points within a single 100 ms cycle and spawned **two** candidates with positions ~10 m
apart, neither of which could associate cleanly on subsequent cycles.

The fix introduces ``fusion.require_paired_inputs`` (default True): the first callback in
a "round" stages its sample but does not publish; the partner's callback publishes the
fused mean and clears both flags.  This test exercises the freshness logic at the
function level (no rclpy spin) so the fix is locked in by CI.
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_fusion_module():  # noqa: ANN201
    """Import ``fusion_node`` without spinning rclpy.

    We pre-stub ``rclpy``, ``rclpy.node``, and ``geometry_msgs.msg`` so the import side
    effects do not require a ROS environment.  The stub ``Node`` is a no-op base class
    that exposes the methods fusion_node calls during ``__init__``.
    """
    rclpy_mod = types.ModuleType('rclpy')
    rclpy_mod.init = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.shutdown = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.spin = MagicMock()  # type: ignore[attr-defined]
    rclpy_node_mod = types.ModuleType('rclpy.node')

    class _StubParam:
        def __init__(self, value):  # noqa: ANN001
            self._value = value

        @property
        def value(self):  # noqa: ANN201
            return self._value

    class _StubNode:
        def __init__(self, name: str) -> None:  # noqa: ARG002
            self._params: dict[str, object] = {}

        def declare_parameter(self, name: str, default):  # noqa: ANN001, ANN201
            self._params.setdefault(name, default)

        def get_parameter(self, name: str):  # noqa: ANN201
            return _StubParam(self._params.get(name))

        def create_publisher(self, *_args, **_kwargs):  # noqa: ANN201
            mock = MagicMock()
            mock.publish = MagicMock()
            return mock

        def create_subscription(self, *_args, **_kwargs):  # noqa: ANN201
            return MagicMock()

        def get_logger(self):  # noqa: ANN201
            log = MagicMock()
            log.info = MagicMock()
            log.warning = MagicMock()
            return log

    rclpy_node_mod.Node = _StubNode  # type: ignore[attr-defined]

    geom_mod = types.ModuleType('geometry_msgs.msg')

    class _Point:
        __slots__ = ('x', 'y', 'z')

        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    geom_mod.Point = _Point  # type: ignore[attr-defined]
    geom_pkg = types.ModuleType('geometry_msgs')
    geom_pkg.msg = geom_mod  # type: ignore[attr-defined]

    sys.modules.setdefault('rclpy', rclpy_mod)
    sys.modules.setdefault('rclpy.node', rclpy_node_mod)
    sys.modules.setdefault('geometry_msgs', geom_pkg)
    sys.modules['geometry_msgs.msg'] = geom_mod

    path = _REPO_ROOT / 'src' / 'fusion' / 'fusion' / 'fusion_node.py'
    spec = importlib.util.spec_from_file_location('fusion_node_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _make_point(mod, x: float, y: float, z: float):  # noqa: ANN001, ANN201
    p = mod.Point()
    p.x, p.y, p.z = x, y, z
    return p


def test_pair_up_publishes_only_after_both_fresh() -> None:
    """The freshness flags must require BOTH sensors to have arrived since the last
    publish before emitting a fused point — radar alone (or camera alone) must not
    publish in pair-up mode.
    """
    mod = _load_fusion_module()
    node = mod.FusionNode()
    pub = node._pub.publish

    radar = _make_point(mod, 100.0, 0.0, 50.0)
    camera = _make_point(mod, 100.5, -0.2, 50.1)

    # Radar arrives first — must NOT publish yet (camera not fresh).
    node._on_radar(radar)
    assert pub.call_count == 0

    # Camera arrives — now both fresh, exactly one fused publish.
    node._on_camera(camera)
    assert pub.call_count == 1


def test_pair_up_resets_freshness_after_publish() -> None:
    """A second radar message right after a publish must NOT trigger another publish
    until camera has refreshed too.  This is the property that collapses the old
    "publish twice per cycle" pattern into one.
    """
    mod = _load_fusion_module()
    node = mod.FusionNode()
    pub = node._pub.publish

    node._on_radar(_make_point(mod, 100.0, 0.0, 50.0))
    node._on_camera(_make_point(mod, 100.5, 0.0, 50.0))
    assert pub.call_count == 1

    # Radar refreshes — partner is now stale → should NOT publish.
    node._on_radar(_make_point(mod, 110.0, 0.0, 50.0))
    assert pub.call_count == 1

    # Camera catches up → second pair publish.
    node._on_camera(_make_point(mod, 110.4, 0.0, 50.0))
    assert pub.call_count == 2


def test_legacy_mode_still_publishes_per_callback() -> None:
    """Setting ``fusion.require_paired_inputs=False`` restores the old behaviour: each
    radar/camera callback publishes immediately.  Useful for radar-only or camera-only
    sims and for diffing against historic behaviour during the migration.
    """
    mod = _load_fusion_module()
    node = mod.FusionNode()
    node._require_paired = False  # bypass param for the legacy path
    pub = node._pub.publish

    node._on_radar(_make_point(mod, 100.0, 0.0, 50.0))
    assert pub.call_count == 1, 'legacy mode: radar-only publishes the radar Point'
    node._on_camera(_make_point(mod, 100.5, 0.0, 50.0))
    assert pub.call_count == 2, 'legacy mode: each callback publishes once'
