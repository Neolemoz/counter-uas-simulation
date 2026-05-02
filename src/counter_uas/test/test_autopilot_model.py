"""Phase 5: smoke tests for the autopilot delay/bandwidth model.

We can't easily import ``InterceptorControllerNode`` (it constructs a ROS node), so we test the
mathematical core (FIFO delay + first-order bandwidth) by re-implementing the same recurrence
the node uses.  Any drift in the node's logic should be caught by porting the change here.
"""

from __future__ import annotations

from collections import deque

import pytest


def _delay_fifo(buf: deque, cmd: tuple[float, float, float]) -> tuple[float, float, float]:
    """Mirror ``InterceptorControllerNode._apply_cmd_delay`` for testing."""
    if len(buf) == 0:
        return cmd
    buf.append(cmd)
    return buf.popleft()


def _first_order_step(prev: tuple[float, float, float], cmd: tuple[float, float, float], dt: float, tau: float) -> tuple[float, float, float]:
    a = dt / (dt + tau)
    return (
        prev[0] + a * (cmd[0] - prev[0]),
        prev[1] + a * (cmd[1] - prev[1]),
        prev[2] + a * (cmd[2] - prev[2]),
    )


def test_delay_fifo_zero_length_passthrough() -> None:
    buf: deque = deque()
    assert _delay_fifo(buf, (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


def test_delay_fifo_fixed_lag() -> None:
    """A 3-step FIFO seeded with zeros emits the input three steps later."""
    buf = deque([(0.0, 0.0, 0.0)] * 3)  # no maxlen, matches the node implementation
    out0 = _delay_fifo(buf, (1.0, 0.0, 0.0))
    out1 = _delay_fifo(buf, (2.0, 0.0, 0.0))
    out2 = _delay_fifo(buf, (3.0, 0.0, 0.0))
    out3 = _delay_fifo(buf, (4.0, 0.0, 0.0))
    assert out0 == (0.0, 0.0, 0.0)
    assert out1 == (0.0, 0.0, 0.0)
    assert out2 == (0.0, 0.0, 0.0)
    assert out3 == (1.0, 0.0, 0.0)


def test_first_order_reaches_step_input() -> None:
    """Step input drives the response asymptotically toward the command."""
    state = (0.0, 0.0, 0.0)
    tau = 0.2
    dt = 0.01
    target = (10.0, 0.0, 0.0)
    for _ in range(int(5 * tau / dt)):  # 5 time constants
        state = _first_order_step(state, target, dt, tau)
    # After 5*tau the response should be > 99% of target.
    assert state[0] == pytest.approx(target[0], rel=0.02)


def test_first_order_63_pct_at_one_tau() -> None:
    """First-order tracking reaches ~63% of a step at t = tau (canonical bandwidth check)."""
    state = (0.0, 0.0, 0.0)
    tau = 0.5
    dt = 0.01
    target = (1.0, 0.0, 0.0)
    n = int(round(tau / dt))
    for _ in range(n):
        state = _first_order_step(state, target, dt, tau)
    # Discrete approximation of 1 - exp(-1) ≈ 0.6321; allow a small numerical margin for the
    # forward-Euler discretisation used in the node.
    assert 0.55 < state[0] < 0.7
