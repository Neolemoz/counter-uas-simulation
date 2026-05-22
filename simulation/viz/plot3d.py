from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt


def plot_trajectories(
    target: np.ndarray,
    naive: np.ndarray,
    predictive: np.ndarray,
    title: str = "3D trajectories",
) -> None:
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        target[:, 0],
        target[:, 1],
        target[:, 2],
        color="C0",
        label="Target",
        linewidth=2,
    )
    ax.plot(
        naive[:, 0],
        naive[:, 1],
        naive[:, 2],
        color="C2",
        label="Naive",
        linewidth=2,
    )
    ax.plot(
        predictive[:, 0],
        predictive[:, 1],
        predictive[:, 2],
        color="C1",
        label="Predictive",
        linewidth=2,
    )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8)
    plt.tight_layout()
    plt.show()


def plot_intercept_single(
    p_t0: np.ndarray,
    p_i0: np.ndarray,
    target: np.ndarray,
    interceptor: np.ndarray,
    p_hit: np.ndarray,
    title: str = "3D intercept",
) -> None:
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        target[:, 0],
        target[:, 1],
        target[:, 2],
        color="C0",
        label="Target",
        linewidth=2,
    )
    ax.plot(
        interceptor[:, 0],
        interceptor[:, 1],
        interceptor[:, 2],
        color="C1",
        label="Interceptor",
        linewidth=2,
    )

    ax.scatter(*p_t0, color="C0", s=60, marker="o", label="Target start", depthshade=True)
    ax.scatter(*p_i0, color="C1", s=60, marker="o", label="Interceptor start", depthshade=True)
    ax.scatter(*p_hit, color="red", s=120, marker="X", label="Intercept", depthshade=True)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8)
    plt.tight_layout()
    plt.show()
