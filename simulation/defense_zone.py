"""Radar detection volume: upper hemisphere (yellow) on ground, ground rim ring, station marker."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

# Max horizontal range / sphere radius for detection (meters), e.g. 10 km
R_COVERAGE_M = 10000.0


def create_radar_hemisphere(
    R: float,
    n_phi: int = 72,
    n_theta: int = 144,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Upper hemisphere of radius R centered at origin: rim on z=0, peak at (0,0,R).

    Spherical: θ ∈ [0, 2π] azimuth, φ ∈ [0, π/2] from +z toward horizon.
    x = R sin φ cos θ, y = R sin φ sin θ, z = R cos φ.
    """
    Rf = float(R)
    if Rf <= 0.0 or n_phi < 2 or n_theta < 3:
        z0 = np.zeros((1, 1), dtype=float)
        return z0, z0, z0

    phi = np.linspace(0.0, np.pi / 2.0, n_phi)
    theta = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=True)
    Phi, Theta = np.meshgrid(phi, theta, indexing="ij")
    sp = np.sin(Phi)
    x = Rf * sp * np.cos(Theta)
    y = Rf * sp * np.sin(Theta)
    z = Rf * np.cos(Phi)
    return x, y, z


def create_ground_rim_ring(R: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Max range circle on the ground (z=0) only."""
    Rf = float(R)
    th = np.linspace(0.0, 2.0 * np.pi, 200)
    x = Rf * np.cos(th)
    y = Rf * np.sin(th)
    z = np.zeros_like(x, dtype=float)
    return x, y, z


def plot_radar_detection(
    R: float = R_COVERAGE_M,
    n_phi: int = 72,
    n_theta: int = 144,
) -> None:
    """Yellow hemisphere = detectable volume; blue = radar; red ring = max ground range (rim)."""
    xh, yh, zh = create_radar_hemisphere(R, n_phi=n_phi, n_theta=n_theta)
    xg, yg, zg = create_ground_rim_ring(R)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot_surface(
        xh,
        yh,
        zh,
        color="yellow",
        alpha=0.22,
        linewidth=0,
        edgecolor="none",
        antialiased=True,
        shade=True,
        rstride=1,
        cstride=1,
    )

    ax.plot(
        np.append(xg, xg[0]),
        np.append(yg, yg[0]),
        np.append(zg, zg[0]),
        color="red",
        linewidth=2.0,
    )

    ax.scatter(0.0, 0.0, 0.0, color="blue", s=80, depthshade=True, zorder=10)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(f"Radar detection (hemisphere, R = {R/1000:.0f} km)")

    Rf = float(R)
    z_top = Rf * 1.05
    ax.set_xlim(-Rf, Rf)
    ax.set_ylim(-Rf, Rf)
    ax.set_zlim(0.0, z_top)
    ax.set_box_aspect([1.0, 1.0, 0.45])
    ax.view_init(elev=28.0, azim=45.0)


if __name__ == "__main__":
    plot_radar_detection()
    plt.show()
