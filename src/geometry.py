# === CELL 2: TRAJECTORY GEOMETRY (3D) ===
"""
Geometry helpers for building 3D trajectories and measuring separation
between drones in shared airspace.
"""

from typing import List

import numpy as np

from scenario import Waypoint, CONFIG, define_perimeter_scan_mission, define_scheduled_traffic

# Short alias so we do not have to reach into CONFIG everywhere.
SAFETY_RADIUS_M = CONFIG.safety_radius_m


def interpolate_trajectory_3d(
    waypoints: List[Waypoint],
    samples_per_segment: int | None = None,
):
    """
    Turn a list of 4D waypoints into a sampled 3D trajectory.

    Assumptions:
    - Motion between waypoints is piecewise linear in x, y, z.
    - Time stamps in the waypoints are monotonically increasing.

    This keeps the model simple on purpose: the assignment is assessing
    strategic deconfliction, not low‑level flight dynamics.
    """
    if samples_per_segment is None:
        samples_per_segment = CONFIG.samples_per_segment

    # Extract time and position arrays
    times = np.array([wp.time for wp in waypoints])
    pts = np.array([[wp.x, wp.y, wp.z] for wp in waypoints])

    # Decide how many samples we want across the whole path
    total_samples = max(len(waypoints) * samples_per_segment, 2)
    t_interp = np.linspace(times[0], times[-1], total_samples)

    # Interpolate each coordinate independently over time
    x = np.interp(t_interp, times, pts[:, 0])
    y = np.interp(t_interp, times, pts[:, 1])
    z = np.interp(t_interp, times, pts[:, 2])

    # Stack x, y, z into a (N, 3) array and return with the time vector
    return np.column_stack([x, y, z]), t_interp


def compute_min_separation(traj1: np.ndarray, traj2: np.ndarray) -> float:
    """
    Compute the minimum 3D separation (in meters) between two trajectories.

    traj1, traj2: arrays of shape (N, 3) and (M, 3) in the same coordinate
                  frame. We compute all pairwise distances and return the
                  smallest one.
    """
    diffs = traj1[:, np.newaxis, :] - traj2[np.newaxis, :, :]
    dists = np.linalg.norm(diffs, axis=2)
    return float(np.min(dists))


# Quick geometry sanity check on the current scenario
if __name__ == "__main__":
    mission = define_perimeter_scan_mission()
    traffic0 = define_scheduled_traffic()[0]
    traj_m, _ = interpolate_trajectory_3d(mission["waypoints"])
    traj_t, _ = interpolate_trajectory_3d(traffic0["waypoints"])
    print(
        "Example separation to early cargo:",
        f"{compute_min_separation(traj_m, traj_t):.1f} m",
    )
