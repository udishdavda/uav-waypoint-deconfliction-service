# === CELL 4: 3D TRAJECTORY VISUALIZATION ===
"""
Visualization helpers for inspecting the airspace around the smart hub.

This plot is used in the demo video to show:
- the perimeter-scan mission (our drone)
- the scheduled traffic lanes
- the overall qualitative risk level for the mission
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 – required for 3D projection


def plot_3d_trajectories(result: Dict) -> None:
    """
    Render a 3D view of the mission and all scheduled traffic.

    result: output dictionary from evaluate_mission_clearance.
    """
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")

    # --- Primary mission: perimeter scan drone --------------------------------
    mission = result["mission"]
    traj_p, _ = interpolate_trajectory_3d(mission["waypoints"])
    ax.plot(
        traj_p[:, 0], traj_p[:, 1], traj_p[:, 2],
        color="blue", linewidth=3, label="Perimeter scan drone",
    )

    # --- Scheduled traffic around the hub -------------------------------------
    colors = {"cargo": "red", "delivery": "green", "emergency": "orange"}
    for flight in define_scheduled_traffic():
        traj, _ = interpolate_trajectory_3d(flight["waypoints"])
        ax.plot(
            traj[:, 0], traj[:, 1], traj[:, 2],
            color=colors[flight["role"]], label=flight["id"],
        )

    # --- Axes formatting and labels -------------------------------------------
    ax.set_title(f"3D Trajectories (risk={result['risk_level']})")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z (altitude) [m]")
    ax.legend()
    plt.tight_layout()
    plt.show()


# Render once for the current decision
plot_3d_trajectories(decision)

