# === CELL 3: STRATEGIC CLEARANCE ENGINE ===
"""
Core decision logic for the strategic clearance engine.

Given the perimeter-scan mission and scheduled traffic around the hub,
this module decides whether the mission is SAFE TO FLY, attaches a
qualitative risk level, and produces a human-readable summary.
"""

from typing import Any, Dict


def time_windows_overlap(t1_start: float, t1_end: float,
                         t2_start: float, t2_end: float) -> bool:
    """
    Return True if two mission time windows overlap in time.

    Overlap is defined as any non-empty intersection between intervals
    [t1_start, t1_end] and [t2_start, t2_end].
    """
    return max(t1_start, t2_start) < min(t1_end, t2_end)


def classify_risk_level(worst_sep: float, safety_radius: float) -> str:
    """
    Classify overall mission risk based on worst 3D separation.

    - "high"   : safety radius was violated at least once
    - "medium" : stayed clear but came within 2x safety radius
    - "low"    : comfortably separated from all scheduled traffic
    """
    if worst_sep == float("inf"):
        # No relevant traffic (no time overlap), treat as low risk.
        return "low"
    if worst_sep < safety_radius:
        return "high"
    if worst_sep < 2 * safety_radius:
        return "medium"
    return "low"


def evaluate_mission_clearance(
    mission: Dict[str, Any] | None = None,
    config: DeconflictionConfig = CONFIG,
) -> Dict:
    """
    Evaluate whether the perimeter-scan mission can be flown safely.

    Returns a dictionary with:
    - status: "clear" | "blocked"
    - risk_level: "low" | "medium" | "high"
    - worst_separation_m: float | None
    - conflicts: list of per-flight conflict details
    - mission, config: echoed back for convenience
    """
    if mission is None:
        mission = define_perimeter_scan_mission()

    mission_traj, _ = interpolate_trajectory_3d(mission["waypoints"])
    conflicts: list[Dict[str, Any]] = []
    worst_separation = float("inf")

    for flight in define_scheduled_traffic():
        # Skip flights whose time windows do not overlap with the mission.
        if not time_windows_overlap(
            mission["t_start"], mission["t_end"],
            flight["t_start"], flight["t_end"],
        ):
            continue

        traffic_traj, _ = interpolate_trajectory_3d(flight["waypoints"])
        separation = compute_min_separation(mission_traj, traffic_traj)
        worst_separation = min(worst_separation, separation)

        # Record a conflict only when we actually violate the safety radius.
        if separation < config.safety_radius_m:
            conflicts.append({
                "drone_id": flight["id"],
                "role": flight["role"],
                "min_separation_m": round(separation, 1),
                "overlap_window_s": (
                    max(mission["t_start"], flight["t_start"]),
                    min(mission["t_end"],  flight["t_end"]),
                ),
            })

    status = "clear" if not conflicts else "blocked"
    risk_level = classify_risk_level(worst_separation, config.safety_radius_m)

    return {
        "status": status,
        "risk_level": risk_level,   # extra feature beyond spec
        "worst_separation_m": None if worst_separation == float("inf")
                              else round(worst_separation, 1),
        "conflicts": conflicts,
        "mission": mission,
        "config": config,
    }


def summarize_clearance(result: Dict) -> str:
    """
    Produce a concise, human-readable summary for logs or operator UI.
    """
    if result["status"] == "clear":
        return (
            f"Mission {result['mission']['mission_id']} CLEARED "
            f"with risk level {result['risk_level']} "
            f"(worst separation {result['worst_separation_m']} m)."
        )

    lines = [
        f"Mission {result['mission']['mission_id']} BLOCKED "
        f"(risk level {result['risk_level']}).",
        "Conflicts detected:",
    ]
    for c in result["conflicts"]:
        start, end = c["overlap_window_s"]
        lines.append(
            f"- {c['drone_id']} [{c['role']}] violates safety radius "
            f"({c['min_separation_m']} m) between t={start}s and t={end}s."
        )
    return "\n".join(lines)


# Run once and print summary for the default mission
decision = evaluate_mission_clearance()
print(summarize_clearance(decision))

