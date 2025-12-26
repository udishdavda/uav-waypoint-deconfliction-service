# === CELL 5: TEST HARNESS (NOTEBOOK VERSION) ===
"""
Lightweight test harness for the clearance engine.

In the repo, these tests live in tests/test_engine.py and are run by
pytest. Inside this notebook we keep a simple run_all_tests() wrapper
so we can quickly verify behavior after making changes.
"""

import math


def test_conflict_detected():
    """
    Sanity check: with the default scenario, at least one conflict
    must be detected (early cargo lane near the south edge).
    """
    result = evaluate_mission_clearance()
    assert result["status"] == "blocked"
    assert any(
        c["drone_id"] == "early_cargo_south_corridor"
        for c in result["conflicts"]
    )


def test_no_conflict_when_far_apart():
    """
    If we translate the mission 1 km east, it should be clear of all
    scheduled traffic in both space and time.
    """
    mission = define_perimeter_scan_mission()
    for wp in mission["waypoints"]:
        wp.x += 1000  # move 1 km east

    result = evaluate_mission_clearance(mission)
    assert result["status"] == "clear"


def test_3d_distance_uses_altitude():
    """
    Two vertical points with identical x,y but 30 m difference in z
    should have a 3D separation of exactly 30 m.
    """
    w1 = Waypoint(0, 0, 0, 0)
    w2 = Waypoint(0, 0, 30, 10)

    traj1, _ = interpolate_trajectory_3d(
        [w1, Waypoint(0, 0, 0, 10)], samples_per_segment=1
    )
    traj2, _ = interpolate_trajectory_3d(
        [w2, Waypoint(0, 0, 30, 20)], samples_per_segment=1
    )
    d = compute_min_separation(traj1, traj2)
    assert math.isclose(d, 30.0, rel_tol=1e-3)


def test_exact_safety_buffer_is_clear():
    """
    Two straight, parallel segments exactly SAFETY_RADIUS_M apart
    should be treated as clear (no violation).
    """
    a1 = Waypoint(0, 0, 100, 0)
    a2 = Waypoint(100, 0, 100, 10)

    b1 = Waypoint(0, SAFETY_RADIUS_M, 100, 0)
    b2 = Waypoint(100, SAFETY_RADIUS_M, 100, 10)

    traj1, _ = interpolate_trajectory_3d([a1, a2], samples_per_segment=1)
    traj2, _ = interpolate_trajectory_3d([b1, b2], samples_per_segment=1)
    d = compute_min_separation(traj1, traj2)
    assert math.isclose(d, SAFETY_RADIUS_M, rel_tol=1e-3)


def test_no_conflict_when_no_time_overlap():
    """
    Reuse the same spatial path but shift the mission far forward in time.
    With no temporal overlap, the mission should be clear.
    """
    mission = define_perimeter_scan_mission()
    shifted = {
        "mission_id": mission["mission_id"] + "_shifted",
        "waypoints": [],
        "t_start": mission["t_start"] + 10_000,
        "t_end":   mission["t_end"]   + 10_000,
    }

    for wp in mission["waypoints"]:
        shifted["waypoints"].append(
            Waypoint(wp.x, wp.y, wp.z, wp.time + 10_000)
        )

    result = evaluate_mission_clearance(shifted)
    assert result["status"] == "clear"


def run_all_tests() -> None:
    """
    Execute all test_* functions and print a compact status line
    for each. This mirrors a tiny subset of what pytest would do.
    """
    tests = [
        ("conflict_detected", test_conflict_detected),
        ("no_conflict_when_far_apart", test_no_conflict_when_far_apart),
        ("3d_distance_uses_altitude", test_3d_distance_uses_altitude),
        ("exact_safety_buffer_is_clear", test_exact_safety_buffer_is_clear),
        ("no_conflict_when_no_time_overlap", test_no_conflict_when_no_time_overlap),
    ]
    for name, fn in tests:
        try:
            fn()
            print(f"✅ {name} passed")
        except AssertionError as e:
            print(f"❌ {name} FAILED: {e}")


# Run the tests once for this notebook
run_all_tests()

