# === CELL 1: SCENARIO & CONFIGURATION (SMART HUB NIGHT SCAN) ===
"""
Defines the core data structures, configuration, and airspace scenario
for a night-time perimeter scan around a smart logistics hub near Pune.

This module is intentionally domain-focused: no geometry or deconfliction
logic here, just the "world" the engine operates in.
"""

from dataclasses import dataclass
from typing import List, Dict

import numpy as np  # used later for trajectory generation, imported here once


# --- Core data types ---------------------------------------------------------


@dataclass
class Waypoint:
    """Single 4D waypoint along a drone trajectory."""
    x: float          # meters (local X in hub frame)
    y: float          # meters (local Y in hub frame)
    z: float          # meters AGL (altitude)
    time: float       # seconds from mission start


@dataclass
class DeconflictionConfig:
    """
    Tunable configuration for the strategic clearance engine.

    safety_radius_m:     minimum allowed 3D separation between drones.
    samples_per_segment: how densely to sample between waypoints when
                         constructing trajectories for distance checks.
    """
    safety_radius_m: float = 25.0      # assignment safety radius (meters)
    samples_per_segment: int = 10      # reasonable default for tests/demo


# Single global config instance used by the rest of the code
CONFIG = DeconflictionConfig()


# --- Primary mission: perimeter scan ----------------------------------------


def define_perimeter_scan_mission() -> Dict:
    """
    Night-time perimeter scan around a smart logistics hub near Pune.

    The drone takes off near the south‑west corner of the warehouse,
    flies a rectangular loop around the roof at low altitude, and
    must finish within a 10‑minute slot before morning traffic ramps up.
    """
    perimeter_alt = 60.0  # low layer, below most transit traffic
    return {
        "mission_id": "night_perimeter_scan_pune_hub_v1",
        "waypoints": [
            # SW -> SE -> NE -> NW (warehouse perimeter)
            Waypoint(0,   -20, perimeter_alt,   0),   # SW
            Waypoint(120, -20, perimeter_alt,  60),   # SE
            Waypoint(120,  80, perimeter_alt, 180),   # NE
            Waypoint(0,    80, perimeter_alt, 300),   # NW
        ],
        "t_start": 0,
        "t_end": 600,  # 10‑minute clearance window
    }


# --- Other scheduled traffic around the hub ---------------------------------


def define_scheduled_traffic() -> List[Dict]:
    """
    Simulated traffic around the hub:

    - early-slot cargo corridor along south edge (low‑mid altitude)
    - diagonal urban delivery corridor above the hub
    - high‑altitude emergency-response corridor cutting across the site
    """
    return [
        {
            "id": "early_cargo_south_corridor",
            "role": "cargo",
            "waypoints": [
                Waypoint(-50, -30, 80, 120),
                Waypoint(200, -30, 80, 260),
            ],
            "t_start": 120,
            "t_end": 260,
        },
        {
            "id": "urban_diag_delivery",
            "role": "delivery",
            "waypoints": [
                Waypoint(-50, -50, 100, 100),
                Waypoint(200, 200, 100, 400),
            ],
            "t_start": 100,
            "t_end": 400,
        },
        {
            "id": "emergency_overpass_lane",
            "role": "emergency",
            "waypoints": [
                Waypoint(0,   150, 130,  50),
                Waypoint(150, -50, 130, 250),
            ],
            "t_start": 50,
            "t_end": 250,
        },
    ]


if __name__ == "__main__":
    print("Scenario: night perimeter scan around smart hub loaded.")
    print("Scheduled traffic drones:", len(define_scheduled_traffic()))
