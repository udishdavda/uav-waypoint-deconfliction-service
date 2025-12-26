# FlytHub UTM Clearance Engine

**Real-time 4D (x, y, z, t) airspace conflict detection engine** for drone traffic management. This project determines whether a proposed drone mission can safely operate without violating minimum separation rules against scheduled aerial traffic.

---

## 🎯 Assignment Overview

### Problem Statement  
Develop a clearance engine that evaluates a proposed drone mission and classifies it as **"clear"** or **"blocked"** depending on whether it violates the 25 m safety separation requirement with any existing mission in 4D space (x, y, z, time).

### Solution Summary  
A modular and testable Python-based engine capable of:
- 4D conflict detection using spatial and temporal overlap.
- Trajectory interpolation of missions between waypoints.
- Configurable safety radius verification.
- JSON-like output containing clearance status and conflict details.

---

## 🚀 Quick Start

To run the demo and reproduce results locally:

```bash
pip install -r requirements.txt
jupyter notebook notebooks/flyt_hub_clearance.ipynb
```

**Demonstration:**  
The main notebook walks through a perimeter-scan mission and shows how it is **blocked** by a pre-scheduled cargo lane in the south corridor.

---

## 🏗️ Repository Structure

```
notebooks/                   <- Interactive notebook demo + test harness
src/clearance/               <- Core clearance engine modules
├── engine.py                <- Main API: evaluate_mission_clearance()
├── geometry.py              <- 3D geometry & time-based distance utilities
├── traffic.py               <- Scheduled drones & waypoints database
└── constants.py             <- Safety radius and default parameters
tests/                       <- Pytest suite verifying core logic
requirements.txt             <- Dependency definitions
```

---

## 📊 Key Features

- **4D Collision Detection:**  
  Evaluates drone conflicts by combining 3D distance with time-window overlap.

- **Trajectory Interpolation:**  
  Generates smooth flight paths using linear interpolation between waypoints.

- **Configurable Safety Radius:**  
  Default set to `SAFETY_RADIUS_M = 25`.

- **Time-Aware Analysis:**  
  Distinguishes spatial crossings that do *not* overlap in time as safe.

- **Modular + Testable:**  
  Designed for integration in production or further extension into a full UTM system.

---

## 🧪 Testing and Validation

| Test Case | Purpose | Expected Outcome |
|------------|----------|-----------------|
| `conflict_detected` | Validates detection of known cargo-lane conflict | ✅ Blocked |
| `no_conflict_when_far_apart` | Mission offset by 1 km shows as clear | ✅ Clear |
| `3d_distance_uses_altitude` | Confirms altitude difference affects distance | ✅ Clear |
| `exact_safety_buffer_is_clear` | Exactly 25 m separation considered safe | ✅ Clear |
| `no_conflict_when_no_time_overlap` | Temporal offset removes conflict | ✅ Clear |

### Run the full test suite

```bash
pytest tests/ -v
```

---

## 💻 Example Usage

```python
from src.clearance.engine import evaluate_mission_clearance, define_perimeter_scan_mission

mission = define_perimeter_scan_mission()
result = evaluate_mission_clearance(mission)

if result["status"] == "clear":
    print("✅ Mission approved")
else:
    print("❌ Blocked by:", [c["drone_id"] for c in result["conflicts"]])
```

---

## ⚙️ How It Works

### Algorithm Workflow
1. **Mission Parsing:**  Convert mission waypoints into 4D (x,y,z,t) representation.  
2. **Interpolation:**  Build dense trajectories for proposed and scheduled paths.  
3. **Temporal Filtering:**  Only compare flights during overlapping time intervals.  
4. **Distance Calculation:**  Compute minimal 3D point-to-point distances.  
5. **Decision Logic:**  Return `"blocked"` if any distance < 25 m; `"clear"` otherwise.

### Mathematical Core
- Distance: d = √[(x₁ - x₂)² + (y₁ - y₂)² + (z₁ - z₂)²]
- Temporal Window: Mission overlap determined by [t_start, t_end] intervals.

### Performance
Average runtime: **< 100 ms per mission check**  
Complexity: O(N×M×S²) — scalable for real-time usage with modest traffic volumes.

---

## 📦 Dependencies (requirements.txt)

```
numpy
matplotlib
pytest
pytest-sugar
```

Install all dependencies via:

```bash
pip install -r requirements.txt
```

---

## ✅ Assignment Requirements Coverage

- ✔ **4D Conflict Detection** (x, y, z, time)
- ✔ **Safety Radius (25 m)** via configurable constant
- ✔ **Deterministic API Output** (`evaluate_mission_clearance`)
- ✔ **Extensive Test Suite (5+)** using pytest
- ✔ **Notebook Demonstration + Visualization**
- ✔ **Clean Modular Repo Structure**
- ✔ **Compliance with FlytHub UTM Challenge specifications**

---

## 🧠 Notes for Reviewers

- The notebook and test harness are aligned with the FlytHub UTM evaluation PDF.
- Each test represents a unique scenario verifying spatial, temporal, and boundary conditions.
- The codebase includes explicit comments, modularity, and reproducibility.

---

## 👨‍🔧 Author

**Mechatronics Engineer** — UNSW Sydney (Honours)  
**Specialization:** Automation & Control Systems, Robotics, Mechanical design  
**GitHub:** [Udish Davda](https://github.com/udishdavda)
