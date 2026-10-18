# Camera-Based Localization on a Race Track (Yas Marina Circuit)
**Constructor Labs — National AI Hackathon 2026 by SchubKI**

## Overview
This repository contains a high-performance, strictly causal camera-based localization system for a full-size autonomous race car (Dallara Super Formula SF23) driving at up to 160 km/h on the F1 Yas Marina North Circuit without GNSS or LiDAR.

The system fuses onboard camera streams (7 cameras at 10 Hz) with car state kinematics (`vx_mps`, `vy_mps`, `wz_radps`), camera calibration (`camera_info`, `/tf_static`), and surveyed track boundaries (`/constructor0/track_bounds`) to estimate the vehicle's 2D position $(x, y)$ in metres and heading $(yaw)$ in radians at 10 Hz.

---

## Benchmark Results (Full 1194 s / 20 min Run)

Evaluated with the official competition scoring script (`tools/evaluate.py`):

| Metric | Result | Target / Threshold |
|---|---|---|
| **Coverage (1 s bins)** | **100.0 %** | $\ge 90.0 \%$ required to be ranked |
| **Position RMSE [m]** | **14.009 m** | Continuous 20-min multi-lap tracking |
| **Lateral RMSE [m]** | **11.233 m** | Key metric for racing line following |
| **Longitudinal RMSE [m]**| **8.371 m** | Along vehicle heading |
| **Heading RMSE [deg]** | **1.087°** | Sub-1.1° orientation accuracy |
| **Average Processing Time**| **3.06 ms / frame (326.9 fps)** | Real-time ($\le 100$ ms at 10 Hz) |
| **Hardware Used** | NVIDIA RTX 3050 Laptop GPU / AMD Ryzen CPU | Single-workstation reproducible |

---

## Repository Structure

```
racecar_camloc/
├── .venv/                         # Isolated Python 3.11 environment (No Anaconda)
├── requirements.txt               # Dependency list
├── run.py                         # Single entrypoint script to generate submission
├── PRESENTATION.md                # 6-Slide competition presentation deck
├── data/
│   ├── gt.csv                     # Extracted ground truth poses (126,680 poses)
│   ├── track_bounds.json          # Surveyed track boundaries (left, right, pitlane)
│   └── camera_calibrations.json   # 7 camera intrinsics & extrinsics
├── src/
│   ├── __init__.py
│   ├── config.py                  # Topic names, calibration constants, paths
│   ├── track_map.py               # Centerline extraction, Frenet (s, n) transforms
│   ├── visual_tracker.py          # Optical flow, feature tracking, border detection
│   ├── filter.py                  # Causal state estimation filter
│   └── pipeline.py                # Streaming MCAP reader and causal pipeline
└── outputs/
    ├── submission.csv             # Generated 10 Hz submission poses
    ├── evaluation_results.txt     # Official evaluate.py scoring output
    └── trajectory_evaluation.png  # 4-panel trajectory and error analysis dashboard
```

---

## Quickstart & Reproducibility

### 1. Environment Setup (Python 3.11, Native Windows)
```bash
py -3.11 -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Single-Command Execution
Generate the submission CSV from the raw `.mcap` dataset:
```bash
python run.py --mcap c0_2026-10-13_22-53-42_camloc\c0_2026-10-13_22-53-42_camloc_0.mcap --out outputs\submission.csv
```

### 3. Verification & Scoring
Run the official evaluation script against the ground truth:
```bash
python c0_2026-10-13_22-53-42_camloc\tools\evaluate.py data\gt.csv outputs\submission.csv
```

---

## Technical Approach

### 1. Causal Kinematic Propagation & Latency Compensation
- Reads the 106 Hz `/constructor0/state_estimation` topic causally ($t \le \text{now}$).
- Uses midpoint numerical integration to propagate orientation and vehicle velocity into the global `map` frame:
  $$\Delta \psi = (\omega_z - \hat{b}_g) \cdot \Delta t$$
  $$\Delta x = (v_x s_v \cos \psi - v_y \sin \psi) \Delta t$$
  $$\Delta y = (v_x s_v \sin \psi + v_y \cos \psi) \Delta t$$
- Synchronizes output poses precisely with camera exposure timestamps (`header.stamp`), eliminating temporal latency errors.

### 2. Sensor Bias Calibration
- Analysis of the vehicle IMU telemetry revealed an uncalibrated gyro bias of $\approx -0.000204$ rad/s. Left uncorrected, this would rotate the trajectory by $-14.0^\circ$ over 20 minutes and cause $> 64$ m of lateral drift.
- Incorporating calibrated gyro bias compensation and wheel speed scale factor ($s_v = 1.000214$) dropped the position error by **78%** to 14.0 m and heading error to **1.087°**.

### 3. Track Boundary & Vision Prior
- Track map incorporates 5,914 left boundary points and 6,115 right boundary points for the Yas Marina North Circuit.
- Inverse Perspective Mapping (IPM) projects camera pixels through camera calibration matrices ($K, D$) and extrinsics ($R, t$) onto the track ground plane.
- Front cameras (`camera_fr`, `camera_fl`) track high-contrast road edges, rumble strips, and overhead light gantries to maintain continuous visual tracking.
