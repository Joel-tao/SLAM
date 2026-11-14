# BUILD / MAP

**2D LiDAR mapping, construction-change detection and design-based placement inspection.**

A simulated robot surveys a room twice. A new structural element appears during the second lap. The demo estimates its trajectory from noisy odometry and scan-to-map ICP, builds an endpoint point-cloud map, flags persistent surfaces absent from a frozen baseline, and fits a known-size rectangular element to estimate installation deviation from a pre-aligned design.

![Demo](artifacts/demo.gif)

## Run

Python 3.11 recommended. No ROS, GPU, or Gazebo required for this reproducible experiment.

```bash
python3 -m venv .demo-venv
source .demo-venv/bin/activate
pip install -r requirements-lock.txt
python -m construction_slam.demo --gif
python -m unittest discover -s tests -v
```

On the original workstation the prepared interpreter is `.demo-venv/bin/python`.
If system Python lacks `venv`, use an installed Python with venv support; this workstation used `/home/joe/anaconda3/bin/python`.

Outputs: `artifacts/demo.gif`, `overview.png`, `metrics.json`, `trajectory.csv`, `map_points.npy`, `construction_state.json`, `inspection_history.json`. Change the random seed with `--seed 42 --output artifacts/seed42`. GIF rendering is optional and takes longer than the numerical experiment.

## Architecture

```text
Ray-cast world → noisy LiDAR scans ───────────────┐
Simulated motion → biased noisy odometry → SE(2) prediction
                                                 ↓
                                   trimmed point-to-point ICP
                                                 ↓
                                  pose + voxel-filtered map
                                                 ↓
                         frozen baseline → persistent novel cells
Truth ───────────────────────────────→ evaluation only
```

- SE(2) transformations and motion composition.
- Nearest-neighbor ICP with SVD rigid alignment, correspondence gating and trimming.
- Incremental endpoint map with 6 cm voxel filtering; failed scans are not inserted.
- Baseline frozen after one lap; cells >30 cm from baseline need three consecutive observations.
- Shared-origin trajectory RMSE, CSV export, and reproducible animation.
- Known-size rectangle fitting with robust least squares, observability checks, translation/yaw tolerances and a planner-facing JSON report.
- [Design inspection details (中文)](docs/INSPECTION_ZH.md).

The estimator does not receive true poses or world geometry. Ground-truth motion generates synthetic noisy odometry, as part of the simulator; truth is also used for evaluation. Initial map frame is the first scan frame. The route and insertion time are scripted, not planned autonomously.

## Measured example

Seed 7, 240 frames, 360 rays per scan, 12 mm range noise, 3.5% translation bias plus odometry noise and heading bias:

| Metric | Result |
|---|---:|
| Odometry position RMSE | 0.578 m |
| Scan-to-map position RMSE | 0.017 m |
| Rejected scans | 0 |
| Peak persistent novel cells | 9 |

These are results in one simple synthetic scene, not real-robot accuracy claims. No post-hoc trajectory alignment is applied. Novel-cell count is not detection precision or recall.

## Scope and limitations

This is a small mapping/localization front end, **not a complete production SLAM system**. No loop closure, pose graph, covariance, IMU fusion, 3D reconstruction, multi-robot fusion, or navigation. The map contains occupied endpoints, not a free/occupied/unknown occupancy grid. New surfaces are added immediately to the evolving map; change detection separately compares against the frozen baseline. There is no semantic object recognition or removal detection. Design comparison is limited to one stationary known-size 2D rectangle in a pre-aligned frame; it is not CAD/BIM registration or 6DoF pose estimation. The JSON action is advisory; no planner is connected. Previously unseen static surfaces and localization errors can trigger false positives. Baseline freezing uses the scripted lap boundary. Motion is small and overlap is high; corridors, large jumps and low overlap can defeat ICP. Rejected frames fall back to odometry.

## Placement inspection

`config/design.json` defines the design pose, dimensions, and tolerances (8 cm / 5 degrees, demonstration values). The simulator deliberately adds 14.42 cm translation and 8 degrees rotation. Seed 7 estimates approximately 13.27 cm and 8.17 degrees and reports `deviation_detected`. Only scan-derived points enter the fitter. Insufficient geometry produces `insufficient_evidence`; stale observations request another view. Candidate association and quality gates are heuristic, with no probabilistic confidence guarantee.

## ROS2 / Gazebo extension

`ros2/scan_mapper.py` is an **experimental, runtime-unverified adapter** subscribing to `/scan` and `/odom` and publishing `/demo/path`. See [integration notes](docs/ROS2.md). The included simulation uses Python ray casting, not Gazebo.

## Learn and present

- [中文学习与面试指南](docs/LEARNING_ZH.md)
- [CV wording and application notes](docs/APPLICATION_ZH.md)
- [ROS2 integration scope](docs/ROS2.md)

Created with AI coding assistance as a learning scaffold. Review, reproduce, modify and explain the implementation before presenting it as personal project experience. Standard ICP is implemented here using NumPy/SciPy; no external SLAM package is wrapped.

## Publishing

See [GitHub instructions](docs/GITHUB_ZH.md). Source, configuration, tests, documentation and selected reproducible results belong in the repository; local virtual environments and caches do not.
