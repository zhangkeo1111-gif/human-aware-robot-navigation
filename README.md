# Human-Aware Robot Navigation in Isaac Sim

Research code for human-aware navigation with a simulated Husky A300 in
NVIDIA Isaac Sim 6.0.1. The robot-camera pipeline uses RGB-D, COCO person
detection, robust depth, short-term association, and a constant-velocity
Kalman filter. The navigation experiments use estimated pedestrian states;
ground truth is reserved for offline evaluation.

## Version map

| Git branch | Scope | Status |
| --- | --- | --- |
| `main` | Prediction-aware social navigation | Frozen v0.3.0 (`v0.3.0-social-nav`) |
| `feature/active-passing` | Simple right-side passing and seven classic single-person scenarios | Experimental; one seed-17 run per scenario, not a safety benchmark |
| `feature/third-person-collision-prediction` | Fixed third-person RGB-D collision-risk prediction and YOLO11n/YOLO26n comparison | Experimental; predicts risk but does **not** control avoidance |

Other local exploratory branches and uncommitted work are not part of this
publication. Neither the simulator nor the model weights are bundled here.

## Stable v0.3.0 run

Use an existing Isaac Sim 6.0.1 installation and the local Python runtime
described in [REPRODUCIBILITY.md](REPRODUCIBILITY.md). From the repository root:

```powershell
.\runtime\python.bat stage_probe.py --stage 7 --robot-model a300 --motion-mode kinematic --navwareset-scene --navwareset-scenario circular --robot-behavior social_nav --record-demo --record-first-person
```

The simulator runs the A300 kinematic backend; the perception stack supplies
human position and motion estimates to the controller. This is functional
simulation evidence, **not** a real-robot safety certification.

## Evidence and dependencies

- Stable release protocol and environment: [REPRODUCIBILITY.md](REPRODUCIBILITY.md)
- Stable social-navigation results: [SOCIAL_NAVIGATION_REPORT.md](outputs/social_navigation/SOCIAL_NAVIGATION_REPORT.md)
- Active-passing code and tests: `feature/active-passing`
- Third-person collision-prediction code and compact result reports: `feature/third-person-collision-prediction`
- Official Clearpath A300 descriptions and Isaac Sim character assets are external dependencies; see [REPRODUCIBILITY.md](REPRODUCIBILITY.md).

Raw runs, model weights, downloaded assets, caches, and MP4 videos remain
local. Only code, configuration, small asset wrappers, and compact evaluation
evidence are tracked by Git.
