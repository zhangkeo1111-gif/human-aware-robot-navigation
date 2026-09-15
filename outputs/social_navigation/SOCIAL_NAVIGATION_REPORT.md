# Prediction-Aware Social Navigation: implementation and paired benchmark

## 1. System
A300, Isaac Sim 6.0.1, frozen NavWareSet-style scene, RGB-D 640×360 at 10 Hz simulation time, COCO YOLO11n (conf 0.25), robust foreground depth, Hungarian association and CV KF. No LiDAR dependency. This is simulation, not an exact reconstruction of the real dataset.

## 2. Baseline
`social` remains **Reactive Baseline / DemoController**, byte-identical to the saved source. It is not relabelled as prediction-aware social navigation. `social_nav` selects the independent new controller. The sensor input rejects repeated exposure timestamps before submitting them to YOLO; the initial development run demonstrated the previous duplicate-timestamp crash. The same correction applies to both controllers; KF equations and thresholds are unchanged.

Run one new-controller scene from the project directory:
```powershell
.\runtime\python.bat stage_probe.py --stage 7 --robot-model a300 --motion-mode kinematic --navwareset-scene --navwareset-scenario circular --robot-behavior social_nav --record-demo --record-first-person
```
Legacy `--robot-behavior social` remains available. `python social_benchmark.py benchmark` resumes missing fixed benchmark combinations without replacing completed runs; `python social_benchmark.py report` regenerates the CSV/report from saved run evidence. The complete common configuration is frozen in BENCHMARK_CONFIG_FROZEN.json.

## 3. SocialController
Seven acceleration-limited motion primitives, 31 future points per primitive, CV human prediction, moving anisotropic personal space, known-static-map rejection, hard human-disk rejection, goal/control/heading/switch/stop costs. All accepted active tracks enter the social sum, not only the closest person. Recent heading is retained for low-speed tracks; unknown heading uses isotropic cost. Tracks older than 0.8 s are excluded from normal prediction, with bounded near-danger memory. Emergency and no-safe-primitive fallbacks command STOP; braking still follows the unchanged backend and cannot guarantee avoidance of an unavoidable situation.

## 4. Equations / parameters
Human: `p_h(t) = p_h(now) + v_h*t`. Robot integrates `x_dot=v*cos(yaw), y_dot=v*sin(yaw), yaw_dot=omega` with 0.5 m/s² acceleration, 0.8 m/s² deceleration and 1.2 rad/s² angular acceleration, matching the backend. `C=exp(-0.5*((long/sigma_long)^2+(lat/sigma_side)^2))`; sigma_long is front or rear by sign. Planning inflates sigma using `sqrt(lambda_max(F P F^T)_xy)` and uses real exposed KF covariance. It does not add a fabricated covariance or rewrite KF. Hard center collision uses actual robot conservative radius plus 0.30 m, unchanged; static disk uses the same radius.

`J = w_goal*J_goal + w_social*(mean summed personal cost + group_weight*bridge) + w_control*J_control + w_heading*J_heading + w_switch*J_switch + w_stop*J_stop`.
All soft weights and social dimensions are engineering heuristics, not human-subject-calibrated proxemics constants. Final common config:
```json
{
  "prediction_horizon": 3.0,
  "prediction_dt": 0.1,
  "front_sigma": 1.2,
  "rear_sigma": 0.7,
  "side_sigma": 0.8,
  "fresh_age": 0.4,
  "stale_age": 0.8,
  "min_confidence": 0.25,
  "heading_min_speed": 0.15,
  "k_sigma": 1.0,
  "human_radius": 0.3,
  "emergency_margin": 0.05,
  "static_extra_margin": 0.03,
  "stale_guard_hold": 1.4,
  "stale_near_distance": 1.4,
  "commit_hold": 1.2,
  "cruise_speed": 0.3,
  "group_distance": 1.2,
  "group_direction_cos": 0.85,
  "group_sigma": 0.6,
  "group_weight": 0.8,
  "weights": {
    "goal": 1.0,
    "social": 4.0,
    "control": 0.12,
    "heading": 0.15,
    "switch": 0.8,
    "stop": 0.08
  }
}
```
## 5. Functional results
Status: **SOCIAL NAVIGATION FUNCTIONAL PASS**. Completed metric rows: 30 / 30. Failed development attempts remain under development. No failed functional run is excluded or selected away.
Development history: the first Frontal run exposed duplicate exposure timestamps and crashed; the second exposed a near-waypoint endpoint-heading reversal and stalled. Both are retained. The timestamp submission guard applies equally to both controllers; the new controller heading cost now uses the current waypoint bearing. After successful development at social weight 1.2, one uniform pre-benchmark change to 4.0 made social cost influence choices more often. Frontal, Perpendicular and Circular were then rerun successfully before freezing. No per-scene weight search or formal-run retuning was performed.
Five scenarios × two controllers × three paired repeat labels 17/23/31. Python/NumPy seeds are set; frozen actor loader still uses Randomizer(17), so these are repeatability runs, NOT three independent random human populations. Same routes/speeds/geometry, renderer and camera for both. All benchmark runs record third-person output so render cost is paired; Circular seed 17 social also records first person.

| Scenario / controller | n | Complete | Human / static collision runs | Min distance m | Intrusion s | Integrated social cost | Travel s | Path m | Stop s | Switches |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| frontal / baseline | 3 | 3 | 0 / 0 | 1.474 ± 0.022 | 0.000 ± 0.000 | 0.645 ± 0.011 | 29.300 ± 0.029 | 8.249 ± 0.001 | 0.000 ± 0.000 | 2.000 ± 0.000 |
| frontal / social_nav | 3 | 3 | 0 / 0 | 1.287 ± 0.008 | 0.000 ± 0.000 | 0.901 ± 0.022 | 35.211 ± 0.395 | 8.203 ± 0.001 | 3.856 ± 0.472 | 0.000 ± 0.000 |
| obstruction / baseline | 3 | 3 | 0 / 0 | 1.048 ± 0.004 | 0.000 ± 0.000 | 3.295 ± 0.019 | 36.206 ± 0.025 | 8.347 ± 0.002 | 4.278 ± 0.025 | 4.000 ± 0.000 |
| obstruction / social_nav | 3 | 3 | 0 / 0 | 1.254 ± 0.000 | 0.000 ± 0.000 | 2.669 ± 0.042 | 35.067 ± 0.073 | 8.203 ± 0.001 | 2.500 ± 0.100 | 0.000 ± 0.000 |
| blind_corner / baseline | 3 | 3 | 0 / 0 | 1.140 ± 0.000 | 0.000 ± 0.000 | 1.872 ± 0.000 | 34.217 ± 0.000 | 8.209 ± 0.000 | 5.700 ± 0.000 | 1.000 ± 0.000 |
| blind_corner / social_nav | 3 | 3 | 0 / 0 | 1.084 ± 0.023 | 0.000 ± 0.000 | 1.695 ± 0.102 | 33.772 ± 1.685 | 8.203 ± 0.001 | 2.711 ± 1.708 | 0.000 ± 0.000 |
| perpendicular / baseline | 3 | 3 | 0 / 0 | 0.957 ± 0.000 | 0.000 ± 0.000 | 1.737 ± 0.001 | 34.050 ± 0.000 | 8.313 ± 0.000 | 2.333 ± 0.000 | 8.000 ± 0.000 |
| perpendicular / social_nav | 3 | 3 | 0 / 0 | 1.103 ± 0.002 | 0.000 ± 0.000 | 1.441 ± 0.010 | 32.894 ± 0.086 | 8.203 ± 0.000 | 1.417 ± 0.029 | 0.333 ± 0.577 |
| circular / baseline | 3 | 3 | 0 / 0 | 1.261 ± 0.000 | 0.000 ± 0.000 | 3.504 ± 0.002 | 33.933 ± 0.000 | 8.213 ± 0.002 | 4.122 ± 0.010 | 3.000 ± 0.000 |
| circular / social_nav | 3 | 3 | 0 / 0 | 1.200 ± 0.006 | 0.000 ± 0.000 | 4.053 ± 0.578 | 37.700 ± 0.462 | 8.203 ± 0.001 | 6.194 ± 0.423 | 0.000 ± 0.000 |

Mean ± sample standard deviation over repeats; N/A travel means route incomplete, not a successful short travel time. Collision fields use the unchanged post-run evaluator and include terminal recording padding. Social/efficiency integration stops at first route completion or run end; censoring is explicitly retained.

## 6. Social metrics
Recomputed AFTER the run from evaluation-only human positions and backward-difference velocity, with retained heading at low speed. Nominal same front/rear/side Gaussian, without planner uncertainty inflation (ground-truth positions have no tracker covariance). Report both nominal ellipse intrusion `normalized distance < 1`, summed integrated Gaussian cost, and minimum normalized distance. These quantify a DEFINED HEURISTIC field, not measured comfort. Evaluation never copies selected internal candidate cost and never returns data to planning. Uncertainty inflation and group bridge are planner terms; nominal personal-space evaluation does not pretend to measure group comfort.

## 7. Efficiency / smoothness
Travel time, path length, stop duration, steering time, direction switches, omega variation and squared omega effort are stored in CSV. Direction switches count sign changes among commands with |omega|>0.1 rad/s, including route corrections; they are not necessarily commitment violations. A social improvement accompanied by longer travel is a trade-off, not an automatic winner.
Side commitment also has an explicit synthetic check. The final Circular development run exercised opposite-side exclusion in 11 planning updates; formal activation/violation counts remain separately recorded in the CSV. Yield-only repeats are not presented as evidence of side-switch suppression.

## 8. Scenario analysis

### frontal
- personal_space_violation_s: baseline 0.000; social 0.000.
- integrated_social_cost: baseline 0.645; social 0.901.
- min_gt_distance: baseline 1.474; social 1.287.
- path_length: baseline 8.249; social 8.203.
- Social functional runs: 3/3. These observations do not establish a human preference.

### obstruction
- personal_space_violation_s: baseline 0.000; social 0.000.
- integrated_social_cost: baseline 3.295; social 2.669.
- min_gt_distance: baseline 1.048; social 1.254.
- path_length: baseline 8.347; social 8.203.
- Social functional runs: 3/3. These observations do not establish a human preference.

### blind_corner
- personal_space_violation_s: baseline 0.000; social 0.000.
- integrated_social_cost: baseline 1.872; social 1.695.
- min_gt_distance: baseline 1.140; social 1.084.
- path_length: baseline 8.209; social 8.203.
- Social functional runs: 3/3. These observations do not establish a human preference.

### perpendicular
- personal_space_violation_s: baseline 0.000; social 0.000.
- integrated_social_cost: baseline 1.737; social 1.441.
- min_gt_distance: baseline 0.957; social 1.103.
- path_length: baseline 8.313; social 8.203.
- Social functional runs: 3/3. These observations do not establish a human preference.

### circular
- personal_space_violation_s: baseline 0.000; social 0.000.
- integrated_social_cost: baseline 3.504; social 4.053.
- min_gt_distance: baseline 1.261; social 1.200.
- path_length: baseline 8.213; social 8.203.
- Social functional runs: 3/3. These observations do not establish a human preference.

## 9. Multi-person
Circular uses the unchanged three-person preset. Maximum planned tracks and actual detector/tracker counts are logged; tracker IDs are not physical identities and false/duplicate tracks can exist.

## 10. Group awareness
Pairs within 1.2 m with cosine direction similarity >=0.85 or both below 0.15 m/s add a predicted segment/capsule Gaussian bridge. Synthetic checks validate pair and bridge scoring. Logged pair activations alone do not establish genuine social groups: no human group labels or comfort study were collected.

## 11. Performance

| Scenario / controller | YOLO responses/wall-s | RTF | Steering s | Omega variation rad/s | Min social norm | GPU peak MiB | System RAM peak GiB |
|---|---:|---:|---:|---:|---:|---:|---:|
| frontal / baseline | 6.000 ± 0.408 | 0.606 ± 0.039 | 3.833 ± 0.058 | 2.011 ± 0.003 | 1.741 ± 0.056 | 3446.667 ± 89.226 | 13.897 ± 0.306 |
| frontal / social_nav | 6.738 ± 0.725 | 0.687 ± 0.072 | 0.000 ± 0.000 | 0.000 ± 0.000 | 1.591 ± 0.014 | 3401.667 ± 90.456 | 13.971 ± 0.248 |
| obstruction / baseline | 7.051 ± 0.152 | 0.718 ± 0.020 | 5.150 ± 0.017 | 3.008 ± 0.018 | 1.309 ± 0.005 | 3476.333 ± 142.469 | 13.729 ± 0.510 |
| obstruction / social_nav | 7.053 ± 0.271 | 0.716 ± 0.030 | 0.000 ± 0.000 | 0.000 ± 0.000 | 1.216 ± 0.010 | 3412.667 ± 74.661 | 14.322 ± 0.740 |
| blind_corner / baseline | 6.603 ± 0.653 | 0.670 ± 0.062 | 1.733 ± 0.000 | 1.401 ± 0.000 | 1.370 ± 0.000 | 3392.333 ± 115.660 | 13.677 ± 0.169 |
| blind_corner / social_nav | 6.341 ± 0.363 | 0.642 ± 0.034 | 0.000 ± 0.000 | 0.000 ± 0.000 | 1.340 ± 0.007 | 3369.000 ± 23.643 | 13.708 ± 0.218 |
| perpendicular / baseline | 6.238 ± 0.076 | 0.638 ± 0.005 | 8.067 ± 0.000 | 4.391 ± 0.019 | 1.192 ± 0.000 | 3355.000 ± 14.731 | 13.741 ± 0.327 |
| perpendicular / social_nav | 5.978 ± 0.220 | 0.610 ± 0.025 | 0.356 ± 0.616 | 0.289 ± 0.501 | 1.370 ± 0.004 | 3415.333 ± 45.960 | 13.727 ± 0.112 |
| circular / baseline | 6.185 ± 0.777 | 0.630 ± 0.076 | 3.400 ± 0.000 | 2.789 ± 0.000 | 1.161 ± 0.000 | 3484.667 ± 22.234 | 14.139 ± 1.077 |
| circular / social_nav | 6.268 ± 0.773 | 0.645 ± 0.075 | 0.200 ± 0.000 | 0.566 ± 0.012 | 1.218 ± 0.046 | 3491.000 ± 29.138 | 14.123 ± 0.878 |

Planner diagnostics (not end-to-end latency):
- blind_corner seed 17: YOLO 6.71/wall-s; RTF 0.676; planner P95 2.192 ms; social argmin changes 4; group-active plans 0.
- blind_corner seed 23: YOLO 5.98/wall-s; RTF 0.608; planner P95 2.268 ms; social argmin changes 5; group-active plans 0.
- blind_corner seed 31: YOLO 6.33/wall-s; RTF 0.644; planner P95 2.772 ms; social argmin changes 4; group-active plans 0.
- circular seed 17: YOLO 5.93/wall-s; RTF 0.612; planner P95 2.415 ms; social argmin changes 4; group-active plans 9.
- circular seed 23: YOLO 5.72/wall-s; RTF 0.592; planner P95 2.418 ms; social argmin changes 1; group-active plans 17.
- circular seed 31: YOLO 7.15/wall-s; RTF 0.731; planner P95 2.177 ms; social argmin changes 1; group-active plans 19.
- frontal seed 17: YOLO 6.25/wall-s; RTF 0.639; planner P95 2.466 ms; social argmin changes 3; group-active plans 6.
- frontal seed 23: YOLO 6.40/wall-s; RTF 0.652; planner P95 2.082 ms; social argmin changes 4; group-active plans 0.
- frontal seed 31: YOLO 7.57/wall-s; RTF 0.769; planner P95 2.237 ms; social argmin changes 4; group-active plans 0.
- obstruction seed 17: YOLO 7.36/wall-s; RTF 0.751; planner P95 2.169 ms; social argmin changes 11; group-active plans 7.
- obstruction seed 23: YOLO 6.86/wall-s; RTF 0.698; planner P95 2.305 ms; social argmin changes 11; group-active plans 9.
- obstruction seed 31: YOLO 6.94/wall-s; RTF 0.700; planner P95 2.222 ms; social argmin changes 11; group-active plans 8.
- perpendicular seed 17: YOLO 6.02/wall-s; RTF 0.616; planner P95 2.390 ms; social argmin changes 7; group-active plans 28.
- perpendicular seed 23: YOLO 6.18/wall-s; RTF 0.631; planner P95 2.121 ms; social argmin changes 7; group-active plans 25.
- perpendicular seed 31: YOLO 5.74/wall-s; RTF 0.583; planner P95 2.687 ms; social argmin changes 7; group-active plans 28.

RTF<1 is not wall-clock real time. GPU/RAM scope follows existing resource monitor; no resolution reductions used.

## 12. GT isolation
`social_controller.py` imports only NumPy and time. Input: estimated tracks/covariance/confidence, self pose/current velocity, waypoints, frozen static polygons. Actor GT scripting and post-run evaluation are outside this module. No scenario label or actor route is supplied. The robot self pose is idealized simulation localization, not human GT. Unit checks verify frozen algorithm modules unchanged.

## 13. Videos
Seed 17 was fixed for delivery, not selected after results. Each video is a single continuous run; no cross-run splicing.
- [circular / social_nav / seed 17 third person](D:/detection/robot_human_isaac6/outputs/social_navigation/benchmark/circular/social_nav/seed_17/run_01/NAVWARESET_CIRCULAR.mp4)
- [circular / social_nav / seed 17 robot camera](D:/detection/robot_human_isaac6/outputs/social_navigation/benchmark/circular/social_nav/seed_17/run_01/first_person/A300_THREE_PEOPLE_FIRST_PERSON.mp4)
- [frontal / baseline / seed 17 third person](D:/detection/robot_human_isaac6/outputs/social_navigation/benchmark/frontal/baseline/seed_17/run_01/NAVWARESET_FRONTAL.mp4)
- [frontal / social_nav / seed 17 third person](D:/detection/robot_human_isaac6/outputs/social_navigation/benchmark/frontal/social_nav/seed_17/run_01/NAVWARESET_FRONTAL.mp4)
- [perpendicular / baseline / seed 17 third person](D:/detection/robot_human_isaac6/outputs/social_navigation/benchmark/perpendicular/baseline/seed_17/run_01/NAVWARESET_PERPENDICULAR.mp4)
- [perpendicular / social_nav / seed 17 third person](D:/detection/robot_human_isaac6/outputs/social_navigation/benchmark/perpendicular/social_nav/seed_17/run_01/NAVWARESET_PERPENDICULAR.mp4)

Estimated-state diagnostic: [SOCIAL_COST_TOPDOWN.png](SOCIAL_COST_TOPDOWN.png), with editable vector [SVG](SOCIAL_COST_TOPDOWN.svg). The snapshot uses the first multi-track non-CRUISE/non-STOP decision of the fixed Circular seed 17 run. It shows current uncertainty-inflated contours and 3 s human predictions; it is not a GT trajectory plot or a success-selection metric.

## 14. Limitations
Heuristic personal space; constant-velocity prediction; no intent model; no RL/MPC; no human-subject comfort evaluation; kinematic A300; simulation only; no safety certification. Finite discrete horizon and seven primitives can miss viable paths. Erroneous/duplicate tracks can cause conservative stops. Same RNG labels do not eliminate asynchronous runtime timing variation. Reduced social cost does not imply greater safety.

## 15. Final status
**SOCIAL NAVIGATION FUNCTIONAL PASS**
Integrated nominal personal-space cost is lower in: obstruction, blind_corner, perpendicular; higher or equal in: frontal, circular. This is not uniform dominance over DemoController. Functional PASS means the specified implementation and simulation gates passed, not that the new policy should replace the baseline or that comfort/safety is certified.
All 30 runs have zero nominal personal-space ellipse intrusion; therefore this thresholded metric does not demonstrate an intrusion reduction. The continuous Gaussian cost still varies.
Regression: `{"passed": true, "scope": "Short startup/perception/legacy-controller smoke; formal runs cover full NavWareSet behavior", "runs": [{"scene": "indoor", "passed": true, "source_run": "D:\\detection\\robot_human_isaac6\\outputs\\stage7_social_navigation_regression_1789511609136759000", "rgb_frames": 50, "yolo_samples": 48}, {"scene": "navwareset_frontal", "passed": true, "source_run": "D:\\detection\\robot_human_isaac6\\outputs\\social_navigation\\regression\\frontal\\baseline\\seed_17\\run_01", "rgb_frames": 50, "yolo_samples": 48}]}`.
Explicit gates: `{"thirty_runs": true, "all_social_functional": true, "circular_three_people": true, "social_cost_changes_choice": true, "prediction_and_isolation_checks": true, "side_hold_exercised": true, "no_side_hold_violations": true, "baseline_regression": true, "videos_validated": true}`.
Numerical outcomes and source paths are in SOCIAL_NAVIGATION_RESULTS.csv. No rejected/development runs deleted. Report status is contingent on all required functional checks, not solely lower social cost.
