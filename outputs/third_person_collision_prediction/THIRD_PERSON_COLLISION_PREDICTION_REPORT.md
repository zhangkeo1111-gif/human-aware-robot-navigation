# Third-person pedestrian collision prediction

## 1. Goal
Test whether a fixed third-person RGB-D camera can predict robot–human collision proxies before they occur.

## 2. System
Third-person RGB-D → official COCO YOLO11n person → torso robust depth → world XY → Hungarian + CV-KF → CPA/TTC and 5 s rollout. Robot ego state is known; human GT and scripted future routes are post-run evaluation only.

## 3. Camera Setup
One fixed `/World/CollisionPredictionCamera`, 640×360, 10 Hz simulation time; eye (5,−5,7) m, target (5,0,0) m; 18 mm focal length, 24 mm horizontal aperture (HFOV ≈ 67.38°), 13.5 mm vertical aperture. Same settings across runs. Exact camera intrinsics and view transform are in each `camera_params.json`.

## 4. Collision Scenarios
8 completed single-person presets plus 1 multi-person preset, seed 17; robot open-loop straight at 0.30 m/s, human prescribed paths. Collision proxy threshold is 0.903 m (0.603 m conservative robot radius + 0.30 m human radius). No physical-contact certification.

## 5. Detection Results
Visibility GT was not independently labeled; therefore true visible-person recall is **not reported**. Single-person exposure detection continuity ranges from 45.3% to 93.4%; this includes time before a delayed person enters view and time after leaving. First detection and missed-exposure ratios are in the CSV.

## 6. Tracking Results
Post-run root-position and finite-difference velocity errors are in the CSV, computed only on active tracks that can be associated offline; misses are not imputed into MAE. KF estimates are evaluated, not fed Human GT. Identity continuity is imperfect: single-run ID-switch counts range from 0 to 22. In particular, cut-in and rear-end runs show track fragmentation; this experiment does not establish stable long-term identity.

## 7. Collision Prediction Results
| Scenario | Actual proxy | Predicted | First-alert lead (s) | 3-frame sustained lead (s) | Position MAE (m) | Velocity MAE (m/s) |
|---|---:|---:|---:|---:|---:|---:|
| headon_collision | True | True | 4.77 | 4.57 | 0.17 | 0.08 |
| perpendicular_collision | True | True | 1.00 | 1.00 | 0.13 | 0.07 |
| diagonal_collision | True | True | 1.55 | 1.55 | 0.13 | 0.08 |
| cutin_collision | True | True | 3.40 | 2.70 | 0.18 | 0.10 |
| rear_end_collision | True | True | 8.70 | 2.20 | 0.17 | 0.03 |
| perpendicular_nearmiss | False | False | — | — | 0.13 | 0.07 |
| diagonal_nearmiss | False | False | — | — | 0.14 | 0.06 |
| cutin_nearmiss | False | False | — | — | 0.17 | 0.09 |
| multi_person_collision | True | True | 4.97 | 4.97 | 0.15 | 0.06 |

Observed mechanism counts: TP=5, FP=0, FN=0, TN=3. These few prescribed runs are **not** a statistical safety benchmark.
Positive lead times: 4.77, 1.00, 1.55, 3.40, 8.70 s; first-alert median = 3.40 s; 3-frame sustained median = 2.20 s.
Single-person missed or late collision cases: none. Single-person run-level false-alarm cases: none; the separate three-person run has one target-level false-alarm frame (Section 9).

The 3-frame sustained-alarm onset is a **post-hoc diagnostic** added after observing an isolated early alarm; it is not an online gate or a pre-registered success threshold. First-alert lead can overstate useful warning, especially during KF initialization. `first_ttc_error_s` in the CSV compares first predicted TTC with the actual remaining time.

## 8. Near-miss / False Alarms
A near-miss is judged by measured post-run minimum separation, not by its scenario name. See `actual_collision` and `false_alarm` in the CSV.

## 9. Multi-person Risk Ranking
The three-person run has per-person offline GT association and observed risk levels in `multi_person_collision/seed_17/MULTI_PERSON_RISK_RANKING.json`; online predictions never receive this association.

| Evaluation person | Actual proxy | Maximum online risk | Collision-risk frames | Associated frames |
|---|---:|---|---:|---:|
| Person 1 | True | COLLISION_RISK | 59 | 305 |
| Person 2 | False | SAFE | 0 | 349 |
| Person 3 | False | COLLISION_RISK | 1 | 288 |

Target-level false alarm: Person 3 had at least one collision-risk frame without an actual proxy. The multi-person risk ranking is therefore **not fully correct**; track IDs also fragment and offline GT association is used only for this audit.

## 10. Videos / Figures
Every completed scenario has the original `THIRD_PERSON_COLLISION_PREDICTION.mp4`, a same-run exposure-aligned `THIRD_PERSON_COLLISION_PREDICTION_HUD.mp4` with d_CPA, and `TOP_DOWN_EVIDENCE.png`. Video HUD uses stored camera-based estimates only; top-down Human GT is explicitly evaluation-only.

## 11. Limitations
Simulation only; fixed third-person camera; CV extrapolation; prescribed trajectories; small scenario count; kinematic robot; conservative geometric collision proxy rather than physical-contact certification; no learned intent prediction. Exposure continuity is not visible-person recall.

## 12. Conclusion
YOLO detected a person in 8/8 single-person runs. Tracking position/velocity and warning lead times are reported per run above. Among single-person cases, 0 actual proxy run(s) lacked a positive prediction and 0 non-collision run(s) had a collision-risk false alarm. Do not promote a general safety conclusion from these mechanism-level cases.
