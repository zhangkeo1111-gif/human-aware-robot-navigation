# YOLO26n third-person collision prediction A/B

## 1. Detector Change
Only the official COCO detection checkpoint changed: YOLO11n → YOLO26n. Both resolve COCO person class 0. YOLO26n uses the installed Ultralytics 8.4.144 on CUDA.

## 2. Frozen System
Same fixed camera, 640×360 at 10 Hz, conf 0.25, imgsz 640, depth, Hungarian/CV-KF, CPA/TTC, 5 s rollout, thresholds, robot motion, scenarios and seed 17. The comparison script checked camera/robot/scenario parameters and 349 exposures for every pair. YOLO11n files were read-only. Visibility GT was not labeled, so exposure continuity is not visible-person recall.

## 3. Detection Comparison
| Scene | Continuity 11n / 26n | First detection 11n / 26n (s) | Mean confidence 11n / 26n |
|---|---:|---:|---:|
| headon_collision | 0.840 / 0.917 | 0.12 / 0.12 | 0.720 / 0.609 |
| perpendicular_collision | 0.653 / 0.668 | 11.32 / 11.32 | 0.832 / 0.806 |
| diagonal_collision | 0.691 / 0.716 | 0.32 / 1.62 | 0.764 / 0.732 |
| cutin_collision | 0.481 / 0.479 | 0.12 / 0.12 | 0.638 / 0.590 |
| rear_end_collision | 0.774 / 0.799 | 0.12 / 0.12 | 0.614 / 0.553 |
| perpendicular_nearmiss | 0.817 / 0.814 | 6.22 / 6.12 | 0.857 / 0.827 |
| diagonal_nearmiss | 0.934 / 0.905 | 0.32 / 0.12 | 0.818 / 0.793 |
| cutin_nearmiss | 0.453 / 0.499 | 0.12 / 0.12 | 0.721 / 0.644 |
| multi_person_collision | 1.000 / 1.000 | 0.12 / 0.12 | 0.785 / 0.751 |

Continuity improved in 5/9 scenes. Missed-exposure ratios, median confidence and high-IoU (>0.8) duplicate-box-pair proxies are in the CSV; a high-IoU pair is not automatically a GT-confirmed duplicate.

## 4. Tracking Comparison
| Scene | Position MAE 11n / 26n (m) | P95 11n / 26n (m) | Velocity MAE 11n / 26n (m/s) | ID switches 11n / 26n |
|---|---:|---:|---:|---:|
| headon_collision | 0.173 / 0.173 | 0.203 / 0.207 | 0.081 / 0.074 | 11 / 7 |
| perpendicular_collision | 0.127 / 0.126 | 0.160 / 0.160 | 0.073 / 0.075 | 1 / 0 |
| diagonal_collision | 0.132 / 0.139 | 0.171 / 0.200 | 0.084 / 0.088 | 6 / 8 |
| cutin_collision | 0.182 / 0.173 | 0.216 / 0.220 | 0.097 / 0.102 | 15 / 10 |
| rear_end_collision | 0.167 / 0.185 | 0.211 / 0.215 | 0.027 / 0.030 | 22 / 46 |
| perpendicular_nearmiss | 0.128 / 0.128 | 0.167 / 0.167 | 0.073 / 0.067 | 0 / 0 |
| diagonal_nearmiss | 0.142 / 0.146 | 0.180 / 0.184 | 0.064 / 0.070 | 3 / 3 |
| cutin_nearmiss | 0.165 / 0.162 | 0.212 / 0.215 | 0.089 / 0.088 | 3 / 3 |
| multi_person_collision | 0.153 / 0.157 | 0.212 / 0.212 | 0.063 / 0.067 | 21 / 7 |

ID-switch count fell in 4/9 scenes; total switches: 82 → 84. GT-associated unique-ID fragmentation, track continuity and duplicate-track frames are in the CSV; these offline associations never enter online inference. Cut-in fragmentation: 6 → 6; rear-end: 9 → 11.

## 5. Collision Prediction Comparison
| Scene | Actual | Predicted 11n / 26n | First lead 11n / 26n (s) | Sustained lead 11n / 26n (s) |
|---|---:|---:|---:|---:|
| headon_collision | True | True / True | 4.77 / 4.77 | 4.57 / 4.77 |
| perpendicular_collision | True | True / True | 1.00 / 2.10 | 1.00 / 1.30 |
| diagonal_collision | True | True / True | 1.55 / 2.15 | 1.55 / 1.65 |
| cutin_collision | True | True / True | 3.40 / 3.70 | 2.70 / 3.70 |
| rear_end_collision | True | True / True | 8.70 / 5.87 | 2.20 / 4.07 |
| perpendicular_nearmiss | False | False / True | — / — | — / — |
| diagonal_nearmiss | False | False / False | — / — | — / — |
| cutin_nearmiss | False | False / False | — / — | — / — |
| multi_person_collision | True | True / True | 4.97 / 4.87 | 4.97 / 4.87 |
YOLO11n: single-person TP=5, FN=0, FP=0, TN=3; positive sustained-lead median=2.20 s.
YOLO26n: single-person TP=5, FN=0, FP=1, TN=2; positive sustained-lead median=3.70 s.

YOLO26n perpendicular near-miss false alarm lasted 1 exposure frame(s). The online run-level decision is still a false positive; the post-hoc sustained-alarm diagnostic does not erase it.

## 6. Multi-person Result
| Person | Actual proxy | Collision-risk frames 11n / 26n | Associated frames 11n / 26n |
|---|---:|---:|---:|
| Person 1 | True | 59 / 67 | 305 / 314 |
| Person 2 | False | 0 / 0 | 349 / 349 |
| Person 3 | False | 1 / 0 | 288 / 288 |

Per-person identity here comes from post-run GT association only. Online risk uses camera-derived tracks, not GT.

## 7. Runtime
| Metric (median across nine runs) | YOLO11n | YOLO26n |
|---|---:|---:|
| YOLO response mean (ms) | 18.84 | 21.94 |
| YOLO response P95 (ms) | 27.78 | 25.52 |
| YOLO responses / wall-s | 5.10 | 5.21 |

Responses/wall-s includes Isaac rendering and synchronous pipeline work; it is not detector-only FPS.

## 8. Conclusion
1. Detection continuity: improved in 5/9 scenes; median 0.77 → 0.80.
2. ID switches / fragmentation: switches fell in 4/9 scenes and totaled 82 → 84; unique-ID fragmentation totaled 43 → 34.
3. Collision prediction: both detectors warned in all five positive single-person runs; YOLO26n introduced 1 false-alarm frame(s) in one near-miss run while Person 3 collision-risk frames fell from 1 to 0. This is mixed, not uniformly more stable.
This single-seed, nine-scene mechanism test does not establish general detector superiority.