# Third-person collision-prediction robustness protocol

Frozen prediction baseline: `5e6231285135e4cc6cf5c7c72eb7561a0f6f9f40` on `feature/third-person-collision-prediction`; working tree was clean before adding the experimental harness. The existing nine `seed_17` optimized runs are reused as baseline evidence. No predictor threshold or model parameter is tuned during this evaluation.

Source SHA-256 at the freeze point:

| File | SHA-256 |
| --- | --- |
| `third_person_collision_prediction.py` | `55899D32A87F9BB9E27D5AEA6259D08253904CD20F15B9632B5C65EEFBE066E0` |
| `tracker.py` | `0E631F593C3F249F2D72A7DB3CDBC96C7904B434FFF71598B563AE50704D07BD` |
| `config.yaml` | `AED02EA8DAEAE879752450EF17B6F52E273E4CA2A3910C69CBC04057B02C9F96` |
| `yolo_worker.py` | `516CC2FC8B754BD2E17545D240443A56E0210C4BDA03D9C8E8BB404EBB9E3F16` |

Detector: official COCO `yolo26n.pt`, Ultralytics 8.4.144, person class 0, confidence 0.25, image size 640. Fixed camera: 640×360, 10 Hz, 18 mm focal length, 24×13.5 mm aperture, eye `(5,-5,7)`, target `(5,0,0)`. KF: measurement sigma 0.15 m, acceleration sigma 1.0 m/s², initial velocity sigma 1.0 m/s, association gate 0.8 m, innovation gate χ² 9.21034, max missed 5. Robot speed 0.30 m/s. Collision radius 0.9027795979748836 m; warning radius 1.1027795979748836 m. CPA and rollout horizon 5 s, rollout step 0.1 s, two consecutive risk exposures for system-level confirmation.

Phase A: all nine original scenarios at simulator seeds 17, 23, 31. Phase B: three hard near-misses at seed 17. Phase C: four specified scenarios at seed 17, detector-output dropout rates 0, 0.1, 0.2, 0.3 and independent dropout seed 101. A zero-dropout run may reuse a matching run from another phase.

Hard near-miss delays were selected **once before predictor runs** using only the prior scene's GT trajectories and approximate temporal alignment, retaining spatial path and speed:

| Scenario | Original delay | Frozen hard delay | Change |
| --- | ---: | ---: | ---: |
| perpendicular | 6.0 s | 6.8 s | +0.8 s |
| diagonal | 4.0 s | 7.7 s | +3.7 s |
| cut-in | 4.0 s | 6.0 s | +2.0 s |

The latter two shifts are larger than the example 0.3–0.8 s range because the original trajectories were farther apart (2.33 m and 1.48 m minimum separation); pretending a smaller shift made them hard would be misleading. If a frozen hard case actually collides or is not within 1.0–1.3 m, retain and report that failure rather than adjusting it after seeing prediction outputs.

Dropout is inserted after the YOLO worker's exposure-aligned response and before depth projection/tracking. The raw boxes are logged, so injected missing detections can be distinguished from YOLO misses. No GT enters inference.
