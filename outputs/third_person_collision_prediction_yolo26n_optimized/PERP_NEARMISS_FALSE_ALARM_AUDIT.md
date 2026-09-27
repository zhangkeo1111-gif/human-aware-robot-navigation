# Perpendicular near-miss false-alarm audit (frozen YOLO26n recording)

Source: seed 17 `perpendicular_nearmiss`, frame 63 (exposure 6.283 s). Post-run minimum actual separation is 1.427 m, above the frozen ≈0.903 m collision proxy. The original online risk nevertheless reports `COLLISION_RISK` in **one** exposure only. Frames 58–68 (±5) were inspected; raw recordings remain unchanged.

| Exposure frame | Box/confidence | Torso depth (m) | World XY (m) | Active KF `[x,y,vx,vy]` | Rollout min (m) | Raw risk |
|---:|---|---:|---|---|---:|---|
| 61 | 1 / 0.467 | 5.895 | (4.288, −2.373) | (4.288, −2.373, 0, 0), age 1 | — | not eligible |
| 62 | 1 / 0.799 | 6.142 | (4.194, −2.180) | (4.232, −2.259, −0.171, 0.353), age 2 | — | not eligible |
| **63** | **1 / 0.399** | **6.124** | **(4.247, −2.157)** | **(4.233, −2.186, −0.095, 0.511), age 3** | **0.593** | **COLLISION_RISK** |
| 64 | 0 | — | — | no fresh track | — | no risk |
| 65 | 1 / 0.494 | 6.146 | (4.334, −2.136) | (4.300, −2.121, +0.154, 0.401), age 5 | 1.713 | SAFE |
| 66 | 1 / 0.733 | 6.174 | (4.357, −2.113) | (4.338, −2.099, +0.212, 0.357), age 6 | 2.030 | SAFE |

At the alarm frame, `t_CPA=4.949 s`, `d_CPA=0.592 m`, and the independent 5 s rollout minimum is 0.593 m. The KF velocity is based on just three detections; the estimated X velocity changes sign from −0.095 m/s at the alarm to +0.154 m/s two exposures later. The one missing detection at frame 64 breaks the alarm sequence. The bbox is growing as the person enters view, but torso depth at frames 62–66 stays around 6.12–6.17 m, and the same track ID survives; there is no evidence of a major depth jump or association switch at the alarm. The KF covariance at frame 63 includes velocity variance ≈0.541 (m/s)² on X.

**Primary classification: `VELOCITY_TRANSIENT` during `KF_INITIALIZATION`.** A newly eligible age-3 velocity briefly extrapolates into the robot's 5 s path. This is a causal, single-exposure model transient, not an actual collision. Changing CPA/TTC, collision radius, depth, or KF Q/R would not be supported. Preserve the raw risk and add a separate online two-consecutive-exposure `confirmed_risk`, with both first-alert times reported. This may delay true alerts by about one 10 Hz exposure and must be measured.
