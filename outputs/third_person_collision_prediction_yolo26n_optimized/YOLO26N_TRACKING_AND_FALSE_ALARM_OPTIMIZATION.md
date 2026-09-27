# YOLO26n tracking and false-alarm optimization

Seed 17; one new continuous Isaac Sim 6.0.1 run per scenario, 349 camera exposures each. Frozen YOLO26n recordings and all original videos/reports remain in `../third_person_collision_prediction_yolo26n/`. New raw runs/videos are local-only under this directory. [Per-exposure comparison](YOLO26N_BEFORE_AFTER_OPTIMIZATION.csv), [rear-end audit](REAR_END_TRACK_FRAGMENTATION_AUDIT.md), and [near-miss audit](PERP_NEARMISS_FALSE_ALARM_AUDIT.md) provide the detailed evidence. This is a single-seed mechanism check, not a statistical safety benchmark.

## 1. Problems

- Rear-end: frozen YOLO26n run had 46 evaluator ID switches and 11 unique-ID fragments versus 22 and 9 for frozen YOLO11n.
- Perpendicular near-miss: no actual collision proxy, but one raw `COLLISION_RISK` exposure caused a run-level false positive.

## 2. Root-Cause Audit

Rear-end exposure replay shows two mechanisms. Long box-free intervals cause normal track death after five misses. Later, overlapping same-person boxes with close world XY create duplicate tracks; the single visible box can alternate between them, inflating the evaluator's nearest-track ID-switch count. YOLO26n had 19 exposures with bbox IoU ≥0.50 and 24 exposures with nearby active tracks (<0.15 m), versus 16 and 19 for YOLO11n. Isolated extra boxes occasionally project to background depth ≈9.1 m instead of the person's ≈7.3 m and correctly fail the existing 0.8 m gate. There was no rear-end innovation-gate rejection in replay. Neither KF Q/R nor the association gate was changed.

The near-miss alarm occurs at exposure 63, exactly when a new KF track first becomes risk-eligible (age 3). Its estimated X velocity then changes sign as measurements accumulate; torso depth is stable over the relevant exposures and the track ID does not switch. Primary diagnosis: `VELOCITY_TRANSIENT` during `KF_INITIALIZATION`, not a depth jump or association error.

## 3. Minimal Changes

The original detector boxes/observations and an untouched parallel `raw_tracks` KF stream remain logged. Only the collision-prediction track input removes a same-exposure duplicate when bbox IoU ≥0.50, world-XY separation ≤0.15 m, and torso-depth difference ≤0.35 m; it keeps the higher-confidence measurement. Distinct people and dissimilar-depth boxes remain separate. `consolidated_tracks` is explicitly logged beside `raw_tracks`.

The original CPA/TTC and 5 s rollout still produce `raw_risk`. A separate causal `confirmed_risk` becomes `CONFIRMED_COLLISION_RISK` only after two consecutive exposure frames with raw collision risk. Raw and confirmed first-alert times are both saved. Collision radius ≈0.903 m, +0.20 m warning margin, YOLO26n, camera, depth, KF Q/R, 0.8 m gate, five-miss lifetime, scene timing, and robot motion are unchanged. No GT enters prediction.

## 4. Rear-end Before / After

| Measure | Frozen YOLO26n before | Same new run: raw KF | Same new run: prediction KF |
|---|---:|---:|---:|
| Evaluator ID switches | 46 | 45 | **31** |
| GT-associated unique-ID fragments | 11 | 12 | **7** |
| Duplicate active-track frames | 32 | 33 | **17** |
| Position MAE (m) | 0.185 | 0.189 (same-run raw) | 0.190 |

The same-run raw-versus-prediction comparison isolates the tracker-interface change from small Isaac/rendering variation. Sixteen duplicate measurements were suppressed in the new run. The collision remains predicted: raw lead 3.87 s, confirmed lead 3.77 s. The frozen run's first raw lead was 5.87 s, so the optimized recording has a **2.10 s shorter first-alert lead** relative to that separate run; only 0.10 s of the new run's delay is due to confirmation. Its earlier frozen alert was isolated, and the frozen three-frame sustained lead was 4.07 s. Do not describe the historical lead change as solely a confirmation cost. Remaining 31 switches/7 fragments mean identity continuity is improved, not solved.

## 5. Near-miss Before / After

Perpendicular near-miss still has exactly one raw collision-risk frame (exposure 63), so the original false-alarm evidence is visible. It has **zero confirmed collision-risk frames** and `actual_collision=false`. No duplicate measurement was suppressed in this case: the improvement comes solely from causal confirmation, not altered detector/depth/KF geometry.

## 6. Nine-Scenario Regression

| Scenario | Actual proxy | Raw risk | Confirmed risk | Confirmed lead (s) | ID switches before→after |
|---|---:|---:|---:|---:|---:|
| headon_collision | yes | yes | yes | 4.77 | 7→5 |
| perpendicular_collision | yes | yes | yes | 1.10 | 0→0 |
| diagonal_collision | yes | yes | yes | 1.45 | 8→4 |
| cutin_collision | yes | yes | yes | 3.50 | 10→5 |
| rear_end_collision | yes | yes | yes | 3.77 | 46→31 |
| perpendicular_nearmiss | no | **yes: 1 frame** | **no** | — | 0→0 |
| diagonal_nearmiss | no | no | no | — | 3→2 |
| cutin_nearmiss | no | no | no | — | 3→2 |
| multi_person_collision | yes | yes | yes | 4.77 | 7→7 |

For the five single-person positive runs, confirmed TP=5, FN=0; among three single-person negatives, confirmed FP=0, TN=3. Raw risk still has one FP run. Across all nine, evaluator ID switches decrease 84→56 and unique-ID fragments 34→22 (different-run comparison). No original output was overwritten.

## 7. Lead-Time Trade-off

| Collision case | New-run raw lead (s) | New-run confirmed lead (s) | Confirmation delay (s) | Frozen raw lead (s) |
|---|---:|---:|---:|---:|
| Head-on | 4.87 | 4.77 | 0.10 | 4.77 |
| Perpendicular | 2.10 | 1.10 | **1.00** | 2.10 |
| Diagonal | 2.15 | 1.45 | 0.70 | 2.15 |
| Cut-in | 4.40 | 3.50 | 0.90 | 3.70 |
| Rear-end | 3.87 | 3.77 | 0.10 | 5.87 |

Median new-run confirmed lead is 3.50 s (five positives); median raw-to-confirmed delay is 0.70 s. A two-frame condition does **not** guarantee only 0.10 s loss from the *first* raw alert when that alert is isolated and a later pair supplies the first confirmation. No true collision was lost here, but the 1.00 s worst observed confirmation delay matters for downstream response time.

## 8. Multi-person Check

Offline same-exposure GT association is used **only for evaluation**. In the optimized run, Person 1 (actual collision proxy) has 66 raw risk frames, 63 of which coincide with confirmed system risk. Person 2 and Person 3 have 0 raw/confirmed risk frames each and actual minimum separation 2.315 m / 1.489 m respectively. The frozen YOLO26n run had Person 1/2/3 raw risk frames 67/0/0. This single run shows no obvious deterioration; it does not prove general multi-person identity stability.

## 9. Runtime

Medians across nine runs, using per-exposure means except where P95 is stated:

| Stage | Frozen YOLO26n | Optimized YOLO26n |
|---|---:|---:|
| YOLO model inference | 21.94 ms | 24.64 ms |
| Worker round-trip response | not recorded | 25.49 ms |
| Raw + prediction association/KF | not recorded | 0.237 ms |
| Risk prediction + confirmation | not recorded | 0.052 ms |
| Exposure processing, incl. worker/overlay | not recorded | 28.00 ms mean; 31.37 ms P95 |

The inference-time difference is a separate-run observation, not evidence that duplicate filtering slows YOLO; the checkpoint and settings did not change. Tracking/risk overhead is small relative to worker response. Isaac rendering and simulation throughput are excluded from these per-exposure processing timings; they must not be called detector FPS.

## 10. Remaining Limitations

Early detector gaps still retire tracks; low-confidence off-person/background-depth boxes still exist; rear-end still has 31 evaluator switches and 7 unique-ID fragments. The evaluator can alternate between nearby tracks and therefore over-count physical identity loss. Position MAE is not systematically better (rear-end 0.185→0.190 m across separate runs). Two-frame confirmation can delay useful warning by up to 1.00 s in these cases. The filter has only one seed and nine prescribed scenes; no safety or broad detector-generalization claim follows.
Confirmation here is a system-level consecutive-exposure gate, not a proof that the **same** person produced both risk frames; the three-person run has no Person 2/3 raw risk, but alternating-person alerts remain an untested edge case.

## 11. Conclusion

1. **Rear-end fragmentation improved:** 46→31 evaluator switches and 11→7 fragments versus the frozen run; same-run raw KF comparison is 45→31 and 12→7.
2. **Confirmed false positives decreased:** perpendicular near-miss raw FP is retained, while its confirmed FP disappears; all three near-misses are confirmed negative.
3. **Collision recall is retained in this regression:** 5/5 positive single-person runs have confirmed advance alerts; multi-person Person 1 remains risky, Person 2/3 stay safe.
4. **Lead-time cost is explicit:** 0.10–1.00 s raw-to-confirmed first-alert delay within the new runs; the rear-end frozen-to-new raw lead is 2.10 s shorter and cannot be attributed solely to confirmation.
5. **Unresolved:** remaining detector gaps, extra-background boxes, fragmented long-term IDs, and sensitivity of first-alert time to transient early risks.
