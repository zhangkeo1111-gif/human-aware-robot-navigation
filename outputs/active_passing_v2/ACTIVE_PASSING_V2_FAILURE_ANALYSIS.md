# Active Passing V2 Failure Analysis

## 1. Short answer

The robot did not complete the pass because the planner lost all feasible same-side candidates before it reached the parallel-passing phase. Each candidate had to satisfy two hard conditions at the same time:

1. the robot body had to remain outside the estimated person-clearance boundary; and
2. the estimated person had to remain inside the runtime camera hard field of view for the allowed duration.

When both conditions became impossible for the remaining right-side candidates, the controller deliberately entered `EMERGENCY_STOP`. This was a safety stop, not a simulator crash or a missing video frame.

## 2. What actually happened

The static mechanism gate used eight real continuous runs with the same frozen classic static-obstruction setup. Every run reached `OFFSET` at least once, but none reached `PARALLEL`, `RECOVER` or `PASS_DONE`.

| Run | Final route s (m) | Final lateral d (m) | Max abs(d) (m) | Replans | Final phase | Route complete | Collision proxy |
|---|---:|---:|---:|---:|---|---:|---:|
| run_01 | 0.016 | 0.000 | 0.000 | 3 | EMERGENCY_STOP | No | No |
| run_02 | 0.870 | -0.202 | 0.202 | 3 | EMERGENCY_STOP | No | No |
| run_03 | 1.228 | -0.295 | 0.295 | 3 | EMERGENCY_STOP | No | No |
| run_04 | 1.711 | -0.584 | 0.584 | 3 | EMERGENCY_STOP | No | No |
| run_05 | 1.687 | -0.582 | 0.582 | 3 | EMERGENCY_STOP | No | No |
| run_06 | 1.772 | -0.592 | 0.592 | 3 | EMERGENCY_STOP | No | No |
| run_07 | 1.823 | -0.597 | 0.597 | 3 | EMERGENCY_STOP | No | No |
| run_08 | 2.481 | -0.644 | 0.644 | 3 | EMERGENCY_STOP | No | No |

The latest run is the clearest example. It selected the right side, progressed to route-local `s=2.453 m` before the final replan, and then stopped with `SAME_SIDE_REPLAN_INVALID`. The robot never reached the selected candidate’s parallel-start position, so entering `PARALLEL` would not have been safe.

## 3. Candidate-level evidence from run 08

The final replan at simulation time 12.75 s evaluated nine right-side candidates: offsets 0.65, 0.80 and 0.95 m with transition lengths 2.0, 2.5 and 3.0 m.

- valid candidates: **0/9**
- human-clearance invalid: **9/9**
- predicted hard-FOV invalid: **9/9**
- static-map invalid: **0/9**

Representative diagnostics from that replan:

| Offset | Minimum estimated clearance (m) | Longest predicted hard-FOV loss (s) | Maximum bearing (deg) |
|---:|---:|---:|---:|
| 0.65 m | about -0.24 | 2.7 | about 100.5 |
| 0.80 m | about -0.14 to -0.19 | 3.0–3.4 | about 104 |
| 0.95 m | about -0.03 to -0.14 | 3.3–3.9 | about 107–108 |

The maximum permitted offset was 0.95 m. Increasing the transition length reduced curvature but did not remove the clearance or visibility failures. The static geometry check itself passed; the rejection came from the estimated human interaction geometry and camera visibility.

## 4. Why the state machine stopped

The implemented state machine is intentionally persistent and conservative:

`NAVIGATE → OFFSET → PARALLEL → RECOVER → PASS_DONE`

During `OFFSET`, it repeatedly rechecks the selected path. If the path becomes hard-invalid, it can perform at most three same-side replans. A replan cannot switch to a completely new side or relax the hard safety gates. Once the replan budget is exhausted, the controller commands zero velocity and latches `EMERGENCY_STOP`.

Therefore, the observed behavior is:

1. choose a geometrically valid initial offset;
2. begin moving laterally;
3. approach the person while the predicted path becomes less feasible;
4. retry the same side with the remaining candidates;
5. find no candidate satisfying both hard constraints;
6. stop before parallel passing rather than entering an unsafe pass.

## 5. Role of tracking uncertainty

The latest causal track estimate at the final stop was fresh (age about 0.15 s) and located near `[4.01, -0.04]` m with a small instantaneous velocity estimate. V2 already applies the covariance-based uncertainty rule: a velocity not separated from zero by the 1.5-sigma criterion is held stationary for planner prediction only. The raw tracker state is not modified.

This reduces the risk of projecting a static person through the scene solely because of depth jitter, but it cannot solve a geometric infeasibility. Even with a stationary short-horizon prediction, the maximum 0.95 m lateral offset was marginal or insufficient under the configured robot-plus-human clearance, and the future path also lost the hard camera FOV for too long.

Thus the dominant failure is not simply “KF velocity noise”; it is the combination of limited lateral clearance and visibility constraints near the interaction point.

## 6. What was ruled out

- **Infrastructure failure:** all eight simulations completed and wrote valid summaries, detections, planner logs and videos.
- **Static-map collision:** static-invalid candidates were zero in the final run; the robot also had no static collision.
- **Physical collision:** all eight runs had `collision_proxy = false`.
- **Video problem:** first-person recordings were generated from the robot camera and passed H.264/1280×720/10-fps checks.
- **GT control leakage:** GT was not passed to `ActivePassingControllerV2`; GT is used only by the post-run evaluator and the explicitly labelled diagnostic plot.
- **Detector or camera crash:** the YOLO and camera streams remained active during the stopped runs.

## 7. What this result means

The current V2 implementation can produce a bounded lateral offset and stop safely, but it has **not demonstrated a complete active pass**. The partial lateral motion must not be reported as successful avoidance.

The honest result is therefore:

- safety stop: demonstrated;
- collision-free partial offset: demonstrated;
- complete pass and recovery: not demonstrated;
- static mechanism gate: **FAIL (0/8; required two consecutive successes)**.

## 8. Future work, not performed in this experiment

The following changes would require a new, explicitly approved experiment and were not applied here:

1. a controlled geometry-feasibility test with a stable estimated track;
2. an explicit planner log of predicted human velocity, clearance and bearing at every replan;
3. a camera-aware path that preserves visibility while reaching the parallel segment;
4. a wider or differently parameterized lateral candidate set, if the route width and safety policy allow it;
5. a separately justified replan/side-switch policy.

No parameter was relaxed after the failed gate to manufacture a pass, and no Head-on, Overtaking or formal benchmark was started.

## 9. Evidence

- Main report: [ACTIVE_PASSING_V2_REPORT.md](ACTIVE_PASSING_V2_REPORT.md)
- Numeric results: [ACTIVE_PASSING_V2_DEVELOPMENT_RESULTS.csv](ACTIVE_PASSING_V2_DEVELOPMENT_RESULTS.csv)
- Final gate: [FINAL_STATUS.json](FINAL_STATUS.json)
- Latest first-person video (local-only): `development/static_obstruction/active_passing_v2_default/seed_17/run_08/first_person/CLASSIC_STATIC_OBSTRUCTION_FIRST_PERSON.mp4`
- Latest planner candidates: `run_08/passing_candidates.json`
- Latest causal planner trace: `run_08/social_planning.json`
