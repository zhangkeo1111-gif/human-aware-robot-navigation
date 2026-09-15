# A300 skid-steer command-response audit

Date: 2026-09-11. Final status: **PARTIAL — yaw-response acceptance failed**.

This is a physics/control diagnostic, not an avoidance validation. The A300 assets, wheel geometry, mass, camera, human, YOLO, Depth and KF algorithms were not changed. No omega multiplier was introduced. No rejected parameter trial was promoted to the default configuration.

## Root cause: evidence and limits

The immediate failure is a large discrepancy between differential wheel motion and chassis yaw under the imported four-cylinder contact model. This is **not explained by insufficient max-force clipping, an inverted wheel mapping, hidden chassis-ground contact, or an obviously inflated imported yaw inertia** in the checks performed.

Evidence supports contact/slip/drivetrain interaction as the remaining mechanism, but does **not uniquely prove a PhysX bug or a particular missing tire parameter**:

- Baseline damping 100 tracks curve wheel targets reasonably closely while chassis yaw remains very small. Reducing damping to 10 destroys differential wheel-speed tracking. Increasing it to 1000 reduces curve wheel-speed error without restoring chassis yaw.
- Four finite isotropic-friction settings do not restore curve yaw. The lowest setting also degrades reverse motion and in-place turning. Uniformly reducing friction is not an acceptable fix.
- Actual contact reports contain only the four wheel-cylinder/ground pairs. No chassis, motor, suspension or bumper ground-contact pair was reported.
- Wheel axes, dimensions, rigid masses and inertias are consistent with the imported source. No geometry or inertia rescaling was performed.
- There was a real contact-configuration issue: ground restitution 0.8 combined with tire restitution 0 using `average`, producing an effective 0.4. A tire-specific `min` combination eliminates the roughly 0.047 m/s reported vertical-velocity residual. It **does not resolve yaw underresponse**.
- Increasing articulation solver iterations to 64/8 still leaves curve yaw near 0.028 rad/s. This is not a convergence-only remedy.

**Concrete blocker:** the current isotropic rigid-wheel setup and ideal wheel mapping do not satisfy the yaw gate; a single calibrated gain is invalid because response depends strongly on translation, direction and command magnitude. A supported directional tire-slip model has not been established in this installed rigid-material API. Further model work would be a separate, explicitly scoped step, not another blind scalar sweep.

## Physics configuration audit

| Item | Verified value / evidence |
|---|---|
| Wheel radius / width | 0.1651 / 0.1143 m; four Cylinder colliders, not visual meshes |
| Track / wheelbase | 0.5468 / 0.512 m; original left/right center asymmetry preserved |
| Joint axes | Y; collider local Z rotated 90 degrees around X, aligning cylinder axis with wheel axle |
| Wheel order in logged arrays | front-left, front-right, rear-left, rear-right |
| Drive | Force velocity drive; stiffness 0; baseline damping 100 N m s/rad |
| USD damping units | 1.745329 per degree corresponds to 100 per radian; CLI conversion is explicit |
| Max drive force / velocity | USD unbounded; runtime max effort approximately 3.4e38, max velocity 17453.293 rad/s; these are importer limits, **not OEM ratings** |
| Wheel target vs actual | Logged every 0.1 simulation second in `robot_backend.json`; see tables below |
| Joint effort signal | `get_measured_joint_efforts()`, peak magnitude 30.589 in baseline and 29.226 in final matrix. This is a measured joint-effort signal, not a separately validated actuator-current/drive-torque estimate |
| Base body mass / COM | 70 kg; COM [0, 0, 0.1147795] m |
| Base diagonal inertia | [1.1408911, 4.6217346, 5.1478233] kg m²; identity principal rotation |
| Wheel bodies | 2.5 kg each; principal inertia [0.019758025, 0.019758025, 0.03407251] kg m²; principal rotation aligns axial inertia with axle |
| Total specified mass | 80 kg; no mass retuning. Source fixed attachments without separate inertials remain a source-model limitation |
| Tire friction | Baseline static/dynamic 0.5/0.4, friction combine `min` |
| Ground | Static/dynamic 0.5/0.5, restitution 0.8; no explicit combine override (average default) |
| Tire restitution | 0; baseline combine average, optional diagnostic combine min |
| Solver / step | TGS, CPU rigid dynamics, PCM/patch friction; 1/60 s physics and render steps, one physics step per loop; no added substeps |
| Iterations | Baseline unauthored; installed schema defaults are 32/1 for articulations. The convergence trial explicitly authors and reads back 64/8 |
| Contact scene settings | CCD true; bounce threshold 0; friction correlation distance 0.025 m; friction offset threshold 0.04 m |

Source: `outputs/stage2_a300_physics_baseline/physics_audit.json`; corrected complete collider bounds and contact audit: `outputs/stage2_a300_solver64_8_checked/physics_audit.json`.

The full instance-aware traversal finds **16 collision shapes**, of which **4 are wheel cylinders**, and 50 visual meshes with 790,958 faces. The earlier integration report's “5 collision shapes” was an incomplete non-instance traversal; it must not be used as the current count. No duplicate wheel collider was found.

Ground contacts in baseline: all four wheels, minimum reported separation approximately -3.33e-6 m; no hidden body-ground pair. Conservative transformed USD bounding boxes can extend below the plane because they enclose rotated local bounds; **they are not penetration measurements**. Use actual contact separations for that claim. The early audit returned empty bounds because guide-purpose colliders were excluded; the corrected audit includes guide purpose and represents any unavailable bound as null.

### Directional friction support

The installed `PhysxMaterialAPI` exposes friction/restitution combine modes and compliant-contact attributes, but no longitudinal/lateral friction coefficients or rigid tire friction-direction binding. No anisotropic material was fabricated from an unverified interface. This is a limitation of the verified integration path, not a claim that every PhysX vehicle extension lacks tire modeling.

Official references: [PhysxMaterialAPI](https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/latest/physxschema/class_physx_schema_physx_material_a_p_i.html), [rigid-body dynamics](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyDynamics.html). Installed schema and actual runtime records take precedence over generic documentation defaults.

## Method and finite A/B sequence

Each short test uses 4 s per command: forward, reverse, left curve, right curve, positive rotation, stop. Statistics exclude the first second of each block and use samples before 3.95 s. The full matrix has ten requested motion points plus a stop block (44 s). These are finite development probes, not repeated-seed confidence estimates.

Velocity columns below use the robot velocity API for continuity with the supplied baseline. **Pose-derived yaw is also reported**: it is smaller at several operating points. Neither signal passes the gate; the more favorable API signal must not be treated as proof of physical turn accuracy.

| Trial | Forward m/s | Reverse m/s | Left yaw rad/s | Right yaw rad/s | Rotation yaw rad/s |
|---|---:|---:|---:|---:|---:|
| Baseline: damping 100, friction .5/.4 | .29927 | -.30057 | .02217 | -.02755 | .16816 |
| Damping 10, other settings baseline | .29909 | -.30063 | -.00144 | -.00080 | -.00193 |
| Friction .4/.3, damping 100 | .29929 | -.30056 | .01999 | -.02715 | .17420 |
| Friction .3/.2, damping 100 | .29927 | -.30054 | .01806 | -.02655 | .18169 |
| Friction .1/.05, damping 100 | .29929 | -.28410 | .01389 | -.02598 | .04254 |
| Damping 1000, other settings baseline | .29936 | -.30045 | .01936 | -.02755 | .18506 |
| Restitution min only | .29949 | -.30034 | .02143 | -.02850 | .18282 |
| Restitution min + solver 64/8 | .29974 | -.30006 | .02789 | -.02826 | .15956 |

There was no friction × damping grid. Damping 1 and 0.1 were not pursued after 10 already eliminated differential response. Max force was not increased because the imported limit was already effectively unbounded. The 1000 trial tests whether better wheel-speed tracking alone fixes yaw; it does not.

Left-curve wheel-speed MAE (rad/s): damping 10 = .69591; 100 = .04896; 1000 = .00897. Thus calling damping 100 simply “excessive resistance” is incorrect: it is velocity-error servo gain, and lowering it weakened tracking.

## Before

| v cmd m/s | omega cmd rad/s | v actual m/s | omega actual rad/s |
|---:|---:|---:|---:|
| .30 | 0 | .29927 | -.00001 |
| -.30 | 0 | -.30057 | -.00001 |
| .20 | .40 | .14544 | .02217 |
| .20 | -.40 | .18365 | -.02755 |
| 0 | .40 | -.00519 | .16816 |

## After: complete diagnostic matrix

Configuration: **damping 100, friction .5/.4, restitution-combine min, unchanged geometry and mapping**. This is the fully tested diagnostic configuration, **not a promoted yaw fix**. The later 64/8 solver trial did not pass either, so no second tuned configuration is claimed as final.

The wheel-command formula makes ideal omega exactly equal to omega cmd at every point below.

| v cmd | omega cmd / ideal | v actual | omega API | pose-derived omega | yaw gain (API/cmd) |
|---:|---:|---:|---:|---:|---:|
| .30 | 0 | .29949 | .00003 | .00000 | — |
| -.30 | 0 | -.30034 | -.00002 | .00000 | — |
| .20 | .20 | .18807 | .01098 | .00275 | .0549 |
| .20 | -.20 | .20182 | -.01308 | -.00433 | .0654 |
| .20 | .40 | .14604 | .02119 | .00926 | .0530 |
| .20 | -.40 | .18857 | -.02846 | -.01864 | .0712 |
| .20 | .60 | .07731 | .01768 | .00549 | .0295 |
| .20 | -.60 | .13241 | -.04012 | -.02799 | .0669 |
| 0 | .40 | -.00222 | .18289 | .13676 | .4572 |
| 0 | -.40 | .00139 | -.18285 | -.13662 | .4571 |
| 0 | 0 | -.00046 | .00000 | .00000 | — |

Units: v in m/s, omega in rad/s. Exact values and signed linear-speed error are in `outputs/stage2_a300_final_matrix/motion_summary.json`.

**No correction factor is justified.** Curve gains .0295–.0712 and rotation gains ~.457 are not a common constant. Left .6 produces less yaw than left .4, while right response is different. Rotation is approximately symmetric; moving turns are not. The official small wheel-center asymmetry is preserved, but this audit does not establish it as the unique cause of the much larger dynamic asymmetry.

## Wheel behavior and stability

Final matrix wheel arrays, rad/s, ordered FL/FR/RL/RR:

| v, omega | Targets | Mean actual |
|---|---|---|
| .30, 0 | [1.8171, 1.8171, 1.8171, 1.8171] | [1.8132, 1.8143, 1.8141, 1.8133] |
| -.30, 0 | [-1.8171, -1.8171, -1.8171, -1.8171] | [-1.8196, -1.8196, -1.8207, -1.8208] |
| .20, .20 | [.8802, 1.5426, .8802, 1.5426] | [.8473, 1.6218, .8145, 1.6701] |
| .20, -.20 | [1.5426, .8802, 1.5426, .8802] | [1.5316, .8236, 1.5380, .8959] |
| .20, .40 | [.5490, 1.8738, .5490, 1.8738] | [.5201, 1.8592, .5703, 1.9233] |
| .20, -.40 | [1.8738, .5490, 1.8738, .5490] | [1.8746, .5498, 1.8959, .5435] |
| .20, .60 | [.2178, 2.2050, .2178, 2.2050] | [.2354, 2.1858, .2300, 2.2359] |
| .20, -.60 | [2.2050, .2178, 2.2050, .2178] | [2.2264, .2459, 2.2197, .2146] |
| 0, .40 | [-.6624, .6624, -.6624, .6624] | [-.6752, .5786, -.6425, .5792] |
| 0, -.40 | [.6624, -.6624, .6624, -.6624] | [.6326, -.5920, .6793, -.5707] |

No command clipping. Largest target magnitude 2.205 rad/s; largest settled sampled wheel speed 2.4921 rad/s. No exaggerated wheel commands were used.

Across settled matrix windows: maximum |lateral velocity| .004384 m/s; maximum |roll| or |pitch| .000370 degrees; base z .1359690–.1359717 m. These establish numerical body stability in this run, **not realistic tire fidelity**. Low chassis lateral speed also does not mean low wheel-contact slip: a four-wheel skid turn requires local lateral slip, and rolling-equivalent wheel speeds are inconsistent with chassis motion here.

RTF: baseline short motion 1.5706; full matrix 1.5635; solver 64/8 short trial 1.4755. These are motion-only wall-clock measurements on this laptop, not full-perception real-time claims.

## Changes actually made

- `robot_backend.py`: opt-in tire friction, velocity-drive damping, restitution-combine min and articulation-iteration diagnostics. Asset geometry, masses, imported source and wheel mapping unchanged.
- `stage_probe.py`: finite matrix/curve command sequences, opt-in physical settings, wheel/body diagnostics and unambiguous open-loop curve-demo labels.
- `a300_motion_validation.py`: contact/material/rigid-body audit and settled-window command-response summaries.
- `config.yaml`: **unchanged this round**. Default friction .5/.4, damping 100 and original restitution combination remain. The zero-restitution candidate is explicitly selected by `--tire-restitution-min`; it is not silently promoted as a yaw repair.
- No changes to `yolo_worker.py`, `walking_actor.py`, `tracker.py`, installed runtime source or official Clearpath description.

Two diagnostic API mistakes were corrected during execution (BBoxCache positional arguments / guide-purpose bounds; applying typed articulation attributes before reset instead of calling a legacy setter on missing USD attributes). Their failed logs remain visible; they are not counted as completed trials. The completed corrected audit is `stage2_a300_solver64_8_checked`.

## Perception regression and curve demo

`outputs/stage7_a300_curve_perception`: 20 simulation seconds, open-loop forward → left → forward → right → stop, no avoidance. Same walking actor and perception configuration. RGB-D mount [0.37, 0, 0.50] m, 640×360, HFOV approximately 106 degrees, clipping .05–100 m.

- 200 RGB camera ticks; 198 completed YOLO responses; depth available for 198/198 responses.
- 111 responses with a person detection, 121 with a track, 117 accepted depth/localization rows. Counts have different units; they are not detection recall.
- First forward and left-turn windows: 34/34 and 40/40 received responses respectively contain detections and tracks. Later person visibility declines as the open-loop robot passes the actor; the stop window has no person detection. This is a functional smoke test, not a full-FOV perception benchmark.
- Full recorded run RTF .4976, ~29.86 physics steps/s and 4.93 completed YOLO responses/s. Recording a second 1280×720 RTX view adds work; these figures are not directly comparable with the motion-only RTF.
- `A300_CURVE_DIAGNOSTIC.mp4` is H.264, 20 s at 10 fps, made from the real simulation render. Preview checked. It shows **remaining understeer**, not a passing curve or avoidance demo.

## JetBot regression

Final current-code motion regression completed in `outputs/stage2_jetbot_skid_regression/`: forward .33089 m/s, reverse -.33910 m/s, left/right yaw +.41839/-.41554 rad/s, rotation +.41354 rad/s. RTF 1.6753. These reproduce the earlier JetBot motion behavior; no A300 contact override was applied to JetBot. This round rechecks motion, not a new JetBot perception benchmark.

## Reproduction

Run from `D:/detection/robot_human_isaac6`; no installation or asset conversion is needed.

```powershell
.\runtime\python.bat stage_probe.py --stage 2 --motion-test --physics-audit --seconds 90 --label _a300_physics_baseline
.\runtime\python.bat stage_probe.py --stage 2 --motion-test --wheel-damping 10 --seconds 90 --label _a300_damping10
.\runtime\python.bat stage_probe.py --stage 2 --motion-test --tire-friction .4 .3 --seconds 90 --label _a300_friction04
.\runtime\python.bat stage_probe.py --stage 2 --motion-test --tire-friction .3 .2 --seconds 90 --label _a300_friction03
.\runtime\python.bat stage_probe.py --stage 2 --motion-test --tire-friction .1 .05 --seconds 90 --label _a300_friction01
.\runtime\python.bat stage_probe.py --stage 2 --motion-test --wheel-damping 1000 --seconds 90 --label _a300_damping1000
.\runtime\python.bat stage_probe.py --stage 2 --motion-matrix --tire-restitution-min --physics-audit --seconds 120 --label _a300_final_matrix
.\runtime\python.bat stage_probe.py --stage 2 --motion-test --tire-restitution-min --solver-iterations 64 8 --physics-audit --seconds 90 --label _a300_solver64_8_checked
.\runtime\python.bat stage_probe.py --stage 7 --curve-demo --record-demo --tracking-check --robust-depth --tire-restitution-min --seconds 180 --label _a300_curve_perception
.\runtime\python.bat stage_probe.py --stage 2 --robot-model jetbot --motion-test --seconds 90 --label _jetbot_skid_regression
```

Use a new label if preserving old run files. CLI process exit alone is not acceptance: inspect `summary.json`, `motion_summary.json` and any `failure.txt`.

## Remaining issues and final status

1. Curve |yaw| is well below .25 rad/s; rotation is below .30 rad/s. The core task is **not solved**.
2. Moving-turn gain is nonlinear/asymmetric, so a fixed correction factor would hide errors rather than calibrate them.
3. Velocity-API yaw and pose-derived yaw differ; both are retained, neither supports PASS. More solver iterations do not close the performance gap.
4. No validated longitudinal/lateral tire model or OEM actuator identification is available in this integration. Do not claim physical fidelity from the official visual/geometry asset alone.
5. Actual high-curvature perception and collision avoidance remain unvalidated because the chassis cannot yet execute those curves correctly.

**Final status: PARTIAL.** Geometry/contact audit, finite A/B, full matrix and functional perception/video checks were executed. Straight motion remains normal. No successful skid-steer repair or safe-avoidance result is claimed.
