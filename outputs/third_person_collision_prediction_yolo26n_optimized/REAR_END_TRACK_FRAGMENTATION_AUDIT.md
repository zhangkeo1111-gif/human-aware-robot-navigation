# Rear-end causal fragmentation audit (frozen recordings)

Source: seed 17 `rear_end_collision` exposure logs for the existing YOLO11n and YOLO26n runs. `REAR_END_TRACK_FRAGMENTATION_AUDIT.csv` contains one row per detector observation (or one empty row for an exposure without a box), replayed with the unchanged `config.yaml` tracker. It includes predicted prior XY, association distance, innovation/Mahalanobis value, track age, births/deaths, and overlapping-box evidence. This is a post-run diagnosis, not a GT input to inference.

| Observation | YOLO11n | YOLO26n |
|---|---:|---:|
| Exposure frames | 349 | 349 |
| Exposures with no person box | 79 | 70 |
| Exposures with overlapping boxes (max IoU ≥ 0.50) | 16 | 19 |
| Exposures with ≥2 active tracks within 0.15 m | 19 | 24 |
| GT-associated ID switches (existing evaluator) | 22 | 46 |
| GT-associated unique-ID fragments (existing evaluator) | 9 | 11 |

## Event-level findings

- Frames 1–44: long intervals without a box exhaust the unchanged five-miss lifecycle. YOLO26n tracks 1 and 2 die at frames 9 and 40; tracks 2 and 3 are reborn at frames 32 and 44. This is detector intermittency followed by ordinary track retirement, **not** a Hungarian distance-gate failure. YOLO11n has the analogous sequence at frames 9/35/41/48/59/66.
- YOLO26n frame 130 (12.98 s): two boxes overlap at IoU ≈ 0.68 and yield world XY about (4.32, −0.17) and (4.31, −0.11) m. The second measurement is within ≈0.04 m of the existing track, but one-to-one Hungarian assignment leaves it unmatched and creates track 4. Frames 134–135 show the same overlapping pair. These are duplicate same-person measurements, not a large position innovation.
- Frames 132–218: one of the two nearby tracks alternately receives the single surviving box. The existing evaluator, which associates the active track closest to the single GT human each exposure, alternates IDs even while a long-lived track persists. YOLO26n has more such nearby-track exposures (24 vs 19), explaining much of its larger **reported** switch count. This count is partly an evaluator/duplicate-selection effect, not 46 independent identity-loss events.
- Frames 133/141/151: isolated boxes with torso depth ≈9.1 m appear while the supported human depth is ≈7.2–7.3 m. Their world XY is ≈1.1–1.2 m from the prior human track, beyond the unchanged 0.8 m association gate, so they form short-lived off-person tracks. The large depth/world jump is on the extra box; the main human measurement remains smooth. No evidence supports changing KF Q/R or the gate.
- Frames 148, 169, 202, 217, 233 and 340 show further overlapping same-person measurements 0.01–0.05 m from an existing track and fresh duplicate births. YOLO11n also has duplicate births (e.g. frames 77 and 91), but fewer alternating nearby-track exposures.

## Answers to the requested checks

1. **BBox jump:** early track rebirth follows absent boxes; later excess switches cluster around overlapping/toggling boxes. The principal person's box usually moves smoothly. Some extra boxes jump to another depth/background region.
2. **Confidence:** duplicate extra boxes are often low (roughly 0.25–0.37), but low confidence alone does not explain all switches. No confidence-threshold change is justified.
3. **Depth/world XY jump:** present on some extra boxes (≈7.3 → 9.1 m), not the principal human sequence; same-person duplicate boxes are often close in XY.
4. **Hungarian/gating vs death:** early fragmentation is death after consecutive detection misses; later duplicate births are a one-to-one assignment consequence. Background-box births exceed the 0.8 m gate. The replay found no innovation-gate rejection in either rear-end run.
5. **Two nearby tracks for one human:** yes, 24 YOLO26n exposure frames versus 19 YOLO11n frames under the stated 0.15 m proximity diagnostic.
6. **Why YOLO26n appears worse:** more overlapping-box exposures and more alternating selection of nearby duplicate tracks, despite fewer box-free exposures. Both detector outputs and the evaluator's nearest-active-track ID definition contribute; this audit does not establish an intrinsic detector identity capability.

## Minimal-change implication

Keep the original raw tracker/log. At the predictor interface, suppress only same-exposure measurements with strong bbox overlap **and** very close world XY/depth, while retaining an untouched raw KF replay for audit. Do not change detector, depth estimator, KF noise, gate, or track lifecycle. The long early no-detection gaps will remain unresolved by this change.
