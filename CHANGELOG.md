# Changelog

## v0.3.0 — Prediction-Aware Social Navigation

- Added independent Prediction-Aware SocialController (`social_nav`).
- Added short-horizon CV pedestrian prediction and dynamic anisotropic personal space.
- Added KF covariance-based uncertainty inflation and seven motion primitives.
- Added social-cost-aware selection, side commitment and lightweight group-aware cost.
- Completed 30 paired runs across five frozen scenarios; all 15 SocialController runs completed without defined collision proxies.
- Preserved DemoController (`social`) as the reactive baseline.
- Added timestamp deduplication before perception submission after a reproducible startup fault; KF equations unchanged.

Status: SOCIAL NAVIGATION FUNCTIONAL PASS. Not uniform superiority: integrated personal-space cost improved in three scenarios and worsened in two. No safety certification or human comfort validation.

This is the first independent Git snapshot of this project. The tag does not fabricate earlier implementation history.

## v0.2.x — NavWareSet-style Environment

- Added NavWareSet-inspired synthetic geometry and Frontal, Obstruction, Blind Corner, Perpendicular and Circular scenarios.
- Added structured experiment logging and first/third-person video.

Historical milestone only: no corresponding project commit was found in the parent repository; no retrospective tag created.

## v0.1.x — Final A300 Demo

- Integrated Husky A300 and the kinematic skid-steer backend.
- Added RGB-D + YOLO + KF closed-loop human avoidance and indoor demonstration.
- First-person recording was added subsequently; its exact historical commit is unavailable.

Historical milestone only: no corresponding project commit was found; no retrospective tag created.
