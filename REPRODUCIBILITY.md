# Reproducibility and version policy

Stable version: **0.3.0**, annotated tag **v0.3.0-social-nav**. Status: SOCIAL NAVIGATION FUNCTIONAL PASS (simulation functional validation, not uniform superiority or safety certification).

## Environment and commands

- Windows; NVIDIA Isaac Sim **6.0.1**, existing local `runtime/python.bat` (runtime excluded from Git).
- Existing YOLO worker environment: `D:/detection/robot_human_avoidance/.venv/python.exe`; official COCO `yolo11n.pt` at `D:/detection/robot_human_avoidance/yolo11n.pt`. Weights/environment remain external; a Git checkout alone is not a complete offline installation.
- Main parameters: `config.yaml`; frozen experiment definition: `outputs/social_navigation/BENCHMARK_CONFIG_FROZEN.json`. No parameters were changed for this release.

From `D:/detection/robot_human_isaac6`:
```powershell
.\runtime\python.bat stage_probe.py --stage 7 --robot-model a300 --motion-mode kinematic --navwareset-scene --navwareset-scenario circular --robot-behavior social_nav --record-demo --record-first-person
python social_benchmark.py checks
python social_benchmark.py benchmark
```
Use `--robot-behavior social` for the unchanged Reactive Baseline. Indoor: `--demo-scene --demo-controller` instead of the NavWareSet flags. Benchmark resumes missing combinations, with paired labels 17/23/31; the frozen character loader uses Randomizer(17), so these are timing-repeatability runs, not independent populations. No benchmark rerun is needed for this release.

## Assets and evidence

- Official A300 description: https://github.com/clearpathrobotics/clearpath_common.git ; branch `jazzy`; commit `893db1fdb37d6e601d85eb53573edafa991a67b9`.
- Both existing nested checkouts (`assets/husky_a300/clearpath_common` and `assets/husky_a300/source/clearpath_common`) were clean at that commit. They are preserved and ignored, not converted into submodules. `prepare_asset.py` uses the first path.
- Track the two small scene USDs, scene builder source, A300 wrapper/converted USD payloads (~4.4 MB), URDF/xacro, preparation script and parameter metadata. Converted assets are required by the current launcher and are retained unchanged; the expanded URDF has local mesh paths and regeneration requires the external checkout. No re-conversion was performed. Official character assets/Isaac caches remain external.
- Raw outputs, videos, frames, runtime, downloads, logs and arrays stay local and ignored. Explicit exceptions preserve small final reports, summary CSV, frozen metadata, diagnostic figure, and six source references required by existing checks. References to raw runs/videos in reports intentionally require the local output store.
- Historical 30-run evidence is retained unchanged. Release checks on 2026-09-16: Python compilation PASS; fresh 5-second Indoor / NavWareSet baseline / SocialController starts each completed 300 steps and 50 RGB frames, with 48 / 48 / 47 YOLO responses. Logs: ignored `outputs/version_management/`. Existing generated USD/SVG whitespace warnings were reviewed and retained without rewriting assets or results.

## Git policy

Independent local repository created with user approval; the parent `D:/detection` repository and its unrelated dirty state/history are untouched. No prior Isaac project commits/tags existed there. No remote or push is configured by this release.

`main` = most recent verified stable snapshot. Develop on `feature/<name>`, `fix/<name>` or `experiment/<name>`; e.g. `git switch -c feature/lidar main`. Do not merge unverified work into main or move release tags. Future releases use semantic version numbers and annotated milestone tags. v0.1/v0.2 are documented milestones only, not invented Git tags.
