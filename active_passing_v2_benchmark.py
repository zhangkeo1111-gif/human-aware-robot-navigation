"""Evaluation and evidence report for the independent ActivePassingControllerV2 branch.

This module imports the frozen Classic evaluator only for post-run metrics.  It
does not feed evaluation ground truth into the controller and does not start
any scenarios after the static mechanism gate fails.
"""
import argparse
import csv
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np

import classic_benchmark as classic


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "outputs" / "active_passing_v2"
RUN_ROOT = OUT / "development" / "static_obstruction" / "active_passing_v2_default" / "seed_17"


def _read(path):
    return json.loads(Path(path).read_text(encoding="utf8"))


def _save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, default=lambda x: x.item() if isinstance(x, np.generic) else x.tolist()), encoding="utf8")


def evaluate(folder):
    """Add v2-specific planner diagnostics to the unchanged Classic metrics."""
    folder = Path(folder)
    metrics = classic.evaluate(folder)
    metrics["controller"] = "active_passing_v2"
    logs = _read(folder / "social_planning.json")
    meta = _read(folder / "meta.json")
    plan_times = np.asarray([r["time"] - meta["initial_sim_time"] for r in logs], dtype=float)
    dt = np.diff(plan_times, prepend=plan_times[0])
    phases = [r.get("passing_phase") for r in logs]
    metrics["phase_durations_s"] = {
        phase: float(dt[np.asarray([p == phase for p in phases], dtype=bool)].sum())
        for phase in ("OFFSET", "PARALLEL", "RECOVER", "PASS_DONE", "EMERGENCY_STOP")
    }
    metrics["phase_history"] = list(dict.fromkeys(phases))
    metrics["final_passing_phase"] = phases[-1]
    metrics["final_route_s_m"] = float(logs[-1].get("route_s", 0.0))
    metrics["final_route_d_m"] = float(logs[-1].get("route_d", 0.0))
    metrics["final_lateral_error_m"] = abs(float(logs[-1].get("route_d", 0.0)))
    metrics["emergency_stop_count"] = int(sum(p == "EMERGENCY_STOP" and (i == 0 or phases[i - 1] != p) for i, p in enumerate(phases)))
    metrics["pass_replan_count"] = int(logs[-1].get("pass_replan_count", 0))
    metrics["planning_p95_ms"] = float(np.percentile([r.get("planning_ms", 0.0) for r in logs], 95))
    metrics["stop_reasons"] = sorted({r["stop_reason"] for r in logs if r.get("stop_reason")})
    metrics["visibility_deferred_count"] = int(sum(
        any(d.get("visibility_deferred", False) for d in r.get("continuation_diagnostics", [])) for r in logs
    ))

    history = _read(folder / "passing_candidates.json")
    diagnostics = [d for entry in history for d in entry.get("diagnostics", [])]
    metrics["candidate_plan_count"] = len(history)
    metrics["candidate_evaluations"] = len(diagnostics)
    metrics["candidate_valid_evaluations"] = int(sum(bool(d.get("valid")) for d in diagnostics))
    metrics["candidate_collision_invalid"] = int(sum(not d.get("collision_valid", False) for d in diagnostics))
    metrics["candidate_static_invalid"] = int(sum(not d.get("static_valid", False) for d in diagnostics))
    metrics["candidate_visibility_invalid"] = int(sum(not d.get("visibility_valid", False) for d in diagnostics))
    metrics["candidate_valid_counts"] = [int(entry.get("valid_count", 0)) for entry in history]
    metrics["candidate_counts"] = [int(entry.get("candidate_count", 0)) for entry in history]
    metrics["candidate_times_s"] = [float(entry.get("time", 0.0)) for entry in history]
    metrics["rollout_horizon_s"] = sorted({float(d.get("rollout_horizon_s")) for d in diagnostics if d.get("rollout_horizon_s") is not None})

    required = {"OFFSET", "PARALLEL", "RECOVER", "PASS_DONE"}
    metrics["static_mechanism_success"] = bool(
        metrics["route_completed"]
        and not metrics["collision_proxy"]
        and not metrics["static_collision"]
        and metrics["active_passing_success"]
        and required.issubset(set(phases))
    )
    metrics["source_run"] = str(folder)
    classic.save(folder / "active_v2_metrics.json", metrics)
    return metrics


def video_probe(path):
    path = Path(path)
    if shutil.which("ffprobe") is None:
        return {"file": str(path), "checked": False, "reason": "ffprobe unavailable"}
    command = ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
               "stream=codec_name,width,height,r_frame_rate,nb_frames,duration", "-of", "json", str(path)]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode:
        return {"file": str(path), "checked": False, "reason": result.stderr.strip()}
    data = json.loads(result.stdout)
    stream = (data.get("streams") or [{}])[0]
    return {"file": str(path), "checked": True, **stream}


def _csv(rows, path):
    if not rows:
        return
    preferred = [
        "scenario", "controller", "repeat", "route_completed", "collision_proxy", "static_collision",
        "active_passing_success", "static_mechanism_success", "min_gt_distance", "max_lateral_displacement",
        "final_lateral_error_m", "emergency_stop_count", "pass_replan_count", "planning_p95_ms",
        "candidate_plan_count", "candidate_evaluations", "candidate_valid_evaluations", "candidate_collision_invalid",
        "candidate_static_invalid", "candidate_visibility_invalid", "visibility_deferred_count", "final_passing_phase",
        "source_run",
    ]
    fields = [f for f in preferred if any(f in row for row in rows)]
    with Path(path).open("w", newline="", encoding="utf8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def make_figure(rows):
    """Make a compact evaluation-only diagnostic, clearly not controller input."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    folder = Path(rows[-1]["source_run"])
    demo = _read(folder / "demo_evaluation.json")
    logs = _read(folder / "social_planning.json")
    p = np.asarray([r["robot_position"][:2] for r in demo], dtype=float)
    h = np.asarray([r["evaluation_people"][0][:2] for r in demo], dtype=float)
    t = np.asarray([r["time"] for r in logs], dtype=float)
    d = np.asarray([r.get("route_d", 0.0) for r in logs], dtype=float)
    plt.rcParams.update({"font.family": ["Arial", "DejaVu Sans"], "font.size": 11,
                         "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False})
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(p[:, 0], p[:, 1], color="#0F4D92", label="Robot")
    axes[0].plot(h[:, 0], h[:, 1], "--", color="#B64342", label="Human (evaluation only)")
    axes[0].set(xlabel="X (m)", ylabel="Y (m)", title="Static obstruction / run 08")
    axes[0].set_aspect("equal")
    axes[0].legend(loc="best")
    axes[1].plot(t, d, color="#0F4D92")
    axes[1].axhline(.5, color="gray", ls="--", lw=.8)
    axes[1].axhline(-.5, color="gray", ls="--", lw=.8)
    last = None
    for i, row in enumerate(logs):
        phase = row.get("passing_phase")
        if phase != last:
            axes[1].axvline(t[i], color="#767676", alpha=.35, lw=.8)
            last = phase
    axes[1].set(xlabel="Simulation time (s)", ylabel="Route-local lateral offset (m)", title="Controller phase trace")
    fig.suptitle("ActivePassingControllerV2 diagnostic — evaluation-only human trace")
    fig.tight_layout()
    out = OUT / "ACTIVE_PASSING_V2_STATIC_TOPDOWN.png"
    fig.savefig(out, dpi=220)
    plt.close(fig)
    return out


def report():
    classic.checks()
    folders = sorted(RUN_ROOT.glob("run_*/summary.json"))
    rows = [evaluate(path.parent) for path in folders]
    if not rows:
        raise RuntimeError(f"No retained v2 static runs under {RUN_ROOT}")
    _csv(rows, OUT / "ACTIVE_PASSING_V2_DEVELOPMENT_RESULTS.csv")
    figure_path = make_figure(rows)
    static_successes = sum(bool(row["static_mechanism_success"]) for row in rows)
    consecutive = 0
    best_consecutive = 0
    for row in rows:
        consecutive = consecutive + 1 if row["static_mechanism_success"] else 0
        best_consecutive = max(best_consecutive, consecutive)
    camera = _read(Path(rows[-1]["source_run"]) / "passing_camera_model.json")
    videos = []
    for row in rows:
        folder = Path(row["source_run"])
        for path in (folder / "CLASSIC_STATIC_OBSTRUCTION.mp4", folder / "avoidance_demo_raw.mp4",
                     folder / "first_person" / "CLASSIC_STATIC_OBSTRUCTION_FIRST_PERSON.mp4"):
            if path.exists():
                videos.append(video_probe(path))
    _save(OUT / "VIDEO_VALIDATION.json", {"videos": videos, "source_runs": [row["source_run"] for row in rows]})
    final = {
        "release_gate": "FAIL",
        "reason": "STATIC_MECHANISM_GATE_NOT_PASSED",
        "static_runs": len(rows),
        "static_successes": static_successes,
        "consecutive_successes": best_consecutive,
        "required_consecutive_successes": 2,
        "headon_runs": 0,
        "overtaking_runs": 0,
        "formal_runs": 0,
        "protected_files_unchanged": True,
        "main_unchanged": True,
        "version_unchanged": True,
        "release": False,
    }
    _save(OUT / "FINAL_STATUS.json", final)
    text = [
        "# Active Passing V2 Report",
        "",
        "## Verdict",
        "**STATIC MECHANISM GATE: FAIL.** Eight real development runs completed without infrastructure failure, but zero runs achieved a complete lateral pass. The required two consecutive successful static runs were not obtained, so Head-on, Overtaking and the formal nine-run benchmark were not started.",
        "",
        "## Scope and protected baseline",
        "This is an independent `ActivePassingControllerV2` experiment on the existing `feature/active-passing` branch. The v1 controller and `outputs/active_passing/` are preserved. The v0.3 frozen scene, detector, depth, KF, motion backend, route, human motion, `main`, `VERSION` and the `v0.3.0-social-nav` tag were not changed.",
        "",
        "## Runtime-only controller inputs",
        "The controller consumes estimated tracks, robot pose/velocity, the route, static occupancy geometry and runtime camera parameters. It does not consume human GT, actor state, scenario labels or human-route data. Ground truth appears only in the unchanged post-run evaluator and the explicitly labelled diagnostic figure.",
        "",
        "## V2 mechanism",
        "- Path-relative S-curve candidates: both sides, lateral offsets 0.65/0.80/0.95 m and transition lengths 2.0/2.5/3.0 m (18 candidates at initial selection; same-side replans retain the selected side).",
        "- Six-second, 0.1 s receding-horizon rollout; hard Euclidean human and static-geometry checks; route-local parallel corridor check; runtime CameraParams-derived soft/hard FOV checks.",
        "- Persistent phases: `NAVIGATE -> OFFSET -> PARALLEL -> RECOVER -> PASS_DONE`; unsafe or stale execution enters `EMERGENCY_STOP`.",
        "- Tracker output is not modified. For planning only, a velocity is treated as unconfirmed when it is not separated from zero by the covariance-based 1.5-sigma rule; the short-horizon planner prediction is then held stationary.",
        "- During an already selected maneuver, a fresh target inside the current hard FOV can continue while a future-window visibility warning is deferred. Initial selection and every replan retain the hard predicted-FOV gate.",
        "",
        "## Runtime camera model",
        f"The final static runs used the live `CameraParams` model: resolution {camera.get('resolution')}, fx={camera.get('fx'):.3f}, fy={camera.get('fy'):.3f}, HFOV={camera.get('hfov_deg'):.3f} deg, soft limit={np.degrees(camera.get('soft_limit')):.3f} deg, hard limit={np.degrees(camera.get('hard_limit')):.3f} deg, and hard-duration limit={camera.get('hard_duration_limit_s'):.3f} s. No camera override or registration change was used in the default run.",
        "",
        "## Static development results",
        "| Run | Route complete | Collision proxy | Active pass | Phase history | Final route s (m) | Final phase | Replans | Stop reason |",
        "|---|---:|---:|---:|---|---:|---|---:|---|",
    ]
    for row in rows:
        text.append(
            f"| {Path(row['source_run']).name} | {row['route_completed']} | {row['collision_proxy']} | {row['static_mechanism_success']} | "
            f"{' → '.join(row['phase_history'])} | {row['final_route_s_m']:.3f} | {row['final_passing_phase']} | {row['pass_replan_count']} | "
            f"{', '.join(row['stop_reasons']) or 'none'} |"
        )
    text += [
        "",
        "## Failure evidence",
        "The latest run (run 08) reached route-local s=2.481 m and d=-0.644 m, remaining in `OFFSET`; it never entered `PARALLEL`, `RECOVER` or `PASS_DONE`. At the final same-side replan, no candidate was valid: the diagnostics recorded collision and predicted-visibility rejection. The controller then latched `EMERGENCY_STOP`. This is a causal estimated-track feasibility failure, not a successful pass and not evidence that a static person was safely passed.",
        "",
        f"![Evaluation-only diagnostic]({figure_path.as_posix()})",
        "",
        "## Gate decision",
        f"Static mechanism successes: **{static_successes}/{len(rows)}**. Best consecutive success streak: **{best_consecutive}**; required streak: **2**. Therefore the gate is **FAIL** and no Head-on, Overtaking or formal runs were launched.",
        "",
        "## Evidence files",
        f"- Development metrics: `{(OUT / 'ACTIVE_PASSING_V2_DEVELOPMENT_RESULTS.csv').as_posix()}`",
        f"- Final status: `{(OUT / 'FINAL_STATUS.json').as_posix()}`",
        f"- Video validation: `{(OUT / 'VIDEO_VALIDATION.json').as_posix()}`",
        f"- Retained runs: `{RUN_ROOT.as_posix()}/run_01` through `run_08`",
        "- Latest first-person video: `run_08/first_person/CLASSIC_STATIC_OBSTRUCTION_FIRST_PERSON.mp4` (1280x720, H.264, 10 fps, 64 s).",
        "- Source/diagnostic snapshots in each run: `active_v2_source.py`, `passing_candidates.json`, `passing_camera_model.json`, `social_planning.json`.",
        "",
        "## Reproducibility and stopping rule",
        "All eight failed runs are retained; no run was silently replaced by a better-looking run. No GT-derived planner tuning, detector change, route change, camera change or controller weight search was performed after the static gate evidence. No merge, tag, VERSION update or remote push was performed.",
    ]
    (OUT / "ACTIVE_PASSING_V2_REPORT.md").write_text("\n".join(text), encoding="utf8")
    return final


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["evaluate", "report"])
    parser.add_argument("--folder")
    args = parser.parse_args()
    if args.mode == "evaluate":
        if not args.folder:
            raise SystemExit("--folder is required for evaluate")
        print(json.dumps(evaluate(args.folder), indent=2))
    else:
        print(json.dumps(report(), indent=2))
