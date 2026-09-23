"""Post-run GT evaluation only; never imported by the online predictor."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linear_sum_assignment

ROOT = Path(__file__).resolve().parent / 'outputs' / 'third_person_collision_prediction'
CASES = ('headon_collision', 'perpendicular_collision', 'diagonal_collision',
         'cutin_collision', 'rear_end_collision', 'perpendicular_nearmiss',
         'diagonal_nearmiss', 'cutin_nearmiss', 'multi_person_collision')


def hud_video(folder, frames):
    """Add d_CPA to the recorded same-exposure video; no GT or new detections."""
    import cv2
    source = folder/'THIRD_PERSON_COLLISION_PREDICTION.mp4'
    target = folder/'THIRD_PERSON_COLLISION_PREDICTION_HUD.mp4'
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f'Cannot read {source}')
    writer = cv2.VideoWriter(str(target), cv2.VideoWriter_fourcc(*'mp4v'), 10., (640, 360))
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f'Cannot write {target}')
    written = 0
    try:
        for frame in frames:
            okay, image = capture.read()
            if not okay:
                raise RuntimeError(f'Video shorter than perception trace at frame {written}')
            overlay = image.copy()
            cv2.rectangle(overlay, (0, 264), (640, 360), (10, 17, 25), -1)
            image = cv2.addWeighted(overlay, .88, image, .12, 0)
            risk_by_id = {r['track_id']: r for r in frame['risks']}
            active = [t for t in frame['tracks'] if t['miss_count']==0]
            for j, track in enumerate(active[:3]):
                x, y, vx, vy = track['state']
                risk = risk_by_id.get(track['track_id'])
                level = risk['risk'] if risk else 'WARMUP'
                ttc = risk['predicted_ttc_s'] if risk else None
                cpa = risk['d_cpa_m'] if risk else None
                line1 = f"ID {track['track_id']}  XY {x:.2f},{y:.2f}m  V {vx:.2f},{vy:.2f}m/s"
                line2 = f"{level}  TTC {'--' if ttc is None else f'{ttc:.2f}s'}  d_CPA {'--' if cpa is None else f'{cpa:.2f}m'}"
                y0 = 276+28*j
                cv2.putText(image, line1, (8, y0), cv2.FONT_HERSHEY_SIMPLEX,
                            .39, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(image, line2, (8, y0+13), cv2.FONT_HERSHEY_SIMPLEX,
                            .39, (80, 190, 255) if level=='COLLISION_RISK' else (210, 230, 230),
                            1, cv2.LINE_AA)
            writer.write(image)
            written += 1
        if capture.read()[0]:
            raise RuntimeError('Video longer than perception trace')
    finally:
        capture.release()
        writer.release()
    return str(target)


def gt_at(rows, stamp, count):
    times = np.asarray([r['time'] for r in rows])
    human = np.asarray([r['human_xy'] for r in rows], dtype=float)
    position = np.asarray([[np.interp(stamp, times, human[:, i, j]) for j in range(2)]
                           for i in range(count)])
    # Central finite difference over 0.4 s reduces walking-gait/root jitter.
    velocity = np.asarray([[(np.interp(stamp+.2, times, human[:, i, j])-
                             np.interp(stamp-.2, times, human[:, i, j]))/.4
                            for j in range(2)] for i in range(count)])
    return position, velocity


def overlapping_box_pairs(boxes):
    """Count high-IoU box pairs as a detector-duplicate proxy, not GT precision."""
    count = 0
    for i, a in enumerate(boxes):
        for b in boxes[i+1:]:
            x1, y1 = max(a[0], b[0]), max(a[1], b[1])
            x2, y2 = min(a[2], b[2]), min(a[3], b[3])
            intersection = max(0., x2-x1)*max(0., y2-y1)
            area_a = max(0., a[2]-a[0])*max(0., a[3]-a[1])
            area_b = max(0., b[2]-b[0])*max(0., b[3]-b[1])
            if intersection / max(area_a+area_b-intersection, 1e-9) > .8:
                count += 1
    return count


def assess(case, seed=17):
    folder = ROOT / case / f'seed_{seed}'
    if not (folder/'summary.json').exists():
        return None
    summary = json.loads((folder/'summary.json').read_text())
    frames = json.loads((folder/'perception.json').read_text())
    truth = json.loads((folder/'evaluation_gt.json').read_text())
    count = len(summary['human_specs'])
    errors, velocity_errors = [], []
    current_ids = [None]*count
    associated_ids = [set() for _ in range(count)]
    switches = 0
    duplicates = 0
    observed = 0
    track_support = 0
    estimates = [[] for _ in range(count)]
    risk_by_track = {}
    first_risk_row = None
    for frame in frames:
        observed += bool(frame['boxes'])
        active = [t for t in frame['tracks'] if t['miss_count']==0]
        stamp = frame['exposure_time']
        hxy, hv = gt_at(truth, stamp, count)
        if active:
            track_support += 1
            xy = np.asarray([t['state'][:2] for t in active])
            cost = np.linalg.norm(hxy[:, None, :]-xy[None, :, :], axis=2)
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if cost[i, j] > 1.5:
                    continue
                t = active[j]
                errors.append(float(cost[i, j]))
                velocity_errors.append(float(np.linalg.norm(np.asarray(t['state'][2:])-hv[i])))
                estimates[i].append((stamp, *t['state'][:2]))
                if current_ids[i] is not None and current_ids[i] != t['track_id']:
                    switches += 1
                current_ids[i] = t['track_id']
                associated_ids[i].add(t['track_id'])
            duplicates += max(0, len(active)-count)
        if frame['risks'] and first_risk_row is None and any(r['risk']=='COLLISION_RISK' for r in frame['risks']):
            first_risk_row = frame
        for r in frame['risks']:
            identifier = r['track_id']
            if r['risk']=='COLLISION_RISK' and identifier not in risk_by_track:
                risk_by_track[identifier] = frame['predictor_time']
    position_mae = float(np.mean(errors)) if errors else None
    position_p95 = float(np.percentile(errors, 95)) if errors else None
    velocity_mae = float(np.mean(velocity_errors)) if velocity_errors else None
    first = next((f for f in frames if any(r['risk']=='COLLISION_RISK' for r in f['risks'])), None)
    first_risk = first['predictor_time'] if first else None
    risk_flags = [any(r['risk']=='COLLISION_RISK' for r in f['risks']) for f in frames]
    # Post-hoc audit of isolated alarms, not a gate inside the online predictor.
    sustained = next((frames[i]['predictor_time'] for i in range(len(frames)-2)
                      if all(risk_flags[i:i+3])), None)
    first_ttc = next((r['predicted_ttc_s'] for r in first['risks'] if r['risk']=='COLLISION_RISK'), None) if first else None
    confidences = [c for f in frames for c in f['confidence']]
    result = dict(scenario=case, seed=seed,
                  detector=Path(summary['yolo']['checkpoint']).name,
                  ultralytics_version=summary['yolo'].get('ultralytics_version'),
                  device=summary['yolo'].get('device'),
                  inference_count=len(frames),
                  yolo_responses_per_wall_s=len(frames)/summary['wall_seconds'],
                  yolo_mean_ms=float(np.mean([f['yolo_ms'] for f in frames])) if frames else None,
                  yolo_p95_ms=float(np.percentile([f['yolo_ms'] for f in frames], 95)) if frames else None,
                  actual_collision=summary['actual_collision'],
                  predicted_collision=summary['predicted_collision'],
                  first_detection_time=summary['first_detection_time'],
                  first_risk_time=first_risk,
                  actual_collision_time=summary['actual_collision_time'],
                  lead_time=(summary['actual_collision_time']-first_risk
                             if summary['actual_collision_time'] is not None and first_risk is not None else None),
                  first_sustained_risk_time=sustained,
                  sustained_lead_time=(summary['actual_collision_time']-sustained
                                       if summary['actual_collision_time'] is not None and sustained is not None else None),
                  predicted_ttc_at_first_risk=first_ttc,
                  first_ttc_error_s=(first_ttc-(summary['actual_collision_time']-first_risk)
                                     if first_ttc is not None and summary['actual_collision_time'] is not None else None),
                  min_predicted_distance=(min((r['rollout_min_distance_m'] for f in frames
                                               for r in f['risks']), default=None)),
                  min_actual_gt_distance=summary['min_actual_gt_distance_m'],
                  position_mae=position_mae, position_p95=position_p95,
                  velocity_mae=velocity_mae, id_switches=switches,
                  track_fragmentation=sum(max(0, len(ids)-1) for ids in associated_ids),
                  duplicate_track_count=duplicates,
                  duplicate_detection_pairs=sum(overlapping_box_pairs(f['boxes']) for f in frames),
                  false_alarm=bool(summary['predicted_collision'] and not summary['actual_collision']),
                  frames=len(frames), detected_frames=observed,
                  detection_continuity=observed/len(frames) if frames else None,
                  missed_exposure_ratio=1-observed/len(frames) if frames else None,
                  mean_confidence=float(np.mean(confidences)) if confidences else None,
                  median_confidence=float(np.median(confidences)) if confidences else None,
                  exposure_to_predictor_p95_s=float(np.percentile(
                      [f['exposure_to_predictor_s'] for f in frames], 95)) if frames else None,
                  track_continuity=track_support/len(frames) if frames else None,
                  video=hud_video(folder, frames))
    (folder/'evaluation.json').write_text(json.dumps(result, indent=2))
    if first_risk_row:
        # Aggregate associations here are evaluation-only; online track IDs are untouched.
        (folder/'risk_by_track.json').write_text(json.dumps(risk_by_track, indent=2))
    if count > 1:
        person_risk = [dict(person=f'Person {i+1}', track_ids=set(),
                            collision_risk_frames=0, warning_frames=0,
                            associated_frames=0, first_collision_risk_time=None,
                            actual_min_distance_m=min(r['distances_m'][i] for r in truth))
                       for i in range(count)]
        for frame in frames:
            active = [t for t in frame['tracks'] if t['miss_count']==0]
            if not active:
                continue
            human_xy, _ = gt_at(truth, frame['exposure_time'], count)
            cost = np.linalg.norm(human_xy[:, None, :]-np.asarray([t['state'][:2] for t in active])[None, :, :], axis=2)
            rows_, cols_ = linear_sum_assignment(cost)
            by_track = {r['track_id']: r for r in frame['risks']}
            for i, j in zip(rows_, cols_):
                if cost[i, j] > 1.5:
                    continue
                person = person_risk[i]
                track_id = active[j]['track_id']
                person['track_ids'].add(track_id)
                person['associated_frames'] += 1
                risk = by_track.get(track_id, {}).get('risk', 'SAFE')
                person['warning_frames'] += risk == 'WARNING'
                person['collision_risk_frames'] += risk == 'COLLISION_RISK'
                if risk == 'COLLISION_RISK' and person['first_collision_risk_time'] is None:
                    person['first_collision_risk_time'] = frame['predictor_time']
        for person in person_risk:
            person['track_ids'] = sorted(person['track_ids'])
            person['actual_collision_proxy'] = person['actual_min_distance_m'] < summary['collision_threshold_m']
            person['observed_risk_level'] = ('COLLISION_RISK' if person['collision_risk_frames'] else
                                             'WARNING' if person['warning_frames'] else 'SAFE')
        (folder/'MULTI_PERSON_RISK_RANKING.json').write_text(json.dumps(person_risk, indent=2))
    plot_row = next((f for f in frames if any(r['cpa_collision'] for r in f['risks'])), first_risk_row)
    plot(folder, case, truth, estimates, plot_row, summary)
    return result


def plot(folder, case, truth, estimates, first_risk, summary):
    robot = np.asarray([r['robot_xy'] for r in truth])
    human = np.asarray([r['human_xy'] for r in truth])
    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    ax.plot(robot[:, 0], robot[:, 1], color='#193b5b', lw=2.2, label='Robot actual trajectory (evaluation)')
    palette = ('#b83d32', '#a1741c', '#2a8c80')
    for i, est in enumerate(estimates):
        ax.plot(human[:, i, 0], human[:, i, 1], '--', color=palette[i], lw=1.5,
                label=f'Human {i+1} GT (evaluation)')
        if est:
            values = np.asarray(est)
            ax.plot(values[:, 1], values[:, 2], color=palette[i], lw=1., alpha=.8,
                    label=f'Human {i+1} KF estimate')
    closest = min(truth, key=lambda row: min(row['distances_m']))
    who = int(np.argmin(closest['distances_m']))
    closest_robot = np.asarray(closest['robot_xy'])
    closest_human = np.asarray(closest['human_xy'][who])
    ax.plot([closest_robot[0], closest_human[0]], [closest_robot[1], closest_human[1]],
            ':', color='black', lw=1.3, label='Closest same-time separation')
    ax.scatter([closest_robot[0], closest_human[0]],
               [closest_robot[1], closest_human[1]], s=24, facecolors='white',
               edgecolors='black', zorder=4)
    if first_risk:
        risk = next(r for r in first_risk['risks'] if r['risk']=='COLLISION_RISK')
        track = next(t for t in first_risk['tracks'] if t['track_id']==risk['track_id'])
        t = np.arange(0, 5.01, .1)
        state = np.asarray(track['state'])
        path = state[:2][None, :]+t[:, None]*state[2:][None, :]
        ax.plot(path[:, 0], path[:, 1], ':', color='#7a3b8a', lw=1.8,
                label='CV prediction at shown risk frame')
        if risk['t_cpa_s'] is not None:
            point = state[:2]+state[2:]*risk['t_cpa_s']
            ax.scatter([point[0]], [point[1]], marker='x', s=90, color='#7a3b8a',
                       label='Predicted human CPA point')
    if summary['actual_collision_time'] is not None:
        moment = min(truth, key=lambda r: abs(r['time']-summary['actual_collision_time']))
        ax.scatter([moment['robot_xy'][0]], [moment['robot_xy'][1]], marker='*', s=140,
                   color='black', label='First collision proxy (evaluation)')
    ax.set(title=(case.replace('_', ' ').title()+
                  f"\nMinimum same-time separation: {summary['min_actual_gt_distance_m']:.2f} m "
                  f"(proxy threshold {summary['collision_threshold_m']:.2f} m)"),
           xlabel='World X (m)', ylabel='World Y (m)',
           xlim=(-1, 11), ylim=(-3, 3), aspect='equal')
    ax.grid(alpha=.2)
    ax.legend(loc='upper center', bbox_to_anchor=(.5, -.14), ncol=2, fontsize=8)
    fig.tight_layout()
    fig.savefig(folder/'TOP_DOWN_EVIDENCE.png', dpi=160, bbox_inches='tight')
    plt.close(fig)


def main(detector_label='YOLO11n'):
    rows = [r for case in CASES if (r := assess(case)) is not None]
    if not rows:
        raise SystemExit('No completed runs found')
    columns = [key for key in rows[0] if key != 'video']+['video']
    with (ROOT/'COLLISION_PREDICTION_RESULTS.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, columns)
        writer.writeheader(); writer.writerows(rows)
    single_rows = [r for r in rows if r['scenario'] != 'multi_person_collision']
    positives = [r for r in single_rows if r['actual_collision']]
    negatives = [r for r in single_rows if not r['actual_collision']]
    tp = sum(r['predicted_collision'] for r in positives)
    fp = sum(r['predicted_collision'] for r in negatives)
    fn = len(positives)-tp
    tn = len(negatives)-fp
    leads = [r['lead_time'] for r in positives if r['lead_time'] is not None and r['lead_time'] > 0]
    sustained_leads = [r['sustained_lead_time'] for r in positives
                       if r['sustained_lead_time'] is not None and r['sustained_lead_time'] > 0]
    failed_positives = [r['scenario'] for r in positives if not r['predicted_collision'] or
                        r['lead_time'] is None or r['lead_time'] <= 0]
    false_alarms = [r['scenario'] for r in negatives if r['predicted_collision']]
    lines = [
        '# Third-person pedestrian collision prediction', '',
        '## 1. Goal',
        'Test whether a fixed third-person RGB-D camera can predict robot–human collision proxies before they occur.', '',
        '## 2. System',
        f'Third-person RGB-D → official COCO {detector_label} person → torso robust depth → world XY → Hungarian + CV-KF → CPA/TTC and 5 s rollout. Robot ego state is known; human GT and scripted future routes are post-run evaluation only.', '',
        '## 3. Camera Setup',
        'One fixed `/World/CollisionPredictionCamera`, 640×360, 10 Hz simulation time; eye (5,−5,7) m, target (5,0,0) m; 18 mm focal length, 24 mm horizontal aperture (HFOV ≈ 67.38°), 13.5 mm vertical aperture. Same settings across runs. Exact camera intrinsics and view transform are in each `camera_params.json`.', '',
        '## 4. Collision Scenarios',
        f'{len(single_rows)} completed single-person presets plus {len(rows)-len(single_rows)} multi-person preset, seed 17; robot open-loop straight at 0.30 m/s, human prescribed paths. Collision proxy threshold is 0.903 m (0.603 m conservative robot radius + 0.30 m human radius). No physical-contact certification.', '',
        '## 5. Detection Results',
        'Visibility GT was not independently labeled; therefore true visible-person recall is **not reported**. '
        f"Single-person exposure detection continuity ranges from {100*min(r['detection_continuity'] for r in single_rows):.1f}% to {100*max(r['detection_continuity'] for r in single_rows):.1f}%; "
        'this includes time before a delayed person enters view and time after leaving. First detection and missed-exposure ratios are in the CSV.', '',
        '## 6. Tracking Results',
        'Post-run root-position and finite-difference velocity errors are in the CSV, computed only on active tracks that can be associated offline; misses are not imputed into MAE. KF estimates are evaluated, not fed Human GT. Identity continuity is imperfect: '
        f"single-run ID-switch counts range from {min(r['id_switches'] for r in single_rows)} to {max(r['id_switches'] for r in single_rows)}. "
        'In particular, cut-in and rear-end runs show track fragmentation; this experiment does not establish stable long-term identity.', '',
        '## 7. Collision Prediction Results',
        '| Scenario | Actual proxy | Predicted | First-alert lead (s) | 3-frame sustained lead (s) | Position MAE (m) | Velocity MAE (m/s) |',
        '|---|---:|---:|---:|---:|---:|---:|',
    ]
    for r in rows:
        fmt = lambda x: '—' if x is None else f'{x:.2f}'
        lines.append(f"| {r['scenario']} | {r['actual_collision']} | {r['predicted_collision']} | {fmt(r['lead_time'])} | {fmt(r['sustained_lead_time'])} | {fmt(r['position_mae'])} | {fmt(r['velocity_mae'])} |")
    lines += ['', f'Observed mechanism counts: TP={tp}, FP={fp}, FN={fn}, TN={tn}. These few prescribed runs are **not** a statistical safety benchmark.',
              f"Positive lead times: {', '.join(f'{v:.2f}' for v in leads) if leads else 'none'} s; "
              f"first-alert median = {float(np.median(leads)):.2f} s; "
              f"3-frame sustained median = {float(np.median(sustained_leads)):.2f} s."
              if leads and sustained_leads else 'No positive sustained advance warning recorded.',
              f"Single-person missed or late collision cases: {', '.join(failed_positives) if failed_positives else 'none'}. "
              f"Single-person run-level false-alarm cases: {', '.join(false_alarms) if false_alarms else 'none'}; "
              "the separate three-person run has one target-level false-alarm frame (Section 9).", '',
              'The 3-frame sustained-alarm onset is a **post-hoc diagnostic** added after observing an isolated early alarm; it is not an online gate or a pre-registered success threshold. First-alert lead can overstate useful warning, especially during KF initialization. `first_ttc_error_s` in the CSV compares first predicted TTC with the actual remaining time.', '',
              '## 8. Near-miss / False Alarms',
              'A near-miss is judged by measured post-run minimum separation, not by its scenario name. See `actual_collision` and `false_alarm` in the CSV.', '',
              '## 9. Multi-person Risk Ranking',
              ('The three-person run has per-person offline GT association and observed risk levels in `multi_person_collision/seed_17/MULTI_PERSON_RISK_RANKING.json`; online predictions never receive this association.'
               if any(r['scenario']=='multi_person_collision' for r in rows) else
               'Not run yet; no multi-person ranking claim.'), '',
              '## 10. Videos / Figures',
              'Every completed scenario has the original `THIRD_PERSON_COLLISION_PREDICTION.mp4`, a same-run exposure-aligned `THIRD_PERSON_COLLISION_PREDICTION_HUD.mp4` with d_CPA, and `TOP_DOWN_EVIDENCE.png`. Video HUD uses stored camera-based estimates only; top-down Human GT is explicitly evaluation-only.', '',
              '## 11. Limitations',
              'Simulation only; fixed third-person camera; CV extrapolation; prescribed trajectories; small scenario count; kinematic robot; conservative geometric collision proxy rather than physical-contact certification; no learned intent prediction. Exposure continuity is not visible-person recall.', '',
              '## 12. Conclusion',
              f'YOLO detected a person in {sum(r["detected_frames"]>0 for r in single_rows)}/{len(single_rows)} single-person runs. '
              f'Tracking position/velocity and warning lead times are reported per run above. '
              f'Among single-person cases, {fn} actual proxy run(s) lacked a positive prediction and {fp} non-collision run(s) had a collision-risk false alarm. '
              'Do not promote a general safety conclusion from these mechanism-level cases.', '']
    ranking_file = ROOT/'multi_person_collision'/'seed_17'/'MULTI_PERSON_RISK_RANKING.json'
    if ranking_file.exists() and any(r['scenario']=='multi_person_collision' for r in rows):
        ranking = json.loads(ranking_file.read_text())
        marker = lines.index('## 10. Videos / Figures')
        lines[marker:marker] = [
            '| Evaluation person | Actual proxy | Maximum online risk | Collision-risk frames | Associated frames |',
            '|---|---:|---|---:|---:|',
            *[f"| {p['person']} | {p['actual_collision_proxy']} | {p['observed_risk_level']} | {p['collision_risk_frames']} | {p['associated_frames']} |"
              for p in ranking], '',
        ]
        false_people = [p['person'] for p in ranking if p['collision_risk_frames'] and not p['actual_collision_proxy']]
        if false_people:
            lines[lines.index('## 10. Videos / Figures'):lines.index('## 10. Videos / Figures')] = [
                f"Target-level false alarm: {', '.join(false_people)} had at least one collision-risk frame without an actual proxy. The multi-person risk ranking is therefore **not fully correct**; track IDs also fragment and offline GT association is used only for this audit.", ''
            ]
    (ROOT/'THIRD_PERSON_COLLISION_PREDICTION_REPORT.md').write_text('\n'.join(lines), encoding='utf8')
    print(f'EVALUATED {len(rows)} runs TP={tp} FP={fp} FN={fn} TN={tn}')


def compare_detectors():
    """Compare frozen YOLO11n results without rewriting any baseline artifact."""
    old_root = Path(__file__).resolve().parent/'outputs'/'third_person_collision_prediction'
    new_root = Path(__file__).resolve().parent/'outputs'/'third_person_collision_prediction_yolo26n'
    with (old_root/'COLLISION_PREDICTION_RESULTS.csv').open(newline='') as handle:
        old_rows = {r['scenario']: r for r in csv.DictReader(handle)}
    with (new_root/'COLLISION_PREDICTION_RESULTS.csv').open(newline='') as handle:
        new_rows = {r['scenario']: r for r in csv.DictReader(handle)}
    if set(old_rows) != set(CASES) or set(new_rows) != set(CASES):
        raise RuntimeError('Both detectors must have all nine frozen scenarios')
    columns = ('scenario', 'detector', 'first_detection_time', 'detection_continuity',
               'missed_exposure_ratio', 'mean_confidence', 'median_confidence',
               'duplicate_detection_pairs', 'position_mae', 'position_p95',
               'velocity_mae', 'id_switches', 'track_fragmentation',
               'duplicate_track_count', 'track_continuity', 'actual_collision',
               'predicted_collision', 'first_risk_time', 'actual_collision_time',
               'lead_time', 'sustained_lead_time', 'predicted_ttc_at_first_risk',
               'min_predicted_distance', 'false_alarm', 'inference_count',
               'yolo_mean_ms', 'yolo_p95_ms', 'yolo_responses_per_wall_s')
    all_rows = []
    for case in CASES:
        old_summary = json.loads((old_root/case/'seed_17'/'summary.json').read_text())
        new_summary = json.loads((new_root/case/'seed_17'/'summary.json').read_text())
        for summary, checkpoint in ((old_summary, 'yolo11n.pt'), (new_summary, 'yolo26n.pt')):
            info = summary['yolo']
            if (Path(info['checkpoint']).name != checkpoint or info['person_class'] != 0 or
                    info['class_count'] != 80 or 'coco' not in info['training_data_metadata'].lower()):
                raise RuntimeError(f'Detector metadata mismatch: {case} {checkpoint}')
        if (new_summary['yolo']['model_task'] != 'detect' or
                new_summary['yolo']['device'] != 'cuda:0' or
                new_summary['yolo']['confidence'] != .25 or
                new_summary['yolo']['imgsz'] != 640):
            raise RuntimeError(f'YOLO26n inference configuration mismatch: {case}')
        for key in ('seed', 'camera', 'robot_speed_m_s', 'collision_threshold_m',
                    'warning_threshold_m', 'human_specs'):
            if old_summary[key] != new_summary[key]:
                raise RuntimeError(f'Frozen configuration differs: {case} {key}')
        if old_rows[case]['actual_collision'] != new_rows[case]['actual_collision']:
            raise RuntimeError(f'Actual collision changed: {case}')
        for label, root, row, summary in (('YOLO11n', old_root, old_rows[case], old_summary),
                                          ('YOLO26n', new_root, new_rows[case], new_summary)):
            folder = root/case/'seed_17'
            frames = json.loads((folder/'perception.json').read_text())
            truth = json.loads((folder/'evaluation_gt.json').read_text())
            if len(frames) != 349:
                raise RuntimeError(f'Exposure count differs: {case} {label}')
            ids = [set() for _ in summary['human_specs']]
            for frame in frames:
                active = [t for t in frame['tracks'] if t['miss_count']==0]
                if not active:
                    continue
                human_xy, _ = gt_at(truth, frame['exposure_time'], len(ids))
                cost = np.linalg.norm(human_xy[:, None, :]-
                                      np.asarray([t['state'][:2] for t in active])[None, :, :], axis=2)
                matched_people, matched_tracks = linear_sum_assignment(cost)
                for i, j in zip(matched_people, matched_tracks):
                    if cost[i, j] <= 1.5:
                        ids[i].add(active[j]['track_id'])
            confidence = [c for frame in frames for c in frame['confidence']]
            extras = dict(median_confidence=float(np.median(confidence)) if confidence else None,
                          duplicate_detection_pairs=sum(overlapping_box_pairs(f['boxes']) for f in frames),
                          track_fragmentation=sum(max(0, len(x)-1) for x in ids),
                          inference_count=len(frames),
                          yolo_mean_ms=float(np.mean([f['yolo_ms'] for f in frames])),
                          yolo_p95_ms=float(np.percentile([f['yolo_ms'] for f in frames], 95)),
                          yolo_responses_per_wall_s=len(frames)/summary['wall_seconds'])
            all_rows.append({key: ({**row, **extras, 'detector': label}.get(key)) for key in columns})
    with (new_root/'YOLO11N_VS_YOLO26N_COMPARISON.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, columns)
        writer.writeheader(); writer.writerows(all_rows)

    def f(value, digits=2):
        return '—' if value in ('', None) else f'{float(value):.{digits}f}'
    def m(label, key):
        return [float(r[key]) for r in all_rows if r['detector']==label and r[key] not in ('', None)]
    def med(label, key):
        values = m(label, key)
        return f(np.median(values)) if values else '—'
    def total(label, key):
        return sum(int(r[key]) for r in all_rows if r['detector']==label)
    def value(case, label, key):
        return next(r[key] for r in all_rows if r['scenario']==case and r['detector']==label)
    def positive_sustained_median(label):
        values = [float(r['sustained_lead_time']) for r in all_rows
                  if r['detector']==label and r['scenario'] in CASES[:5]
                  and r['sustained_lead_time'] not in ('', None)]
        return f(np.median(values)) if values else '—'
    continuity_wins = sum(float(new_rows[c]['detection_continuity']) >
                          float(old_rows[c]['detection_continuity']) for c in CASES)
    id_wins = sum(int(new_rows[c]['id_switches']) < int(old_rows[c]['id_switches']) for c in CASES)
    positives = CASES[:5]
    negatives = CASES[5:8]
    lines = ['# YOLO26n third-person collision prediction A/B', '',
             '## 1. Detector Change',
             'Only the official COCO detection checkpoint changed: YOLO11n → YOLO26n. '
             'Both resolve COCO person class 0. YOLO26n uses the installed Ultralytics 8.4.144 on CUDA.', '',
             '## 2. Frozen System',
             'Same fixed camera, 640×360 at 10 Hz, conf 0.25, imgsz 640, depth, Hungarian/CV-KF, '
             'CPA/TTC, 5 s rollout, thresholds, robot motion, scenarios and seed 17. '
             'The comparison script checked camera/robot/scenario parameters and 349 exposures for every pair. '
             'YOLO11n files were read-only. Visibility GT was not labeled, so exposure continuity is not visible-person recall.', '',
             '## 3. Detection Comparison',
             '| Scene | Continuity 11n / 26n | First detection 11n / 26n (s) | Mean confidence 11n / 26n |',
             '|---|---:|---:|---:|']
    for case in CASES:
        a, b = old_rows[case], new_rows[case]
        lines.append(f"| {case} | {f(a['detection_continuity'],3)} / {f(b['detection_continuity'],3)} | "
                     f"{f(a['first_detection_time'])} / {f(b['first_detection_time'])} | "
                     f"{f(a['mean_confidence'],3)} / {f(b['mean_confidence'],3)} |")
    lines += ['', f'Continuity improved in {continuity_wins}/9 scenes. '
              'Missed-exposure ratios, median confidence and high-IoU (>0.8) duplicate-box-pair proxies are in the CSV; '
              'a high-IoU pair is not automatically a GT-confirmed duplicate.', '',
              '## 4. Tracking Comparison',
              '| Scene | Position MAE 11n / 26n (m) | P95 11n / 26n (m) | Velocity MAE 11n / 26n (m/s) | ID switches 11n / 26n |',
              '|---|---:|---:|---:|---:|']
    for case in CASES:
        a, b = old_rows[case], new_rows[case]
        lines.append(f"| {case} | {f(a['position_mae'],3)} / {f(b['position_mae'],3)} | "
                     f"{f(a['position_p95'],3)} / {f(b['position_p95'],3)} | "
                     f"{f(a['velocity_mae'],3)} / {f(b['velocity_mae'],3)} | "
                     f"{a['id_switches']} / {b['id_switches']} |")
    lines += ['', f'ID-switch count fell in {id_wins}/9 scenes; total switches: '
              f"{total('YOLO11n','id_switches')} → {total('YOLO26n','id_switches')}. "
              'GT-associated unique-ID fragmentation, track continuity and duplicate-track frames are in the CSV; '
              'these offline associations never enter online inference. '
              f"Cut-in fragmentation: {value('cutin_collision','YOLO11n','track_fragmentation')} → "
              f"{value('cutin_collision','YOLO26n','track_fragmentation')}; "
              f"rear-end: {value('rear_end_collision','YOLO11n','track_fragmentation')} → "
              f"{value('rear_end_collision','YOLO26n','track_fragmentation')}.", '',
              '## 5. Collision Prediction Comparison',
              '| Scene | Actual | Predicted 11n / 26n | First lead 11n / 26n (s) | Sustained lead 11n / 26n (s) |',
              '|---|---:|---:|---:|---:|']
    for case in CASES:
        a, b = old_rows[case], new_rows[case]
        lines.append(f"| {case} | {a['actual_collision']} | {a['predicted_collision']} / {b['predicted_collision']} | "
                     f"{f(a['lead_time'])} / {f(b['lead_time'])} | "
                     f"{f(a['sustained_lead_time'])} / {f(b['sustained_lead_time'])} |")
    for label, rows in (('YOLO11n', old_rows), ('YOLO26n', new_rows)):
        tp = sum(rows[c]['predicted_collision']=='True' for c in positives)
        fp = sum(rows[c]['predicted_collision']=='True' for c in negatives)
        lines.append(f'{label}: single-person TP={tp}, FN={5-tp}, FP={fp}, TN={3-fp}; '
                     f'positive sustained-lead median={positive_sustained_median(label)} s.')
    near_frames = json.loads((new_root/'perpendicular_nearmiss'/'seed_17'/'perception.json').read_text())
    near_alarm_frames = sum(any(r['risk']=='COLLISION_RISK' for r in frame['risks']) for frame in near_frames)
    lines += ['', f'YOLO26n perpendicular near-miss false alarm lasted {near_alarm_frames} exposure frame(s). '
              'The online run-level decision is still a false positive; the post-hoc sustained-alarm diagnostic does not erase it.']
    old_ranking = json.loads((old_root/'multi_person_collision'/'seed_17'/'MULTI_PERSON_RISK_RANKING.json').read_text())
    new_ranking = json.loads((new_root/'multi_person_collision'/'seed_17'/'MULTI_PERSON_RISK_RANKING.json').read_text())
    lines += ['', '## 6. Multi-person Result',
              '| Person | Actual proxy | Collision-risk frames 11n / 26n | Associated frames 11n / 26n |',
              '|---|---:|---:|---:|']
    for a, b in zip(old_ranking, new_ranking):
        if a['person'] != b['person'] or a['actual_collision_proxy'] != b['actual_collision_proxy']:
            raise RuntimeError('Multi-person GT person alignment changed')
        lines.append(f"| {a['person']} | {a['actual_collision_proxy']} | "
                     f"{a['collision_risk_frames']} / {b['collision_risk_frames']} | "
                     f"{a['associated_frames']} / {b['associated_frames']} |")
    lines += ['', 'Per-person identity here comes from post-run GT association only. '
              'Online risk uses camera-derived tracks, not GT.', '',
              '## 7. Runtime',
              '| Metric (median across nine runs) | YOLO11n | YOLO26n |',
              '|---|---:|---:|',
              f'| YOLO response mean (ms) | {med("YOLO11n","yolo_mean_ms")} | {med("YOLO26n","yolo_mean_ms")} |',
              f'| YOLO response P95 (ms) | {med("YOLO11n","yolo_p95_ms")} | {med("YOLO26n","yolo_p95_ms")} |',
              f'| YOLO responses / wall-s | {med("YOLO11n","yolo_responses_per_wall_s")} | {med("YOLO26n","yolo_responses_per_wall_s")} |', '',
              'Responses/wall-s includes Isaac rendering and synchronous pipeline work; it is not detector-only FPS.', '',
              '## 8. Conclusion',
              f'1. Detection continuity: improved in {continuity_wins}/9 scenes; '
              f'median {med("YOLO11n","detection_continuity")} → {med("YOLO26n","detection_continuity")}.',
              f'2. ID switches / fragmentation: switches fell in {id_wins}/9 scenes and totaled '
              f'{total("YOLO11n","id_switches")} → {total("YOLO26n","id_switches")}; '
              f'unique-ID fragmentation totaled {total("YOLO11n","track_fragmentation")} → '
              f'{total("YOLO26n","track_fragmentation")}.',
              f'3. Collision prediction: both detectors warned in all five positive single-person runs; '
              f'YOLO26n introduced {near_alarm_frames} false-alarm frame(s) in one near-miss run '
              f'while Person 3 collision-risk frames fell from {old_ranking[2]["collision_risk_frames"]} '
              f'to {new_ranking[2]["collision_risk_frames"]}. This is mixed, not uniformly more stable.',
              'This single-seed, nine-scene mechanism test does not establish general detector superiority.']
    (new_root/'YOLO26N_COLLISION_PREDICTION_REPORT.md').write_text('\n'.join(lines), encoding='utf8')
    print(f'COMPARED {len(CASES)} frozen scenarios')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', type=Path, default=ROOT)
    parser.add_argument('--detector-label', default='YOLO11n')
    args = parser.parse_args()
    ROOT = args.output_root
    main(args.detector_label)
    if args.detector_label == 'YOLO26n':
        compare_detectors()
