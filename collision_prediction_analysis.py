"""Post-run GT evaluation only; never imported by the online predictor."""
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
    result = dict(scenario=case, seed=seed,
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
                  duplicate_track_count=duplicates,
                  false_alarm=bool(summary['predicted_collision'] and not summary['actual_collision']),
                  frames=len(frames), detected_frames=observed,
                  detection_continuity=observed/len(frames) if frames else None,
                  missed_exposure_ratio=1-observed/len(frames) if frames else None,
                  mean_confidence=float(np.mean([c for f in frames for c in f['confidence']]))
                                  if any(f['confidence'] for f in frames) else None,
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


def main():
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
        'Third-person RGB-D → official COCO YOLO11n person → torso robust depth → world XY → Hungarian + CV-KF → CPA/TTC and 5 s rollout. Robot ego state is known; human GT and scripted future routes are post-run evaluation only.', '',
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


if __name__ == '__main__':
    main()
