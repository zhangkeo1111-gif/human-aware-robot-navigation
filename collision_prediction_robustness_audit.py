"""Offline-only robustness audit; GT never enters the online predictor."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linear_sum_assignment

from collision_prediction_analysis import gt_at

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'outputs' / 'third_person_collision_prediction_robustness'
BASE = ROOT / 'outputs' / 'third_person_collision_prediction_yolo26n_optimized'
SCENES = ('headon_collision', 'perpendicular_collision', 'diagonal_collision',
          'cutin_collision', 'rear_end_collision', 'perpendicular_nearmiss',
          'diagonal_nearmiss', 'cutin_nearmiss', 'multi_person_collision')
HARD = ('perpendicular_nearmiss_hard', 'diagonal_nearmiss_hard', 'cutin_nearmiss_hard')
DROP = ('headon_collision', 'cutin_collision', 'rear_end_collision',
        'perpendicular_nearmiss_hard')


def folder_for(phase, case, seed, rate):
    if phase == 'multiseed' and seed == 17:
        return BASE / case / 'seed_17', True
    if phase == 'dropout' and rate == 0:
        return folder_for('hard_nearmiss' if case in HARD else 'multiseed', case, seed, 0)[0], True
    path = OUT / phase / case / f'seed_{seed}'
    if phase == 'dropout':
        path /= f'rate_{round(rate*100):02d}_dropseed_101'
    return path, False


def audit_run(phase, case, seed, rate):
    folder, reused = folder_for(phase, case, seed, rate)
    if not (folder / 'summary.json').is_file():
        return None, []
    summary = json.loads((folder / 'summary.json').read_text())
    frames = json.loads((folder / 'perception.json').read_text())
    truth = json.loads((folder / 'evaluation_gt.json').read_text())
    baseline = json.loads((BASE / 'headon_collision' / 'seed_17' / 'summary.json').read_text())
    assert summary['scenario'] == case and summary['seed'] == seed
    assert Path(summary['yolo']['checkpoint']).name == 'yolo26n.pt'
    for key in ('camera', 'robot_speed_m_s', 'collision_threshold_m', 'warning_threshold_m'):
        assert summary[key] == baseline[key], (folder, key)
    assert summary['yolo']['ultralytics_version'] == baseline['yolo']['ultralytics_version']
    if case in SCENES:
        original = json.loads((BASE / case / 'seed_17' / 'summary.json').read_text())
        assert summary['human_specs'] == original['human_specs'], (folder, 'human_specs')
    if phase == 'dropout' and rate:
        assert summary['detection_dropout_rate'] == rate and summary['dropout_seed'] == 101
        assert all('raw_detector_boxes' in f and 'post_dropout_boxes' in f and
                   'dropout_applied' in f for f in frames)
        expected_flags = np.random.default_rng(101).random(len(frames)) < rate
        assert all(f['dropout_applied'] == bool(expected_flags[i]) for i, f in enumerate(frames))
        assert all(f['post_dropout_boxes'] == f['boxes'] and
                   (not f['dropout_applied'] or not f['boxes']) and
                   (f['dropout_applied'] or f['raw_detector_boxes'] == f['post_dropout_boxes'])
                   for f in frames)
    count = len(summary['human_specs'])
    ids, previous = [set() for _ in range(count)], [None] * count
    associated = [0] * count
    raw_risks = [0] * count
    confirmed_risks = [0] * count
    first_person_confirmed = [None] * count
    switches = duplicates = track_support_exposures = 0
    position_errors, velocity_errors = [], []
    cross_person = []
    previous_risk_people = set()
    for frame in frames:
        active = [t for t in frame['tracks'] if t['miss_count'] == 0]
        track_support_exposures += bool(active)
        duplicates += max(0, len(active) - count)
        hxy, hv = gt_at(truth, frame['exposure_time'], count)
        risk_ids = {r['track_id'] for r in frame['risks'] if r['risk'] == 'COLLISION_RISK'}
        risk_people = set()
        if active:
            cost = np.linalg.norm(hxy[:, None, :] - np.asarray([t['state'][:2] for t in active])[None, :, :], axis=2)
            people, columns = linear_sum_assignment(cost)
            for person, column in zip(people, columns):
                if cost[person, column] > 1.5:
                    continue
                track = active[column]
                identifier = track['track_id']
                associated[person] += 1
                position_errors.append(float(cost[person, column]))
                velocity_errors.append(float(np.linalg.norm(np.asarray(track['state'][2:]) - hv[person])))
                switches += previous[person] is not None and previous[person] != identifier
                previous[person] = identifier
                ids[person].add(identifier)
                if identifier in risk_ids:
                    risk_people.add(int(person))
                    raw_risks[person] += 1
                    if frame['confirmed_risk'] == 'CONFIRMED_COLLISION_RISK':
                        confirmed_risks[person] += 1
                        if first_person_confirmed[person] is None:
                            first_person_confirmed[person] = frame['predictor_time']
        if (frame['confirmed_risk'] == 'CONFIRMED_COLLISION_RISK' and risk_people and
                previous_risk_people and not risk_people.intersection(previous_risk_people)):
            cross_person.append(dict(frame_id=frame['frame_id'], time_s=frame['predictor_time'],
                                     previous_people=sorted(previous_risk_people),
                                     current_people=sorted(risk_people)))
        previous_risk_people = risk_people
    actual = bool(summary['actual_collision'])
    raw = bool(summary['predicted_collision'])
    confirmed = bool(summary['confirmed_predicted_collision'])
    mean = lambda key: float(np.mean([f[key] for f in frames])) if frames else None
    result = dict(phase=phase, scenario=case, sim_seed=seed, dropout_rate=rate,
                  dropout_seed=101 if phase == 'dropout' else '', reused_run=reused,
                  actual_collision=actual, raw_prediction=raw, confirmed_prediction=confirmed,
                  raw_first_alert_time=summary['first_risk_time'],
                  confirmed_first_alert_time=summary['first_confirmed_risk_time'],
                  actual_collision_time=summary['actual_collision_time'],
                  raw_lead=summary['lead_time_s'], confirmed_lead=summary['confirmed_lead_time_s'],
                  min_gt_distance=summary['min_actual_gt_distance_m'],
                  min_predicted_dcpa=min((r['d_cpa_m'] for f in frames for r in f['risks']
                                          if r['d_cpa_m'] is not None), default=None),
                  tcpa_at_first_raw=next((r['t_cpa_s'] for f in frames for r in f['risks']
                                          if r['risk'] == 'COLLISION_RISK'), None),
                  raw_risk_frames=sum(f['raw_risk'] == 'COLLISION_RISK' for f in frames),
                  confirmed_risk_frames=sum(f['confirmed_risk'] == 'CONFIRMED_COLLISION_RISK' for f in frames),
                  position_mae=float(np.mean(position_errors)) if position_errors else None,
                  velocity_mae=float(np.mean(velocity_errors)) if velocity_errors else None,
                  id_switches=int(switches), fragments=sum(max(0, len(x) - 1) for x in ids),
                  duplicate_tracks=duplicates, raw_fp=bool(raw and not actual),
                  confirmed_fp=bool(confirmed and not actual), fn=bool(actual and not confirmed),
                  yolo_ms=mean('yolo_ms'), tracking_ms=mean('association_tracking_ms'),
                  risk_ms=mean('risk_prediction_ms'), exposure_count=len(frames),
                  track_support_exposures=track_support_exposures,
                  track_support_rate=track_support_exposures / len(frames) if frames else None,
                  raw_boxes=sum(len(f.get('raw_detector_boxes', f['boxes'])) for f in frames),
                  post_dropout_boxes=sum(len(f['boxes']) for f in frames),
                  injected_exposures=sum(bool(f.get('dropout_applied', False)) for f in frames),
                  cross_person_confirmation_events=len(cross_person), source_folder=str(folder))
    people_rows = [dict(scenario=case, sim_seed=seed, person=f'Person {i+1}',
                        actual_collision=min(r['distances_m'][i] for r in truth) < summary['collision_threshold_m'],
                        raw_risk_frames=raw_risks[i], confirmed_risk_frames=confirmed_risks[i],
                        first_confirmed_risk=first_person_confirmed[i],
                        associated_frames=associated[i], unique_track_ids=len(ids[i]),
                        track_ids=json.dumps(sorted(ids[i]))) for i in range(count)]
    return result, (people_rows, cross_person)


def save_csv(path, rows):
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def scenario_summary(rows):
    metrics = ('confirmed_lead', 'position_mae', 'velocity_mae', 'id_switches',
               'fragments', 'duplicate_tracks', 'yolo_ms', 'tracking_ms', 'risk_ms')
    result = []
    for scene in SCENES:
        group = [r for r in rows if r['phase'] == 'multiseed' and r['scenario'] == scene]
        if len(group) != 3:
            continue
        item = dict(scenario=scene, runs=3,
                    actual_collisions=sum(r['actual_collision'] for r in group),
                    confirmed_alerts=sum(r['confirmed_prediction'] for r in group),
                    confirmed_fp=sum(r['confirmed_fp'] for r in group),
                    fn=sum(r['fn'] for r in group))
        for metric in metrics:
            values = [r[metric] for r in group if r[metric] is not None]
            for label, op in (('mean', np.mean), ('median', np.median), ('min', np.min), ('max', np.max)):
                item[f'{metric}_{label}'] = float(op(values)) if values else ''
        result.append(item)
    return result


def figures(rows):
    plots = OUT / 'figures'
    plots.mkdir(exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'savefig.dpi': 220})
    colors = {17: '#2468a0', 23: '#208b71', 31: '#bb6536'}
    multi = [r for r in rows if r['phase'] == 'multiseed']
    fig, ax = plt.subplots(figsize=(11, 4.5))
    lead_scenes = [s for s in SCENES if 'nearmiss' not in s]
    for j, seed in enumerate((17, 23, 31)):
        subset = {r['scenario']: r for r in multi if r['sim_seed'] == seed}
        ax.scatter(np.arange(len(lead_scenes)) + (j-1)*.10,
                   [subset[s]['confirmed_lead'] if subset[s]['confirmed_lead'] is not None else np.nan
                    for s in lead_scenes], label=f'Seed {seed}',
                   color=colors[seed], s=52)
    ax.set(ylabel='Confirmed warning lead (s)', title='Lead time across repeated simulator seeds')
    ax.set_xticks(np.arange(len(lead_scenes)), [s.replace('_collision', '') for s in lead_scenes])
    ax.tick_params(axis='x', rotation=20)
    ax.legend(frameon=False, ncol=3)
    fig.tight_layout(); fig.savefig(plots / 'LEAD_TIME_BY_SCENARIO_SEED.png'); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    for metric, ax, title in [('id_switches', axes[0], 'ID switches'),
                              ('fragments', axes[1], 'Unique-ID fragments')]:
        positions = np.arange(len(SCENES))
        for j, seed in enumerate((17, 23, 31)):
            lookup = {r['scenario']: r[metric] for r in multi if r['sim_seed'] == seed}
            ax.bar(positions + (j-1)*.25, [lookup.get(s, np.nan) for s in SCENES], .23,
                   color=colors[seed], edgecolor='black', linewidth=.4, label=f'Seed {seed}')
        ax.set(ylabel=title); ax.grid(axis='y', alpha=.2)
    axes[0].legend(frameon=False, ncol=3)
    axes[1].set_xticks(np.arange(len(SCENES)), [s.replace('_collision', '').replace('_nearmiss', ' NM') for s in SCENES], rotation=25)
    fig.tight_layout(); fig.savefig(plots / 'TRACKING_FRAGMENTATION.png'); plt.close(fig)
    drop = [r for r in rows if r['phase'] == 'dropout']
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8))
    for scene, color in zip(DROP, ('#2468a0', '#208b71', '#bb6536', '#7856a8')):
        subset = sorted((r for r in drop if r['scenario'] == scene), key=lambda x: x['dropout_rate'])
        x = [r['dropout_rate']*100 for r in subset]
        label = scene.replace('_collision', '').replace('_nearmiss_hard', ' hard NM')
        for ax, values in zip(axes, ([int(r['confirmed_prediction']) for r in subset],
                                     [r['confirmed_lead'] if r['confirmed_lead'] is not None else np.nan for r in subset],
                                     [r['fragments'] for r in subset])):
            ax.plot(x, values, '-o', color=color, label=label, markersize=4)
    for ax, title in zip(axes, ('Confirmed alert (0/1)', 'Lead time (s)', 'ID fragments')):
        ax.set(xlabel='Injected exposure dropout (%)', title=title, xticks=[0, 10, 20, 30])
        ax.grid(alpha=.2)
    axes[0].legend(frameon=False, fontsize=8)
    fig.tight_layout(); fig.savefig(plots / 'DROPOUT_SENSITIVITY.png'); plt.close(fig)


def report(rows, people, events):
    by_phase = {phase: [r for r in rows if r['phase'] == phase]
                for phase in ('multiseed', 'hard_nearmiss', 'dropout')}
    multi, hard, dropout = (by_phase[p] for p in by_phase)
    collision_cases = SCENES[:5]
    safe_cases = SCENES[5:8]
    f = lambda x, digits=2: '—' if x is None else f'{x:.{digits}f}'
    lines = [
        '# Third-person collision-prediction robustness evaluation', '',
        '## 1. Frozen Baseline', '',
        'The prediction algorithm is frozen at commit `5e6231285135e4cc6cf5c7c72eb7561a0f6f9f40` '
        '(see [FROZEN_PROTOCOL.md](FROZEN_PROTOCOL.md)). All runs use COCO YOLO26n, fixed third-person '
        'RGB-D, duplicate-aware Hungarian CV-KF, 5 s CPA/rollout, and two-consecutive-exposure system '
        'confirmation. The only changes are the predeclared scene delays and the experimental post-YOLO '
        'dropout injector. GT is used offline only.', '',
        '## 2. Multi-seed Setup', '',
        'Nine original scenarios × simulator seeds 17, 23, 31 = 27 runs. The original seed-17 optimized '
        'runs were reused; no best-run selection. These are mechanism-level repeats, not a statistical '
        'safety benchmark. Per-scenario mean, median, min, and max for continuous metrics are in '
        '`MULTISEED_SCENARIO_SUMMARY.csv`.', '',
        '## 3. Multi-seed Results', '',
        '| Scenario | Confirmed alerts / 3 | Actual collisions / 3 | Confirmed FP / 3 | '
        'Lead mean / median / min / max (s) |',
        '| --- | ---: | ---: | ---: | --- |'
    ]
    for scene in SCENES:
        group = [r for r in multi if r['scenario'] == scene]
        leads = [r['confirmed_lead'] for r in group if r['confirmed_lead'] is not None]
        lead_summary = (' / '.join(f(x) for x in (np.mean(leads), np.median(leads), min(leads), max(leads)))
                        if leads else '—')
        lines.append(f'| {scene} | {sum(r["confirmed_prediction"] for r in group)}/3 | '
                     f'{sum(r["actual_collision"] for r in group)}/3 | '
                     f'{sum(r["confirmed_fp"] for r in group)}/3 | {lead_summary} |')
    collision_rows = [r for r in multi if r['scenario'] in collision_cases]
    safe_rows = [r for r in multi if r['scenario'] in safe_cases]
    lines += ['', f'Five single-person collision classes: {sum(r["confirmed_prediction"] for r in collision_rows)}/15 '
              f'confirmed alerts, {sum(r["fn"] for r in collision_rows)} FN. '
              f'Three original near-miss classes: {sum(r["confirmed_fp"] for r in safe_rows)}/9 '
              'confirmed false alarms. Multi-person is audited separately.', '',
              '![Confirmed lead by scenario and seed](figures/LEAD_TIME_BY_SCENARIO_SEED.png)', '',
              '## 4. Hard Near-miss Design', '',
              'Only human start delays differ; paths, speeds, robot motion, camera, collision radius, and '
              'risk logic remain fixed. Delay selection was made before any hard-case predictor run and '
              'is documented in `FROZEN_PROTOCOL.md`. The diagonal and cut-in changes exceed the suggested '
              '0.3–0.8 s example because their original closest distances were too large for a hard case.', '',
              '## 5. Hard Near-miss Results', '',
              '| Case | Actual min distance (m) | Actual collision | Raw-risk frames | Confirmed-risk frames | '
              'Min finite dCPA (m) | First-risk tCPA (s) |',
              '| --- | ---: | --- | ---: | ---: | ---: | ---: |']
    for r in hard:
        lines.append(f'| {r["scenario"]} | {f(r["min_gt_distance"], 3)} | {r["actual_collision"]} | '
                     f'{r["raw_risk_frames"]} | {r["confirmed_risk_frames"]} | '
                     f'{f(r["min_predicted_dcpa"], 3)} | {f(r["tcpa_at_first_raw"])} |')
    lines += ['', 'A confirmed alert when the actual minimum distance exceeds 0.903 m is an observed '
              'false alarm; it is not corrected post hoc. dCPA is reported only when the analytic CPA '
              'time lies inside the 5 s horizon, while the collision flag comes from the separate '
              'discrete rollout; therefore a raw risk can have no finite reported tCPA.', '',
              '## 6. Detection Dropout Setup', '',
              'Independent seed 101 deletes the entire person-detection set at each selected exposure '
              'after YOLO and before depth projection/tracking. Rates: 0%, 10%, 20%, 30%; one simulator '
              'seed (17), four scenes. Raw YOLO boxes and post-dropout boxes are both recorded. '
              '0% references reuse matching runs.', '',
              '## 7. Dropout Results', '',
              '| Scene | Dropout | Injected exposures | Confirmed alert | Confirmed FP | FN | '
              'Lead (s) | Track support | Position MAE (m) | Velocity MAE (m/s) | ID switches | Fragments |',
              '| --- | ---: | ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for r in dropout:
        lines.append(f'| {r["scenario"]} | {r["dropout_rate"]:.0%} | {r["injected_exposures"]} | '
                     f'{r["confirmed_prediction"]} | {r["confirmed_fp"]} | {r["fn"]} | '
                     f'{f(r["confirmed_lead"])} | {r["track_support_rate"]:.1%} | '
                     f'{f(r["position_mae"], 3)} | {f(r["velocity_mae"], 3)} | '
                     f'{r["id_switches"]} | {r["fragments"]} |')
    lines.append('')
    for scene in DROP:
        subset = sorted((r for r in dropout if r['scenario'] == scene), key=lambda x: x['dropout_rate'])
        first_failure = next((r['dropout_rate'] for r in subset if r['fn'] or r['confirmed_fp']), None)
        lines.append(f'- {scene}: first observed confirmed FP/FN at '
                     f'{first_failure:.0%} injected dropout.' if first_failure is not None else
                     f'- {scene}: no confirmed FP/FN through the tested 30% rate.')
    lines += ['', 'No collision-recall failure was observed at the tested rates through 30%, so a recall '
              'failure boundary was not located. The hard-near-miss false alert disappears at 20–30% '
              'in this single dropout-seed experiment because risk observations are removed; that is '
              'not evidence that missing detections improve the system.', '']
    lines += ['', '![Dropout sensitivity](figures/DROPOUT_SENSITIVITY.png)', '',
              '## 8. Multi-person Stability', '',
              '| Seed | Person | Actual collision | Raw-risk frames | Confirmed-risk frames | '
              'First confirmed (s) | Associated frames | Unique IDs |',
              '| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: |']
    for p in people:
        lines.append(f'| {p["sim_seed"]} | {p["person"]} | {p["actual_collision"]} | '
                     f'{p["raw_risk_frames"]} | {p["confirmed_risk_frames"]} | '
                     f'{f(p["first_confirmed_risk"])} | {p["associated_frames"]} | '
                     f'{p["unique_track_ids"]} |')
    ranking = {seed: ' > '.join(p['person'] for p in sorted(
        (x for x in people if x['sim_seed'] == seed),
        key=lambda x: (-x['raw_risk_frames'], x['person']))) for seed in (17, 23, 31)}
    lines += ['', f'Cross-person confirmation events detected with unique GT-assigned risk identities: '
              f'**{len(events)}**. See `multi_person_audit/CROSS_PERSON_CONFIRMATION_EVENTS.json`. '
              'Events involving unassociated tracks are not provably attributable to a person. '
              'Raw-risk-frame ranking by seed: ' + '; '.join(f'{seed}: {rank}' for seed, rank in ranking.items()) + '.', '',
              '## 9. Tracking Fragmentation', '',
              f'Total multi-seed ID switches: {sum(r["id_switches"] for r in multi)}; unique-ID fragments: '
              f'{sum(r["fragments"] for r in multi)}. Rear-end fragments by seed: '
              + ', '.join(f'{r["sim_seed"]}: {r["fragments"]}' for r in multi if r['scenario'] == 'rear_end_collision')
              + '.', '', '![Tracking fragmentation](figures/TRACKING_FRAGMENTATION.png)', '',
              '## 10. Warning Lead Time', '',
              'Lead is actual first collision-proxy time minus first confirmed alert time. It is undefined '
              'for near-misses and missed collisions. A positive lead does not prove calibrated future '
              'trajectory accuracy.', '',
              '## 11. Runtime', '',
              '| Phase | YOLO mean (ms) | Tracking mean (ms) | Risk mean (ms) |',
              '| --- | ---: | ---: | ---: |']
    for phase, group in by_phase.items():
        lines.append(f'| {phase} | {f(np.mean([r["yolo_ms"] for r in group]))} | '
                     f'{f(np.mean([r["tracking_ms"] for r in group]))} | '
                     f'{f(np.mean([r["risk_ms"] for r in group]))} |')
    failures = [r for r in rows if r['confirmed_fp'] or r['fn'] or
                (r['phase'] == 'hard_nearmiss' and (r['actual_collision'] or not 1. <= r['min_gt_distance'] <= 1.3))]
    dropout_collisions = [r for r in dropout if r['actual_collision']]
    hard_false_alarms = sum(r['confirmed_fp'] for r in hard)
    lines += ['', '## 12. Failure Cases', '',
              f'{len(failures)} run rows satisfy a failure/invalid-hard-geometry condition.', '']
    for r in failures:
        lines.append(f'- {r["phase"]} / {r["scenario"]} / seed {r["sim_seed"]} / '
                     f'dropout {r["dropout_rate"]:.0%}: confirmed FP={r["confirmed_fp"]}, '
                     f'FN={r["fn"]}, actual collision={r["actual_collision"]}, '
                     f'min GT distance={f(r["min_gt_distance"], 3)} m.')
    lines += ['', '## 13. Limitations', '',
              'Three seeds and one dropout seed do not establish a safety bound. Injector drops whole '
              'exposures independently, not burst losses or selective person omissions. The fixed camera, '
              'scripted paths, collision proxy, and simulator-to-wall-time distinction limit generalization. '
              'Repeated simulator runs can show small human-trajectory and raw-YOLO count differences '
              'even at the same seed; logged GT minimum distances and raw box counts make this visible. '
              'Dropout comparisons are therefore not an exact paired replay of one detector stream. '
              'Position and velocity MAE are conditional on GT-associated active tracks; missing frames '
              'do not contribute to those errors, so inspect track-support rate alongside them. '
              'The system-level two-frame rule is not inherently person-specific. Offline GT assignment '
              'uses a 1.5 m gate and can leave ambiguous tracks unassigned.', '',
              '## 14. Conclusion', '',
              f'The frozen baseline produced {sum(r["confirmed_prediction"] for r in collision_rows)}/15 '
              f'confirmed single-person collision alerts across seeds, while {hard_false_alarms}/3 frozen '
              f'hard near-misses produced confirmed false alarms. In the three collision scenes under '
              f'tested dropout rates, {sum(r["fn"] for r in dropout_collisions)}/{len(dropout_collisions)} '
              'runs were confirmed misses. These are observed mechanism-level outcomes, not a safety '
              'guarantee. The hard-near-miss false alarms justify a *future controlled comparison* '
              'against a learned short-horizon predictor, but this evaluation does not introduce one '
              'or establish that it would be better.', '']
    (OUT / 'THIRD_PERSON_COLLISION_PREDICTION_ROBUSTNESS_REPORT.md').write_text('\n'.join(lines), encoding='utf-8')


def main():
    requests = [('multiseed', s, seed, 0.) for s in SCENES for seed in (17, 23, 31)]
    requests += [('hard_nearmiss', s, 17, 0.) for s in HARD]
    requests += [('dropout', s, 17, rate) for s in DROP for rate in (0., .1, .2, .3)]
    results, people, events, missing = [], [], [], []
    for phase, scene, seed, rate in requests:
        row, extra = audit_run(phase, scene, seed, rate)
        if row is None:
            missing.append(f'{phase}/{scene}/seed_{seed}/rate_{rate:g}')
            continue
        results.append(row)
        if phase == 'multiseed' and scene == 'multi_person_collision':
            person_rows, cross = extra
            people.extend(person_rows)
            events.extend(dict(sim_seed=seed, **event) for event in cross)
    save_csv(OUT / 'ROBUSTNESS_RESULTS.csv', results)
    save_csv(OUT / 'MULTISEED_SCENARIO_SUMMARY.csv', scenario_summary(results))
    save_csv(OUT / 'multi_person_audit' / 'PER_PERSON_AUDIT.csv', people)
    (OUT / 'multi_person_audit').mkdir(exist_ok=True)
    (OUT / 'multi_person_audit' / 'CROSS_PERSON_CONFIRMATION_EVENTS.json').write_text(json.dumps(events, indent=2))
    (OUT / 'RUN_COVERAGE.json').write_text(json.dumps(dict(expected=len(requests), completed=len(results), missing=missing), indent=2))
    if not missing:
        figures(results)
        report(results, people, events)
    print(f'ROBUSTNESS_AUDIT {len(results)}/{len(requests)} runs; missing={len(missing)}')
    for item in missing:
        print('MISSING', item)


if __name__ == '__main__':
    main()
