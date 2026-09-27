"""Replay saved exposure measurements for a read-only tracker audit.

The replay uses the frozen tracker config and never feeds evaluation GT to it.
"""
import csv
import json
import argparse
from pathlib import Path

import numpy as np

from tracker import Tracker


ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'outputs' / 'third_person_collision_prediction_yolo26n_optimized'
FIELDS = ('detector', 'frame_id', 'exposure_time_s', 'observation_index',
          'bbox', 'bbox_cx_px', 'bbox_cy_px', 'bbox_w_px', 'bbox_h_px',
          'confidence', 'torso_depth_m', 'depth_accepted', 'world_x_m',
          'world_y_m', 'nearest_prior_track_id', 'nearest_prior_cost_m',
          'nearest_prior_pred_x_m', 'nearest_prior_pred_y_m',
          'nearest_prior_age', 'assigned_prior_track_id',
          'association_cost_m', 'innovation_norm_m', 'mahalanobis_squared',
          'association_outcome', 'track_birth_ids', 'track_death_ids',
          'active_track_ids', 'nearby_active_track_pairs', 'max_bbox_iou')


def bbox_iou(a, b):
    xa, ya = max(a[0], b[0]), max(a[1], b[1])
    xb, yb = min(a[2], b[2]), min(a[3], b[3])
    overlap = max(0., xb-xa) * max(0., yb-ya)
    area_a = max(0., a[2]-a[0]) * max(0., a[3]-a[1])
    area_b = max(0., b[2]-b[0]) * max(0., b[3]-b[1])
    return overlap / max(area_a+area_b-overlap, 1e-9)


def replay(detector, folder, config):
    frames = json.loads((folder/'perception.json').read_text())
    tracker = Tracker(config)
    rows = []
    for frame in frames:
        obs = frame['observations']
        accepted = [(j, np.asarray(o['estimated_world_xyz'][:2], dtype=float), o['confidence'])
                    for j, o in enumerate(obs) if 'estimated_world_xyz' in o]
        measurements = [(p, c) for _, p, c in accepted]
        dt = 0. if tracker.time is None else frame['exposure_time']-tracker.time
        prior = tracker.tracks[:]
        predicted = [(t['track_id'], t['state'][:2]+t['state'][2:]*dt,
                      t['covariance'][:2, :2]+dt*(t['covariance'][:2, 2:]+t['covariance'][2:, :2])+
                      dt*dt*t['covariance'][2:, 2:]+np.eye(2)*config['acceleration_sigma_m_s2']**2*dt**4/4,
                      t['age']+1) for t in prior]
        costs = np.asarray([[np.linalg.norm(pos-p) for _, p, _ in accepted]
                            for _, pos, _, _ in predicted]) if predicted and accepted else np.empty((len(prior), len(accepted)))
        assignments = {}
        if predicted and accepted:
            from scipy.optimize import linear_sum_assignment
            gate = config['association_gate_m']
            padded = np.column_stack((np.where(costs <= gate, costs, 1e6),
                                      np.full((len(prior), len(prior)), gate+1e-6)))
            rr, cc = linear_sum_assignment(padded)
            assignments = {int(c): int(r) for r, c in zip(rr, cc) if c < len(accepted) and costs[r, c] <= gate}
        old_ids = {t['track_id'] for t in prior}
        got = tracker.update(frame['exposure_time'], measurements)
        new_ids = {t['track_id'] for t in tracker.tracks}
        if new_ids != {t['track_id'] for t in frame['tracks']}:
            raise RuntimeError(f'Replay ID mismatch: {detector} frame {frame["frame_id"]}')
        active = [t for t in got if t['miss_count'] == 0]
        nearby = sum(np.linalg.norm(np.asarray(a['state'][:2])-b['state'][:2]) < .15
                     for i, a in enumerate(active) for b in active[i+1:])
        for j, o in enumerate(obs or [{}]):
            box = o.get('bbox')
            accepted_index = next((k for k, (original, _, _) in enumerate(accepted) if original == j), None)
            nearest = min(range(len(predicted)), key=lambda i: costs[i, accepted_index]) if predicted and accepted_index is not None else None
            assigned = assignments.get(accepted_index)
            diagnostic = o.get('depth_diagnostic', {})
            row = dict(detector=detector, frame_id=frame['frame_id'],
                       exposure_time_s=frame['exposure_time'], observation_index=j if box else '',
                       bbox=json.dumps(box) if box else '',
                       bbox_cx_px=(box[0]+box[2])/2 if box else '',
                       bbox_cy_px=(box[1]+box[3])/2 if box else '',
                       bbox_w_px=box[2]-box[0] if box else '',
                       bbox_h_px=box[3]-box[1] if box else '',
                       confidence=o.get('confidence', ''), torso_depth_m=diagnostic.get('depth_m', ''),
                       depth_accepted=diagnostic.get('accepted', ''),
                       world_x_m=o.get('estimated_world_xyz', ['', ''])[0],
                       world_y_m=o.get('estimated_world_xyz', ['', ''])[1],
                       nearest_prior_track_id=predicted[nearest][0] if nearest is not None else '',
                       nearest_prior_cost_m=costs[nearest, accepted_index] if nearest is not None else '',
                       nearest_prior_pred_x_m=predicted[nearest][1][0] if nearest is not None else '',
                       nearest_prior_pred_y_m=predicted[nearest][1][1] if nearest is not None else '',
                       nearest_prior_age=predicted[nearest][3] if nearest is not None else '',
                       assigned_prior_track_id=predicted[assigned][0] if assigned is not None else '',
                       association_cost_m=costs[assigned, accepted_index] if assigned is not None else '',
                       innovation_norm_m=costs[assigned, accepted_index] if assigned is not None else '',
                       mahalanobis_squared=(float((accepted[accepted_index][1]-predicted[assigned][1]) @
                           np.linalg.solve(predicted[assigned][2]+np.eye(2)*config['measurement_sigma_m']**2,
                                           accepted[accepted_index][1]-predicted[assigned][1]))
                           if assigned is not None else ''),
                       association_outcome=('no_depth' if accepted_index is None else
                                            'new_birth' if assigned is None else
                                            'innovation_rejected' if any(r['measurement_index'] == accepted_index for r in tracker.rejected)
                                            else 'matched'),
                       track_birth_ids=json.dumps(sorted(new_ids-old_ids)),
                       track_death_ids=json.dumps(sorted(old_ids-new_ids)),
                       active_track_ids=json.dumps([t['track_id'] for t in active]),
                       nearby_active_track_pairs=nearby,
                       max_bbox_iou=max((bbox_iou(box, other['bbox']) for k, other in enumerate(obs)
                                         if box and k != j), default=''))
            rows.append(row)
    return rows


def main():
    config = json.loads((ROOT/'config.yaml').read_text())['tracking']
    OUT.mkdir(parents=True, exist_ok=True)
    bases = [('yolo11n', ROOT/'outputs'/'third_person_collision_prediction'),
             ('yolo26n', ROOT/'outputs'/'third_person_collision_prediction_yolo26n')]
    rows = [row for label, base in bases
            for row in replay(label, base/'rear_end_collision'/'seed_17', config)]
    with (OUT/'REAR_END_TRACK_FRAGMENTATION_AUDIT.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f'Wrote {len(rows)} rear-end observation/exposure rows')


def tracker_stats(frames, truth, count, key):
    from scipy.optimize import linear_sum_assignment
    from collision_prediction_analysis import gt_at
    identifiers = [set() for _ in range(count)]
    previous = [None]*count
    switches = duplicates = 0
    for frame in frames:
        active = [t for t in frame[key] if t['miss_count'] == 0]
        duplicates += max(0, len(active)-count)
        if not active:
            continue
        actual, _ = gt_at(truth, frame['exposure_time'], count)
        costs = np.linalg.norm(actual[:, None, :]-
                               np.asarray([t['state'][:2] for t in active])[None, :, :], axis=2)
        people, track_indices = linear_sum_assignment(costs)
        for person, j in zip(people, track_indices):
            if costs[person, j] > 1.5:
                continue
            identifier = active[j]['track_id']
            switches += previous[person] is not None and previous[person] != identifier
            previous[person] = identifier
            identifiers[person].add(identifier)
    return switches, sum(max(0, len(ids)-1) for ids in identifiers), duplicates


def compare():
    import collision_prediction_analysis as analysis
    analysis.ROOT = OUT
    original = ROOT/'outputs'/'third_person_collision_prediction_yolo26n'
    columns = ('scenario', 'actual_collision', 'id_switches_before',
               'id_switches_same_run_raw', 'id_switches_after', 'fragments_before',
               'fragments_same_run_raw', 'fragments_after',
               'duplicate_tracks_before', 'duplicate_tracks_same_run_raw',
               'duplicate_tracks_after', 'raw_predicted_collision_before',
               'raw_predicted_collision_after', 'confirmed_predicted_collision',
               'raw_first_alert_lead_before_s', 'raw_first_alert_lead_after_s',
               'confirmed_first_alert_lead_s', 'confirmation_delay_s',
               'position_mae_before_m', 'position_mae_after_m',
               'velocity_mae_before_m_s', 'velocity_mae_after_m_s',
               'false_alarm_raw_before', 'false_alarm_raw_after',
               'false_alarm_confirmed', 'suppressed_duplicate_measurements',
               'yolo_inference_mean_before_ms', 'yolo_inference_mean_after_ms',
               'yolo_response_mean_after_ms', 'association_tracking_mean_after_ms',
               'risk_prediction_mean_after_ms', 'overall_processing_mean_after_ms',
               'overall_processing_p95_after_ms')
    rows = []
    for case in analysis.CASES:
        folder = OUT/case/'seed_17'
        if not (folder/'summary.json').exists():
            raise RuntimeError(f'Missing optimized run: {case}')
        after = analysis.assess(case)
        before = json.loads((original/case/'seed_17'/'evaluation.json').read_text())
        summary = json.loads((folder/'summary.json').read_text())
        original_summary = json.loads((original/case/'seed_17'/'summary.json').read_text())
        if (summary['seed'] != 17 or summary['actual_collision'] != original_summary['actual_collision'] or
                summary['camera'] != original_summary['camera'] or
                summary['collision_threshold_m'] != original_summary['collision_threshold_m'] or
                summary['warning_threshold_m'] != original_summary['warning_threshold_m'] or
                summary['robot_speed_m_s'] != original_summary['robot_speed_m_s'] or
                summary['human_specs'] != original_summary['human_specs'] or
                Path(summary['yolo']['checkpoint']).name != 'yolo26n.pt'):
            raise RuntimeError(f'Frozen scene/detector mismatch: {case}')
        frames = json.loads((folder/'perception.json').read_text())
        truth = json.loads((folder/'evaluation_gt.json').read_text())
        if len(frames) != 349 or len(frames) != before['frames']:
            raise RuntimeError(f'Exposure count differs: {case}')
        raw_stats = tracker_stats(frames, truth, len(summary['human_specs']), 'raw_tracks')
        final_stats = tracker_stats(frames, truth, len(summary['human_specs']), 'tracks')
        if final_stats != (after['id_switches'], after['track_fragmentation'], after['duplicate_track_count']):
            raise RuntimeError(f'Evaluation mismatch: {case}')
        if any(f['raw_risk'] == 'COLLISION_RISK' for f in frames) != summary['predicted_collision']:
            raise RuntimeError(f'Raw risk mismatch: {case}')
        if any(f['confirmed_risk'] == 'CONFIRMED_COLLISION_RISK' for f in frames) != summary['confirmed_predicted_collision']:
            raise RuntimeError(f'Confirmed risk mismatch: {case}')
        mean = lambda key: float(np.mean([f[key] for f in frames]))
        row = dict(scenario=case, actual_collision=summary['actual_collision'],
                   id_switches_before=before['id_switches'],
                   id_switches_same_run_raw=raw_stats[0], id_switches_after=final_stats[0],
                   fragments_before=before['track_fragmentation'],
                   fragments_same_run_raw=raw_stats[1], fragments_after=final_stats[1],
                   duplicate_tracks_before=before['duplicate_track_count'],
                   duplicate_tracks_same_run_raw=raw_stats[2], duplicate_tracks_after=final_stats[2],
                   raw_predicted_collision_before=before['predicted_collision'],
                   raw_predicted_collision_after=summary['predicted_collision'],
                   confirmed_predicted_collision=summary['confirmed_predicted_collision'],
                   raw_first_alert_lead_before_s=before['lead_time'],
                   raw_first_alert_lead_after_s=after['lead_time'],
                   confirmed_first_alert_lead_s=summary['confirmed_lead_time_s'],
                   confirmation_delay_s=(summary['first_confirmed_risk_time']-summary['first_risk_time']
                                         if summary['first_confirmed_risk_time'] is not None and
                                         summary['first_risk_time'] is not None else None),
                   position_mae_before_m=before['position_mae'],
                   position_mae_after_m=after['position_mae'],
                   velocity_mae_before_m_s=before['velocity_mae'],
                   velocity_mae_after_m_s=after['velocity_mae'],
                   false_alarm_raw_before=before['false_alarm'],
                   false_alarm_raw_after=after['false_alarm'],
                   false_alarm_confirmed=bool(summary['confirmed_predicted_collision'] and
                                              not summary['actual_collision']),
                   suppressed_duplicate_measurements=summary['suppressed_duplicate_measurements'],
                   yolo_inference_mean_before_ms=before['yolo_mean_ms'],
                   yolo_inference_mean_after_ms=after['yolo_mean_ms'],
                   yolo_response_mean_after_ms=mean('yolo_response_ms'),
                   association_tracking_mean_after_ms=mean('association_tracking_ms'),
                   risk_prediction_mean_after_ms=mean('risk_prediction_ms'),
                   overall_processing_mean_after_ms=mean('overall_processing_ms'),
                   overall_processing_p95_after_ms=float(np.percentile(
                       [f['overall_processing_ms'] for f in frames], 95)))
        rows.append(row)
    with (OUT/'YOLO26N_BEFORE_AFTER_OPTIMIZATION.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    print('Compared', len(rows), 'optimized single-run scenarios')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--compare', action='store_true')
    args = parser.parse_args()
    compare() if args.compare else main()
