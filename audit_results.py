"""Paired saved-frame depth audit. GT is read here for evaluation only."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from tracker import person_depth


def stats(values):
    a = np.asarray(values,dtype=float)
    return {'n':len(a),'mean':float(a.mean()),'median':float(np.median(a)),
            'p95':float(np.percentile(a,95)),'max':float(a.max())} if len(a) else {'n':0}


def tracking_audit(folder):
    detections = json.loads((folder/'detections.json').read_text())
    frames = [d for d in detections if 'evaluation_gt_xyz' in d]
    times = np.array([d['sim_time'] for d in frames])
    gt = np.array([d['evaluation_gt_xyz'][:2] for d in frames])
    velocity = np.gradient(gt,times,axis=0)
    pe,ve,ids = [],[],[]
    missing,extra = 0,0
    for d,g,v in zip(frames,gt,velocity):
        ts = d.get('tracks',[])
        if not ts:
            missing += 1
            continue
        # Single-person evaluation: closest state, no distance gate; extra tracks explicit.
        t = min(ts,key=lambda t:np.linalg.norm(np.asarray(t['state'][:2])-g))
        pe.append(float(np.linalg.norm(np.asarray(t['state'][:2])-g)))
        ve.append(float(np.linalg.norm(np.asarray(t['state'][2:])-v)))
        ids.append(t['track_id'])
        extra += len(ts)-1
    raw = json.loads((folder/'position_evaluation.json').read_text())
    report = {'gt_slots_all_camera_frames':len(frames),'missing_track_slots':missing,'extra_track_rows':extra,
        'position_available_no_distance_gate':stats(pe),'velocity_available_no_distance_gate':stats(ve),
        'id_transitions_available_sequence':sum(a!=b for a,b in zip(ids,ids[1:])),
        'all_raw':stats([r['xy_error_m'] for r in raw]),
        'accepted':stats([r['xy_error_m'] for r in raw if r.get('accepted',True)]),
        'rejected':stats([r['xy_error_m'] for r in raw if not r.get('accepted',True)])}
    (folder/'tracking_audit.json').write_text(json.dumps(report,indent=2))
    print('tracking',folder.name,json.dumps(report))


def control_audit(folder):
    rows = json.loads((folder/'control_evaluation.json').read_text())
    avoidance = [r for r in rows if r['controller_state'].startswith('AVOID_')]
    minimum_row = min(rows,key=lambda r:r['evaluation_gt_distance_m'])
    minimum = minimum_row['evaluation_gt_distance_m']
    last_avoid = avoidance[-1]['sim_time'] if avoidance else rows[0]['sim_time']
    resume = [r for r in rows if r['sim_time']>last_avoid and r['controller_state']=='CRUISE' and r['command'][0]>.15]
    out = {'min_distance_m':minimum,'collision_proxy':minimum<.45,'margin_violation':minimum<.65,
        'result':'FAIL' if minimum<.45 else ('PARTIAL' if minimum<.65 else 'PASS'),
        'avoid_sides':sorted(set(r['controller_state'] for r in avoidance)),
        'avoid_start_sim_time':avoidance[0]['sim_time'] if avoidance else None,
        'peak_command_omega_rad_s':max(abs(r['command'][1]) for r in rows),
        'peak_actual_omega_rad_s':max(abs(r['robot_angular_velocity'][2]) for r in rows),
        'robot_y_range_m':[min(r['robot_position'][1] for r in rows),max(r['robot_position'][1] for r in rows)],
        'human_y_range_m':[min(r['evaluation_gt_human_xyz'][1] for r in rows),max(r['evaluation_gt_human_xyz'][1] for r in rows)],
        'resumed':bool(resume),'resume_time':resume[0]['sim_time'] if resume else None,
        'final_state':rows[-1]['controller_state'],'minimum_distance_row':minimum_row}
    (folder/'control_audit.json').write_text(json.dumps(out,indent=2))
    print('control',folder.name,json.dumps({k:v for k,v in out.items() if k!='minimum_distance_row'}))


def depth_audit(folder):
    config = json.loads(Path(__file__).with_name('config.yaml').read_text())['depth']
    rows = []
    for file in sorted(folder.glob('depth_frame_*.npz')):
        a = np.load(file)
        d = a['depth']
        inv = np.linalg.inv(a['camera_view'].reshape(4,4))
        for box in a['boxes']:
            x1,y1,x2,y2 = box
            u,v = (x1+x2)/2,(y1+y2)/2
            xa,xb = np.clip([int(u-.2*(x2-x1)),int(u+.2*(x2-x1))+1],0,640)
            ya,yb = np.clip([int(v-.2*(y2-y1)),int(v+.2*(y2-y1))+1],0,360)
            values = d[ya:yb,xa:xb]
            values = values[np.isfinite(values)&(values>.05)&(values<100)]
            if len(values)<3:
                continue
            old_z = float(np.median(values))
            old_xyz = (np.array([(u-320)*old_z/240,-(v-180)*old_z/240,-old_z,1])@inv)[:3]
            r = person_depth(d,box,config)
            row = {'frame':file.stem,'old_error':float(np.linalg.norm(old_xyz[:2]-a['gt_evaluation_only'][:2])),**r}
            if 'depth_m' in r:
                z = r['depth_m']
                xyz = (np.array([(r['u']-320)*z/240,-(r['v']-180)*z/240,-z,1])@inv)[:3]
                row['new_error'] = float(np.linalg.norm(xyz[:2]-a['gt_evaluation_only'][:2]))
            rows.append(row)
            if row['old_error']>1.:
                rgb = Image.fromarray(a['rgb']).convert('RGB')
                draw = ImageDraw.Draw(rgb)
                draw.rectangle(box.tolist(),outline='lime',width=2)
                draw.rectangle([int(xa),int(ya),int(xb),int(yb)],outline='red',width=2)
                draw.rectangle(r['roi'],outline='cyan',width=2)
                gray = np.uint8(np.clip(np.nan_to_num(d,nan=10,posinf=10)/10,0,1)*255)
                depth_image = Image.fromarray(gray).convert('RGB')
                dd = ImageDraw.Draw(depth_image)
                dd.rectangle([int(xa),int(ya),int(xb),int(yb)],outline='red',width=2)
                dd.rectangle(r['roi'],outline='cyan',width=2)
                canvas = Image.new('RGB',(1280,420),'white')
                canvas.paste(rgb,(0,60));canvas.paste(depth_image,(640,60))
                ImageDraw.Draw(canvas).text((8,6),
                    f"{file.stem} | RED old ROI: z={old_z:.3f} m, XY error={row['old_error']:.3f} m\n"
                    f"CYAN new torso ROI: z={r.get('depth_m',float('nan')):.3f} m, XY error={row.get('new_error',float('nan')):.3f} m, "
                    f"confidence={r['depth_confidence']:.3f}, accepted={r['accepted']}\n"
                    "Right: optical depth 0-10 m, brighter = farther. Surface localization, not anatomical center.",fill='black')
                canvas.save(folder/(file.stem+'_paired_depth.png'))
    report = {}
    for scope in ['all','truncated','full']:
        s = [r for r in rows if scope=='all' or r['truncated']==(scope=='truncated')]
        report[scope] = {'old':stats([r['old_error'] for r in s]),
            'new_all_raw':stats([r['new_error'] for r in s if 'new_error' in r]),
            'accepted':stats([r['new_error'] for r in s if r['accepted'] and 'new_error' in r]),
            'rejected':stats([r['new_error'] for r in s if not r['accepted'] and 'new_error' in r]),
            'no_candidate':sum('new_error' not in r for r in s)}
    (folder/'paired_depth_audit.json').write_text(json.dumps({'summary':report,'rows':rows},indent=2))
    print(folder.name,json.dumps(report))


def final_report(root):
    folders = [(scene,'no-avoid','reference',root/f'stage7_{scene}_noavoid_valid')
               for scene in ('crossing','headon')]
    folders += [(scene,'depth + lateral',str(run),root/f'stage7_lateral_{scene}_{run}')
                for scene in ('crossing','headon') for run in range(1,4)]
    results = []
    for scene,policy,run,folder in folders:
        control_audit(folder)
        tracking_audit(folder)
        results.append((scene,policy,run,folder,
            json.loads((folder/'control_audit.json').read_text()),json.loads((folder/'summary.json').read_text()),
            json.loads((folder/'tracking_audit.json').read_text())))
    gate = json.loads((root/'stage7_depth_gate_final/paired_depth_audit.json').read_text())['summary']['all']
    full = json.loads((root/'stage7_depth_robust_full/paired_depth_audit.json').read_text())['summary']['full']
    final_runs = [r for r in results if r[1]!='no-avoid']
    failures = sum(r[4]['collision_proxy'] for r in final_runs)
    lines = ['# Depth fix and lateral avoidance: actual experiment report',
        '', f"Final status: {'FAIL - lateral controller is not promoted' if failures else 'Review margin and resume outcomes below'}. "
        f"{failures}/6 final avoidance runs triggered the unchanged collision proxy. "
        "The new controller remains opt-in; the legacy controller is preserved. Depth acceptance improvements are evaluated separately from controller safety.",
        '', '## Depth Fix',
        '', 'The original method was reproduced with max XY error 4.459718 m and P95 0.940204 m '
        '(stage7_depth_legacy_reproduce, 53 samples). On these exact saved frames the final depth replay '
        'had max 0.290483 m and P95 0.285856 m, with 53/53 accepted.',
        '', 'Independent online depth gate (unchanged brake-only controller):',
        '', '| Scope | N | Mean XY m | P95 m | Max m |', '|---|---:|---:|---:|---:|']
    for key in ['old','new_all_raw','accepted','rejected']:
        s = gate[key]
        lines.append(f"| {key} | {s['n']} | {s.get('mean',float('nan')):.6f} | {s.get('p95',float('nan')):.6f} | {s.get('max',float('nan')):.6f} |")
    lines += ['', 'Raw candidates are recorded before confidence/geometry rejection. Accepted means eligible for '
        'KF update/birth, except any separately logged innovation rejection. Rejected candidates remain in raw statistics. '
        'The fix does NOT eliminate every meter-scale raw candidate: ground-only fragments are explicitly rejected.',
        '', f"Full-box paired replay: {full['old']['n']} boxes, old/new mean {full['old']['mean']:.6f}/{full['new_all_raw']['mean']:.6f} m; "
        f"old/new P95 {full['old']['p95']:.6f}/{full['new_all_raw']['p95']:.6f} m. This small degradation is retained, not described as an accuracy improvement.",
        '', 'Implementation: border margin 10 px; adaptive visible-torso ROI; supported near-depth band '
        '(10th percentile only for truncated boxes, median anchor for full boxes); minimum pixel/support checks; '
        'surface-plane normal check rejects ground-like patches. Selected inlier pixels supply both depth and ray. '
        'KF uses an additional 2-D innovation chi-square gate 9.21034. No GT enters these decisions.',
        '', 'Evidence: `stage7_depth_legacy_reproduce/depth_frame_0036_paired_depth.png`, '
        '`stage7_depth_robust_crossing_v2/depth_frame_0035_paired_depth.png`; original RGB/depth/boxes are retained as NPZ. '
        'Intermediate failed depth variants are retained in `stage7_depth_robust_crossing*`; paired replay files use the current helper, '
        'while position_evaluation.json always describes the actual historical run.',
        '', '## Crossing / Head-on: all requested runs',
        '', '| Scenario | Policy | Run | Min distance m | <0.45 collision | <0.65 margin violation | Result |',
        '|---|---|---|---:|---|---|---|']
    for scene,policy,run,folder,a,s,t in sorted(results,key=lambda x:(x[0],x[1]!='no-avoid',x[2])):
        verdict = a['result']
        if policy!='no-avoid' and verdict=='PASS' and not a['resumed']:
            verdict = 'CLEARANCE PASS; NO RESUME'
        lines.append(f"| {scene} | {policy} | {run} | {a['min_distance_m']:.6f} | {a['collision_proxy']} | {a['margin_violation']} | {verdict} |")
    actual = [r for r in results if r[1]!='no-avoid']
    for scene in ['crossing','headon']:
        rs = [r for r in actual if r[0]==scene]
        lines += ['',f"### {scene.title()}",
            f"Collision-free: {sum(not r[4]['collision_proxy'] for r in rs)}/3; full safety margin: {sum(not r[4]['margin_violation'] for r in rs)}/3; resumed: {sum(r[4]['resumed'] for r in rs)}/3."]
        for _,_,run,folder,a,s,t in rs:
            lines.append(f"- Run {run}: sides={a['avoid_sides']}, first avoidance t={a['avoid_start_sim_time']}, "
                f"peak omega command/actual={a['peak_command_omega_rad_s']:.3f}/{a['peak_actual_omega_rad_s']:.3f} rad/s; "
                f"robot Y range={a['robot_y_range_m']}; resumed={a['resumed']}; final state={a['final_state']}. "
                f"Trajectory: `{folder.name}/robot_motion.csv`; full safety diagnostics: `{folder.name}/control_evaluation.json`.")
    lines += ['', '## Tracking',
        '', 'Single-person evaluation uses the closest available estimated track without a distance gate. '
        'Missing GT slots and extra tracks are explicitly counted. GT slots span all processed camera frames, including out-of-FOV intervals. '
        'Velocity reference is the finite-difference derivative of timestamp-aligned live GT. ID transitions count changes between available assignments.',
        '', '| Scene/run | Position mean / median / P95 / max m | Velocity mean / P95 m/s | Missing / GT slots | Extra track rows | ID transitions |',
        '|---|---|---|---|---:|---:|']
    for scene,policy,run,folder,a,s,t in actual:
        p,v = t['position_available_no_distance_gate'],t['velocity_available_no_distance_gate']
        lines.append(f"| {scene}/{run} | {p['mean']:.3f} / {p['median']:.3f} / {p['p95']:.3f} / {p['max']:.3f} | "
            f"{v['mean']:.3f} / {v['p95']:.3f} | {t['missing_track_slots']} / {t['gt_slots_all_camera_frames']} | {t['extra_track_rows']} | {t['id_transitions_available_sequence']} |")
    lines += ['', '| Scene/run | All raw N / P95 / max m | Accepted N / P95 / max m | Rejected N / P95 / max m |', '|---|---|---|---|']
    for scene,policy,run,folder,a,s,t in actual:
        cells = []
        for key in ['all_raw','accepted','rejected']:
            r = t[key]
            cells.append(f"{r['n']} / {r.get('p95',float('nan')):.3f} / {r.get('max',float('nan')):.3f}" if r['n'] else '0 / N/A / N/A')
        lines.append(f"| {scene}/{run} | "+' | '.join(cells)+' |')
    lines += ['', '## Performance', '', '| Scene/run | YOLO responses/s | RTF | Peak whole-GPU MiB | Peak system RAM GiB |', '|---|---:|---:|---:|---:|']
    for scene,policy,run,folder,a,s,t in actual:
        lines.append(f"| {scene}/{run} | {s['yolo']['effective_fps']:.3f} | {s['real_time_factor']:.3f} | {s['gpu_total_memory_peak_mib']:.0f} | {s['system_ram_peak_gib']:.3f} |")
    lines += ['', '## Controller and GT Isolation',
        '', 'The frozen final steering configuration uses v=0.20 m/s, omega limit=1.20 rad/s, '
        'heading deflection=0.80 rad, angular acceleration=2 rad/s², hold=0.75 s. '
        'LEFT/RIGHT use 1.5 s unicycle/CV rollouts at 0.05 s steps, compare minimum separation, and retain the chosen side unless unsafe. '
        'STOP has priority for hard distance, low-quality depth, stale tracks, both candidates unsafe, or loss of an active avoidance target. '
        'Exiting avoidance requires safe CPA, distance above the original 2 m resume threshold, and five fresh cycles. '
        'Original brake thresholds and radii 0.15+0.30+0.20 m are unchanged; no reverse commands.',
        '', 'Actual Jetbot DOF velocity limits were read as 12.56 rad/s per wheel. The final avoidance target '
        'requires at most 8.917 rad/s; commanded and actual angular velocity are recorded separately.',
        '', 'Depth filtering: bbox/depth/intrinsics only. Tracking: measurements/covariance/simulation timestamps only. '
        'Candidate selection and controller: estimated human tracks, robot self-pose/velocity and simulation time only. '
        'Human GT is used only in scenario setup and downstream evaluation, never as a controller input. '
        'No YOLO/model/camera-resolution/simulator/environment changes were made.',
        '', '## Remaining Issues',
        '', '- A finite-horizon safe candidate does not guarantee future collision avoidance. STOP can still leave the robot in a pedestrian corridor.',
        '- Camera FOV loss can terminate observations during steering. The controller conservatively stops when an active avoidance target disappears; automatic resume is not guaranteed.',
        '- Crossing can trigger unnecessary lateral actions and lose its previously working resume behavior. Clearance-only passes are not whole-controller passes.',
        '- In Crossing run 1, the first lateral trigger occurs at simulation t=4.866667 s. An offline, evaluation-only '
        'live-GT velocity check gives stopped-robot CPA distance about 0.943 m (safe), while the estimated-state trigger asserts '
        'a stop-corridor conflict. Velocity/CPA sensitivity therefore remains even after depth outlier rejection; this diagnostic is not fed back to control.',
        '- Extra track rows can occur in the single-person test; no duplicate-removal or detector changes were introduced to hide them.',
        '- Native animated pedestrian motion is not an exact straight line: live paths can deviate from the unchanged head-on target (4,0) to (0,0). No additional yielding, stopping or avoidance behavior was introduced. These runs do not prove performance against an unconditionally straight, nonreactive pedestrian.',
        '- Surface points are not anatomical centers. Ground-like rejection assumes a roughly upright forward camera; general tilted-camera scenes are not validated.',
        '- Independent runs include simulator/async timing variation; no statistical safety guarantee is claimed. No two-person extension was run.',
        '', 'Earlier steering pilots remain in `stage7_lateral_headon_pilot` and `stage7_lateral_headon_corrected`: '
        'min distances 0.377356 m and 0.418065 m, both FAIL. The first exposed a brake/steering speed arbitration bug; '
        'the second used the corrected speed arbitration but a slower/smaller turn. Neither is hidden or used as a passing final repeat.',
        '', 'Reproduce a final run from `runtime`:', '', '```powershell',
        '.\\python.bat ..\\stage_probe.py --stage 7 --seconds 90 --scenario headon --avoidance --robust-depth --lateral-avoidance --label _new_headon_run',
        '```']
    (root/'DEPTH_LATERAL_AVOIDANCE_REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def observation_audit(folder):
    """Exposure-aligned evidence only; GT never enters runtime control."""
    detections = json.loads((folder/'detections.json').read_text())
    records = []
    for file in sorted(folder.glob('exposure_*.npz')):
        data = np.load(file)
        stamp = float(data['sim_time'])
        response = next((d for d in detections if abs(d['sim_time']-stamp)<1e-7),None)
        row = {'exposure_file':file.name,'sim_time':stamp,'response_received':response is not None}
        if response:
            view = data['camera_view'].reshape(4,4)
            root = np.r_[response['evaluation_gt_xyz'],1.]@view
            row.update(boxes=response['boxes'],depth=response['depth_diagnostics'],
                tracks=response['tracks'],rejections=response['measurement_rejected'],
                evaluation_only_root_bearing_deg=float(np.degrees(np.arctan2(root[0],-root[2]))),
                hfov_deg=float(np.degrees(2*np.arctan(float(data['aperture'][0])/(2*float(data['focal']))))))
        Image.fromarray(data['rgb']).save(folder/(file.stem+'.png'))
        records.append(row)
    (folder/'observation_loss_audit.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    print(json.dumps([{k:v for k,v in r.items() if k not in ('tracks','depth')} for r in records],indent=2))


def staged_report(root):
    labels = ['staged_headon_pilot','staged_headon_pilot2','staged_headon_gate','staged_crossing_diagnostic']
    lines = ['# STOP-first staged escape: mechanism audit', '',
        'Status: mechanism gate, not a formal 3+3 benchmark. Perception, depth thresholds, KF equations, scene commands and safety radii were unchanged. '
        'The tracker now exposes its existing covariance; controller-only bounded propagation uses the same CV equations. No simulator human GT enters control.', '',
        '| Run | Min distance m | Collision <0.45 | Margin <0.65 | Lateral trigger | Max local lateral m | Resume | Result |',
        '|---|---:|---|---|---|---:|---|---|']
    details = []
    for label in labels:
        folder = root/('stage7_'+label)
        rows = json.loads((folder/'control_evaluation.json').read_text())
        s = json.loads((folder/'summary.json').read_text())
        minimum = min(r['evaluation_gt_distance_m'] for r in rows)
        lat = max(abs(r['lateral_displacement']) for r in rows)
        turns = [r for r in rows if r['controller_state'].startswith('TURN')]
        translates = [r for r in rows if r['controller_state'].startswith('TRANSLATE')]
        resumed = any(r['controller_state']=='RESUME' for r in rows)
        result = 'FAIL' if minimum<.45 or not resumed else ('PARTIAL' if minimum<.65 else 'DIAGNOSTIC PASS')
        lines.append(f'| {label} | {minimum:.6f} | {minimum<.45} | {minimum<.65} | {bool(turns)} | {lat:.6f} | {resumed} | {result} |')
        valid = [r for r in rows if r.get('fresh_risk_evaluation') and r['risks']]
        heading = turns[0]['heading'] if turns else rows[0]['heading']
        angle = max(abs(float(np.arctan2(np.sin(r['heading']-heading),np.cos(r['heading']-heading)))) for r in rows)
        details += ['',f'## {label}',
            f"- Fresh valid STOP candidate: safe={sum(r['stop_candidate_safe'] for r in valid)}, unsafe={sum(not r['stop_candidate_safe'] for r in valid)}. Empty/invalid-track cycles excluded, not counted as evidence of safety.",
            f'- TURN duration={len(turns)/60:.3f} s; TRANSLATE duration={len(translates)/60:.3f} s; max heading change={np.degrees(angle):.2f} deg.',
            f"- States: {sorted(set(r['controller_state'] for r in rows))}.",
            f"- YOLO responses/s={s['yolo']['effective_fps']:.3f}; RTF={s['real_time_factor']:.3f}; whole-GPU peak={s['gpu_total_memory_peak_mib']:.0f} MiB; system RAM peak={s['system_ram_peak_gib']:.3f} GiB.",
            f'- Full trace: `{folder.name}/control_evaluation.json`; trajectories: `{folder.name}/robot_motion.csv`.',
            '- Phase transitions:']
        previous = None
        for r in rows:
            if r['controller_state']!=previous:
                details.append(f"  - t={r['sim_time']:.3f}: {r['controller_state']}, lateral={r['lateral_displacement']:.3f} m, prediction_age={r['prediction_age']}, uncertainty_stop={r['blocked_uncertainty']}.")
                previous = r['controller_state']
    lines += ['', '## Implemented mechanism',
        '- STOP rollout integrates measured forward velocity with 0.60 m/s² braking, then holds position. Horizon is at least 5 s and extends towards predicted passage (cap 10 s).',
        '- Escape rollout models TURN, straight TRANSLATE to local lateral displacement 0.75 m, and a final stop. It includes acceleration limits and a camera-visibility penalty. Left/right are compared, not hard-coded.',
        '- TURN target 60 deg, v=0.10 m/s; TRANSLATE v=0.32 m/s. Wheel demands are 5.583 rad/s (turn maximum) and 10.667 rad/s (translate), below the measured 12.56 rad/s limit.',
        '- Existing KF covariance is exposed, not changed. Active track ID is retained; controller prediction is bounded by 0.8 s since observation, position variance 0.36 m², and velocity variance 1.0 (m/s)².',
        '- CLEAR/RECOVER/RESUME are implemented but cannot be claimed validated unless reached in the trace. A paused escape retains its phase. Cached unsafe-candidate decisions remain active between fresh updates.',
        '', '## Gate decision and limitations',
        '- Head-on must demonstrate sufficient translation and recovery before formal repetition. Failed pilots are not counted as formal runs, and no missing result is fabricated.',
        '- The initial pilot exposed a lost phase on STOP; pilot2 checked phase restoration. The final gate additionally retains an unsafe-candidate veto between fresh updates.',
        '- Final Head-on gate: TURN_RIGHT starts at 2.067 s, TRANSLATE_RIGHT at 3.233 s, and STOP at 3.267 s. At this STOP, uncertainty is not the trigger: right continuation is predicted unsafe (about 0.190 m), while left is about 0.721 m. The current implementation vetoes the selected side but does not implement a guarded side change. Side-hold timing is stored but not yet used for switching. This is an unresolved controller limitation, not proof that the physical escape is impossible.',
        '- The final gate reaches only 0.104 m lateral displacement, below the 0.75 m target, and collides. FOV/age expiry occurs subsequently; it must not be presented as the sole or first cause of this gate failure.',
        '- Crossing diagnostic reaches RESUME without a TURN/TRANSLATE trigger and maintains 1.279 m minimum distance. This is one diagnostic run, not a three-run acceptance result.',
        '- Ideal CV/unicycle calculation at 3.5–4 m gives about 0.821 m separation, but only about 20% of the full candidate horizon retains camera visibility. This is not an experimental safety result.',
        '- A frontal camera, bounded blind prediction, and required 0.75 m escape compete geometrically. Do not solve this by silently increasing blind prediction age or uncertainty thresholds.',
        '- The original native pedestrian target/speed remain unchanged; native trajectory deviations are not proof of a perfectly straight, nonreactive pedestrian.',
        '- Formal Head-on ×3 and Crossing ×3 are NOT RUN if the mechanism gate fails. The controller remains experimental, not a functional avoidance PASS.',
        '', '## Reproduction', '```powershell',
        '.\\runtime\\python.bat stage_probe.py --stage 7 --seconds 90 --scenario headon --avoidance --robust-depth --lateral-avoidance --label _staged_new_pilot', '```']
    (root/'STAGED_ESCAPE_MECHANISM_REPORT.md').write_text('\n'.join(lines+details)+'\n',encoding='utf-8')


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('folders',nargs='*',type=Path)
    parser.add_argument('--final-report',type=Path)
    parser.add_argument('--staged-report',type=Path)
    parser.add_argument('--observation-audit',type=Path)
    args = parser.parse_args()
    if args.observation_audit:
        observation_audit(args.observation_audit)
    if args.staged_report:
        staged_report(args.staged_report)
    if args.final_report:
        final_report(args.final_report)
    for folder in args.folders:
        depth_audit(folder)
        if (folder/'detections.json').exists():
            tracking_audit(folder)
        if (folder/'control_evaluation.json').exists():
            control_audit(folder)
