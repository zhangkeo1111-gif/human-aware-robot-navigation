"""Separate experiment driver; frozen Classic evaluator is imported unchanged."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import numpy as np
import classic_benchmark as classic

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'outputs/active_passing'

def evaluate(folder):
    folder=Path(folder);m=classic.evaluate(folder);m['controller']='active_passing'
    logs=classic.read(folder/'social_planning.json');meta=classic.read(folder/'meta.json')
    times=np.array([r['time']-meta['initial_sim_time'] for r in logs]);dt=np.diff(times,prepend=times[0])
    phases=[r['passing_phase'] for r in logs];m['passing_duration']=0.
    for phase in ('OFFSET','PARALLEL','RECOVER'):
        value=float(sum(d for d,p in zip(dt,phases) if p==phase))
        m[phase.lower()+'_duration']=value;m['passing_duration']+=value
    m['final_lateral_error']=abs(logs[-1]['route_d'])
    m['minimum_passing_clearance']=m['passing_clearance_m']
    m['emergency_stop_count']=sum(p=='EMERGENCY_STOP' and (i==0 or phases[i-1]!=p) for i,p in enumerate(phases))
    m['pass_replan_count']=logs[-1]['pass_replan_count']
    m['planning_p95_ms']=float(np.percentile([r['planning_ms'] for r in logs],95))
    m['stop_reasons']=sorted({r['stop_reason'] for r in logs if r['stop_reason']})
    # Causal join of held 10 Hz planner state to the 60 Hz ego/control log.
    # No evaluation human coordinates are read into this trace.
    ego=classic.read(folder/'demo_evaluation.json')
    origin=np.array(meta['robot_start'][:2]);axis=np.array(meta['robot_waypoints'][-1])-origin;axis/=np.linalg.norm(axis)
    normal=np.array([-axis[1],axis[0]])
    keys=['controller_state','passing_phase','selected_side','selected_offset','human_s','human_d','target_lateral',
          'current_clearance','predicted_min_clearance','track_age','v_cmd','omega_cmd','stop_reason']
    trace=[]
    for r in ego:
        k=int(np.searchsorted(times,r['time']+1e-7,side='right')-1)
        if k<0:continue
        q=np.array(r['robot_position'][:2])-origin
        trace.append(dict(sim_time=r['time'],plan_time=float(times[k]),plan_age_s=float(r['time']-times[k]),
            route_s=float(q@axis),route_d=float(q@normal),**{key:logs[k][key] for key in keys}))
    with (folder/'active_step_trace.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(trace[0]));w.writeheader();w.writerows(trace)
    classic.save(folder/'active_metrics.json',m)
    print(json.dumps(m),flush=True);return m

def run(scene,repeat,phase):
    parent=OUT/phase/scene/'active_passing'/f'seed_{repeat}'
    index=len(list(parent.glob('run_*')))+1
    OUT.mkdir(exist_ok=True)
    log=OUT/f'{phase}_{scene}_{repeat}_run_{index:02d}.log'
    cmd=[str(ROOT/'runtime/python.bat'),'stage_probe.py','--stage','7','--robot-model','a300','--motion-mode','kinematic',
        '--classic-scene','--classic-scenario',scene,'--robot-behavior','active_passing','--seed',str(repeat),
        '--run-phase',phase,'--headless','--seconds','300','--record-demo']
    if scene in ('headon','static_obstruction'):cmd+=['--record-first-person']
    with log.open('w') as f:result=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
    folder=parent/f'run_{index:02d}'
    if result.returncode or not (folder/'summary.json').exists():
        raise RuntimeError(f'Infrastructure failure retained: {folder}; {log}')
    return evaluate(folder)

def figure(folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder=Path(folder);m=evaluate(folder);rows=classic.read(folder/'demo_evaluation.json');logs=classic.read(folder/'social_planning.json')
    meta=classic.read(folder/'meta.json');p=np.array([r['robot_position'][:2] for r in rows]);h=np.array([r['evaluation_people'][0][:2] for r in rows])
    t=np.array([r['time']-meta['initial_sim_time'] for r in logs]);d=np.array([r['route_d'] for r in logs])
    plt.rcParams.update({'font.family':['Arial','DejaVu Sans'],'font.size':12,'axes.spines.top':False,'axes.spines.right':False,'legend.frameon':False})
    fig,axes=plt.subplots(2,1,figsize=(11,7))
    axes[0].plot(p[:,0],p[:,1],color='#0F4D92',label='Robot');axes[0].plot(h[:,0],h[:,1],'--',color='#B64342',label='Human (evaluation only)')
    axes[0].axhline(0,color='gray',ls=':');axes[0].set(xlim=(-.3,9.5),ylim=(-2,2),xlabel='X (m)',ylabel='Y (m)');axes[0].set_aspect('equal');axes[0].legend(loc='upper right')
    axes[1].plot(t,d,color='#0F4D92');axes[1].axhline(.5,color='gray',ls='--');axes[1].axhline(-.5,color='gray',ls='--')
    last=None
    for i,r in enumerate(logs):
        if r['passing_phase']!=last:
            axes[1].axvline(t[i],color='#767676',alpha=.5);axes[1].annotate(r['passing_phase'],(t[i],d[i]),xytext=(3,18+(i%2)*25),textcoords='offset points',fontsize=8)
            last=r['passing_phase']
    axes[1].set(xlabel='Simulation time (s)',ylabel='Route-local offset (m)')
    fig.suptitle(f'{m["scenario"]} | {logs[-1]["selected_side"]} | active passing={m["active_passing_success"]} | complete={m["route_completed"]}')
    fig.tight_layout();label={'static_obstruction':'STATIC','headon':'HEADON','overtaking':'OVERTAKING'}[m['scenario']]
    fig.savefig(OUT/f'ACTIVE_PASSING_{label}_TOPDOWN.png',dpi=300);plt.close(fig)

def report():
    classic.checks()
    protected=['classic_single_scene.py','classic_single_scene.usd','classic_benchmark.py',
        'outputs/classic_single_pedestrian/CLASSIC_SINGLE_CONFIG_FROZEN.json']
    for f in protected:
        original=subprocess.check_output(['git','show','feature/classic-single-pedestrian:'+f],cwd=ROOT)
        assert original.replace(b'\r\n',b'\n')==(ROOT/f).read_bytes().replace(b'\r\n',b'\n'),f
    import ast
    tree=ast.parse((ROOT/'active_passing_controller.py').read_text())
    names={n.id for n in ast.walk(tree) if isinstance(n,ast.Name)}
    assert not names & {'actor','evaluation_people','scenario','human_route','gt_history'}
    rows=[evaluate(f.parent) for f in sorted((OUT/'development').glob('*/*/seed_*/run_*/summary.json'))]
    with (OUT/'ACTIVE_PASSING_DEVELOPMENT_RESULTS.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    baseline=list(csv.DictReader((classic.OUT/'CLASSIC_SINGLE_PEDESTRIAN_RESULTS.csv').open()))
    text=['# Active Passing Experiment',
        '## Current Problem',
        'Frozen v0.3: Head-on/Static active passing 0/6; overtaking 0/3. This branch is based on main acafebb, not the failed social-nav-v2 branch. The frozen Classic scene/evaluator are copied byte-equivalently from feature/classic-single-pedestrian. No historical run is overwritten.',
        '## Planner',
        'Independent ActivePassingController; inputs are estimated tracks, self pose/velocity, route, known static map and time. Full geometric path uses quintic OFFSET, constant-offset PARALLEL and quintic RECOVER. Execution holds the offset until the estimated target is at least 0.7 m behind; it does not blindly enter the geometric recovery segment.',
        '## Candidate Generation',
        'Six candidates: left/right 0.6, 0.8, 1.0 m. Route coordinates use the start-to-goal unit vector and its normal. Bounded dynamic collision checks use 6 s at 0.1 s with the unchanged 0.5/0.8 m/s2 linear and 1.2 rad/s2 angular limits. Full geometric paths are also checked against static geometry; a parallel-corridor check rejects offsets narrower than the unchanged robot-plus-human radius. This is NOT a claim that the full several-meter maneuver can finish in six seconds at 0.30 m/s.',
        'Score: 4*social_integral - predicted_progress + 0.12*(path_length+steering_effort) + 0.15*final_lateral_error. Hard collision is rejection, not a soft penalty. Nominal personal-space sigmas remain 1.2/0.7/0.8 m; no cost weight search was performed.',
        '## State Machine',
        'NAVIGATE -> PASS_INIT -> OFFSET -> PARALLEL -> RECOVER -> PASS_DONE. EMERGENCY_STOP interrupts unsafe or stale execution. Chosen side is retained on safety recheck; fresh ID changes may rebind only via a unique spatial match within 0.8 m and the bounded freshness interval. Temporary absent tracks may use the existing CV velocity only up to total observation age 0.8 s. No long-term blind passing. RECOVER may continue only after a fresh estimate already established the person behind.',
        'Stationary classification records speed <0.12 m/s for 0.5 s. It does not remove the pedestrian personal-space cost or replace the unchanged KF prediction with a GT/static trajectory.',
        'social_planning.json stores each 10 Hz plan and candidate diagnostics. active_step_trace.csv causally joins that held plan to each 60 Hz ego step: route_s/d are current ego coordinates; human/clearance/track-age fields are explicitly the last plan values, with plan_time and plan_age_s. No future plan or human GT is joined into these fields.',
        '## Static Results',
        '| Development run | Complete | Collision | Active passing | Max lateral m | Emergency stops | Reason |',
        '|---|---|---|---|---:|---:|---|']
    for r in rows:
        text.append(f'| {Path(r["source_run"]).name} | {r["route_completed"]} | {r["collision_proxy"]} | {r["active_passing_success"]} | {r["max_lateral_displacement"]:.3f} | {r["emergency_stop_count"]} | {", ".join(r["stop_reasons"])} |')
    text+=['Development runs are not formal repeats. Run 01 is the initial implementation; run 02 includes bounded target rebinding/corridor checks. Run 03 records its exact controller source and corresponds to the final mechanism revision. All failed runs are retained.',
        '![Actual static trajectory and phases](ACTIVE_PASSING_STATIC_TOPDOWN.png)',
        '## Head-on Results','NOT RUN: static mechanism gate requires two consecutive successes before Head-on.',
        '## Overtaking Results','NOT RUN: Head-on mechanism gate has not passed.',
        '## Baseline / v0.3 / ActivePassing Comparison',
        'The 18 original frozen comparator runs remain available. They are references, not a completed 27-run formal comparison. No new formal ActivePassing benchmark was started.',
        '| Scenario | Controller | Complete | Collision | Active passing | Overtaking |',
        '|---|---|---:|---:|---:|---:|']
    for s in ('static_obstruction','headon','overtaking'):
        for c in ('baseline','social_nav'):
            g=[r for r in baseline if r['scenario']==s and r['controller']==c]
            text.append(f'| {s} | {c} | '+ ' | '.join(str(sum(r[k]=='True' for r in g))+'/3' for k in ('route_completed','collision_proxy','active_passing_success','overtaking_success'))+' |')
    text+=['## Passing Metrics',
        '| Run | Offset s | Parallel s | Recover s | Final lateral m | Min passing clearance m | Replans |',
        '|---|---:|---:|---:|---:|---|---:|']
    for r in rows:text.append(f'| {Path(r["source_run"]).name} | {r["offset_duration"]:.3f} | {r["parallel_duration"]:.3f} | {r["recover_duration"]:.3f} | {r["final_lateral_error"]:.3f} | {r["minimum_passing_clearance"]} | {r["pass_replan_count"]} |')
    text+=['Replan count includes safe same-side retry checks while stopped; it is not a side-switch count. Minimum passing clearance is N/A when no longitudinal passing window is reached.',
        '## Social Metrics',
        '| Run | Min distance m | Stop s | Path length m | Integrated social cost | Norm <1.5 s | Omega variation |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for r in rows:text.append('| '+Path(r['source_run']).name+' | '+' | '.join(f'{r[k]:.3f}' for k in ('min_gt_distance','stop_duration','path_length','integrated_social_cost','social_lt_1_5_s','omega_variation'))+' |')
    text+=['## Perception Audit',
        'Crossing and Blind Corner: NOT RUN, per the prerequisite that all three passing mechanism gates must pass first. Their earlier perception limitation is not used to tune this planner. Static obstruction first-person inspection is part of diagnosing the currently failing mechanism.',
        'Final static run 03 evidence: NAVIGATE 0.0 s; OFFSET 0.30 s; predicted hard-collision stop 3.90 s; same-side retry 5.00 s; stale-target stop 6.00 s. Last YOLO person exposure 5.1667 s (box x=0.07..24.22 px); first subsequent empty exposure 5.2667 s. The raw video shows the person clipping at the left image edge, then leaving view. Evaluation-only bearing from robot root to human root increases from about 49.96 degrees at 5.1667 s to 53.36 degrees at 6 s and 54.18 degrees at 7 s. The camera mount shifts exact optical bearing; these root-bearing numbers are supporting evidence, not a substitute for the image. No GT bearing reaches control.',
        'This diagnoses this implemented maneuver, not a proof that every planner is impossible under the camera constraints. Synthetic continuously observed stationary tracks complete all four maneuver phases; real camera support during the tested turn does not. Reducing curvature or explicitly planning for visibility remains unvalidated future work. No camera change or stale-limit relaxation was used to hide this failure.',
        '![Static first-person visual check](STATIC_FIRST_PERSON_REVIEW.png)',
        '## Performance','Planning P95 includes stopped cycles; a low number must not be interpreted as sustained successful passing performance.',
        '| Run | Planner P95 ms | YOLO responses/wall-s | RTF |','|---|---:|---:|---:|']
    for r in rows:text.append(f'| {Path(r["source_run"]).name} | {r["planning_p95_ms"]:.3f} | {r["yolo_responses_s"]:.3f} | {r["rtf"]:.3f} |')
    text+=['## GT Isolation','Static source inspection and simulator call-site review: no actor, scenario label, human route or evaluation state enters ActivePassingController. GT is only read after the run by the unchanged Classic evaluator/figures. Protected detector, depth configuration, KF, motion backend and controllers are unchanged.',
        '## Limitations','Kinematic A300; simple CV prediction; one pedestrian; preset routes; no human intent model; simulation only; no real robot; no safety certification. Six-second predicted safety does not certify the whole future maneuver. The fixed forward camera and 0.8 s freshness bound can interrupt lateral maneuvers. Only limited offset magnitudes are implemented. Performance and reliability on unrun scenes are unknown.',
        '## Videos']
    for f in sorted((OUT/'development').glob('**/CLASSIC_*.mp4')):text.append(f'- [{f.parent.parent.name}/{f.name}]({f.as_posix()})')
    text+=['## Final Verdict','**ACTIVE PASSING RELEASE GATE FAIL**',
        'Static mechanism has not achieved two consecutive active passes. Head-on, overtaking and the nine formal new runs are therefore blocked, not silently treated as passes. No main merge, VERSION update, tag or remote push. Partial lateral displacement alone is not successful passing.']
    (OUT/'ACTIVE_PASSING_REPORT.md').write_text('\n\n'.join(text),encoding='utf8')
    classic.save(OUT/'FINAL_STATUS.json',dict(release_gate='FAIL',reason='STATIC_MECHANISM_GATE_NOT_PASSED',formal_active_runs=0,
        development_runs=len(rows),protected_files_unchanged=True,main_unchanged=True,release=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['run','evaluate','figure','report']);p.add_argument('--scene',default='static_obstruction');p.add_argument('--repeat',type=int,default=17);p.add_argument('--phase',default='development');p.add_argument('--folder');a=p.parse_args()
    if a.mode=='run':run(a.scene,a.repeat,a.phase)
    elif a.mode=='evaluate':evaluate(a.folder)
    elif a.mode=='figure':figure(a.folder)
    else:report()
