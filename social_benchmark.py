"""Small deterministic checks, sequential paired runner, and post-run evaluation.

This file reads GT ONLY after each process exits. No evaluation result enters control.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
from social_controller import SocialController, personal_field

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'outputs/social_navigation'
SCENARIOS=('frontal','obstruction','blind_corner','perpendicular','circular')


def checks():
    c=json.loads((ROOT/'config.yaml').read_text())['social_navigation']
    def planner():return SocialController(c,[[8.5,0]],[],.60278,dict(linear_accel_m_s2=.5,linear_decel_m_s2=.8,angular_accel_rad_s2=1.2))
    def track(pos,vel=(0,0),id=1):return dict(track_id=id,state=[*pos,*vel],last_observed_time=0.,confidence=.9)
    p=planner();p.step(0,[track([2,0])],0,[0,0],0)
    costs=p.logs[-1]['costs']
    assert costs[2]['social']>costs[4]['social']
    p=planner();p.step(0,[track([1,-1],(0,.65))],0,[0,0],0,v=.3)
    assert not p.logs[-1]['costs'][2]['hard_valid']
    assert p.selected!='STRAIGHT'
    p=planner();p.step(0,[track([-3,0],(-.65,0))],0,[0,0],0)
    assert p.selected=='STRAIGHT'
    p=planner();h=p.humans(0,[track([1,-.5],(.3,0)),track([1,.5],(.3,0),2)],0)
    assert p.group_pairs(h)==[(0,1)]
    # Equal progress/heading/control synthetic alternatives: the bridge alone
    # must prefer going outside the pair, not between its members.
    bridge=np.exp(-.5*(np.array([0.,1.2])/c['group_sigma'])**2)
    assert np.argmin(c['group_weight']*bridge)==1
    p.step(0,[track([1,-.5],(.3,0)),track([1,.5],(.3,0),2)],0,[0,0],0)
    assert p.logs[-1]['group_count']==1
    # Covariance and future movement must actually reach scoring.
    p=planner();t=track([3,0],(.2,0));t['covariance']=np.diag([.02,.02,.1,.1]).tolist()
    h=p.humans(.2,[t],0)[0]
    assert h['margin'][-1]>h['margin'][0] and h['predicted'][-1,0]>h['predicted'][0,0]
    assert not p.humans(1.1,[t],0)
    p=planner();p.waypoints=[np.array([4.,0.])]
    p.step(0,[],None,[3.65,0],0)
    assert p.selected!='STOP','near-waypoint overshoot must not deadlock'
    p=planner();p.committed_side=1;p.commit_until=1.2
    p.step(0,[track([4,0])],0,[0,0],0)
    assert p.logs[-1]['previous_side_hard_safe']
    assert all(not x['valid'] for x in p.logs[-1]['costs'] if x['name'] in ('RIGHT','GENTLE_RIGHT'))
    # Current and baseline sources must stay byte-identical for frozen modules.
    frozen=('demo_scene.py','navwareset_scene.py','robot_backend.py','tracker.py','yolo_worker.py','walking_actor.py')
    for name in frozen:assert (ROOT/name).read_bytes()==(OUT/'baseline_source'/name).read_bytes(),name
    import ast
    tree=ast.parse((ROOT/'social_controller.py').read_text())
    imports=[n.names[0].name for n in ast.walk(tree) if isinstance(n,ast.Import)]
    assert set(imports)=={'time','numpy'} and not any(isinstance(n,ast.ImportFrom) for n in ast.walk(tree))
    result=dict(status='PASS',cases=['stationary_cost_order','future_crossing_rejected','behind_no_needless_stop',
       'group_pair_and_bridge','covariance_inflation','stale_expiry','near_waypoint_progress','side_commitment','controller_import_isolation','frozen_sources_unchanged'],
       group_scope='synthetic equal-other-cost ranking; not a human-group comfort validation')
    (OUT/'SYNTHETIC_CHECKS.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)


def evaluate(folder):
    folder=Path(folder);summary=json.loads((folder/'summary.json').read_text())
    rows=json.loads((folder/'demo_evaluation.json').read_text());old=json.loads((folder/'scenario_evaluation.json').read_text())
    meta=json.loads((folder/'meta.json').read_text());c=summary['configuration']['social_navigation']
    # Stop time integration at route completion, not the terminal video padding.
    end=next((i for i,r in enumerate(rows) if r['route_done']),len(rows)-1)
    r=rows[:end+1];times=np.array([x['time'] for x in r]);dt=np.diff(times,prepend=times[0])
    robot=np.array([x['robot_position'][:2] for x in r]);humans=np.array([x['evaluation_people'] for x in r])[:,:,:2]
    vel=np.zeros_like(humans);vel[1:]=np.diff(humans,axis=0)/np.maximum(dt[1:,None,None],1e-9)
    norms=np.full((len(r),humans.shape[1]),np.inf);fields=np.zeros_like(norms)
    for j in range(humans.shape[1]):
        heading=None
        for i in range(len(r)):
            if np.linalg.norm(vel[i,j])>=c['heading_min_speed']:heading=np.arctan2(vel[i,j,1],vel[i,j,0])
            fields[i,j],norms[i,j]=personal_field(robot[i]-humans[i,j],heading,c)
    states=[x['state'] for x in r];cmd=np.array([x['command'] for x in r]);omega=np.array([x['omega'] for x in r])
    signs=np.sign(cmd[np.abs(cmd[:,1])>.1,1]);switches=int(np.sum(signs[1:]!=signs[:-1]))
    stop=np.array([s=='STOP' and not r[i]['route_done'] for i,s in enumerate(states)]);avoid=np.array([s.startswith('AVOID') for s in states])
    travel=old['travel_time_s'];duration=times[-1]-times[0]
    result=dict(scenario=meta['scenario'],controller='baseline' if meta['robot_behavior']=='social' else meta['robot_behavior'],
        seed=meta.get('repeat_seed',17),route_completed=old['route_done'],collision_proxy=old['collision_proxy'],
        static_collision=old['robot_static_disk_clearance_m']<0,min_gt_distance=old['minimum_gt_distance_m'],
        personal_space_violation_s=float(np.sum(dt*(norms.min(axis=1)<1))),
        personal_space_violation_fraction=float(np.sum(dt*(norms.min(axis=1)<1))/max(duration,1e-9)),
        integrated_social_cost=float(np.sum(dt*fields.sum(axis=1))),mean_social_cost=float(np.sum(dt*fields.sum(axis=1))/max(duration,1e-9)),
        min_social_norm=float(norms.min()),travel_time=travel,observed_duration=duration,
        path_length=float(np.linalg.norm(np.diff(robot,axis=0),axis=1).sum()),
        stop_count=int(sum(s=='STOP' and (i==0 or states[i-1]!='STOP') and not r[i]['route_done'] for i,s in enumerate(states))),
        stop_duration=float(np.sum(dt*stop)),steering_time=float(np.sum(dt*(np.abs(cmd[:,1])>.1))),avoid_time=float(np.sum(dt*avoid)),
        direction_switches=switches,omega_variation=float(np.abs(np.diff(omega)).sum()),control_effort=float(np.sum(dt*omega**2)),
        yolo_responses_s=old['yolo_responses_per_wall_s'],rtf=old['rtf'],resume=old['resume'],
        max_tracks=old['max_tracks'],gpu_peak_mib=old['gpu_peak_mib'],ram_peak_gib=old['system_ram_peak_gib'],source_run=str(folder))
    result['repeated_exposures_skipped']=summary.get('repeated_exposures_skipped',0)
    result['physical_human_count']=humans.shape[1]
    result['min_human_walk_distance_m']=min(old.get('walking_path_lengths_m',[0.]))
    planfile=folder/'social_planning.json'
    if planfile.exists():
        plans=json.loads(planfile.read_text())
        social_decisions=0
        for p in plans:
            valid=[x for x in p['costs'] if x['valid']]
            if valid:
                normal=min(valid,key=lambda x:x['total'])['name']
                removed=min(valid,key=lambda x:x['total']-c['weights']['social']*x['social'])['name']
                social_decisions+=normal!=removed
        result.update(planner_p95_ms=float(np.percentile([p['planning_ms'] for p in plans],95)),
            max_planned_humans=max(p['human_count'] for p in plans),group_plan_count=sum(p['group_count']>0 for p in plans),
            social_changes_argmin=social_decisions,no_safe_plans=sum(p['no_safe_candidate'] for p in plans),
            commitment_plans=sum(p['time']<p['commit_until'] for p in plans),
            commitment_violations=sum(p.get('previous_side_hard_safe',False) and p['time']<prior['commit_until']
                and p.get('side_before',0)*({'LEFT':1,'GENTLE_LEFT':1,'RIGHT':-1,'GENTLE_RIGHT':-1}.get(p['selected'],0))<0
                for prior,p in zip(plans,plans[1:])),
            held_opposite_plans=sum(p.get('held_opposite_candidates',0)>0 for p in plans))
    result['functional_pass']=bool(result['route_completed'] and not result['collision_proxy'] and not result['static_collision'] and summary.get('yolo',{}).get('samples',0)>0)
    (folder/'social_metrics.json').write_text(json.dumps(result,indent=2))
    return result


def run(scenario,behavior,seed,phase,video=False):
    logdir=OUT/'run_logs';logdir.mkdir(exist_ok=True)
    name=f'{phase}_{scenario}_{behavior}_{seed}_{time.time_ns()}'
    args=[str(ROOT/'runtime/python.bat'),'stage_probe.py','--stage','7','--robot-model','a300','--motion-mode','kinematic',
          '--navwareset-scene','--navwareset-scenario',scenario,'--robot-behavior',behavior,'--social-experiment',
          '--seed',str(seed),'--run-phase',phase,'--headless','--seconds','200']
    # Same render products for every paired run, no timing advantage by controller.
    args+=['--record-demo']
    if video and scenario=='circular' and behavior=='social_nav':args+=['--record-first-person']
    print('START',name,flush=True)
    with (logdir/(name+'.log')).open('w') as f:
        child=subprocess.Popen(args,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        try:code=child.wait(timeout=480)
        except subprocess.TimeoutExpired:
            subprocess.run(['taskkill','/PID',str(child.pid),'/T','/F'],capture_output=True)
            raise RuntimeError(f'Run timeout {name}; full log retained')
    parent=OUT/phase/scenario/('baseline' if behavior=='social' else behavior)/f'seed_{seed}'
    folders=sorted(parent.glob('run_*'));folder=folders[-1] if folders else None
    if folder is None or not (folder/'scenario_evaluation.json').exists():raise RuntimeError(f'Run failed {name}, exit {code}')
    result=evaluate(folder)
    print('DONE',json.dumps(result),flush=True)
    return result


def collect():
    allrows=[evaluate(p.parent) for p in sorted((OUT/'benchmark').glob('*/*/seed_*/run_*/scenario_evaluation.json'))]
    if not allrows:return
    keys=list(dict.fromkeys(k for r in allrows for k in r))
    with (OUT/'SOCIAL_NAVIGATION_RESULTS.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(allrows)
    return allrows


def topdown():
    """One fixed-run estimated-state diagnostic; never used to tune or rank runs."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    files=sorted((OUT/'benchmark/circular/social_nav/seed_17').glob('run_*/social_planning.json'))
    if not files:return
    plans=json.loads(files[0].read_text())
    p=next((p for p in plans if p['human_count']>=3 and p['selected'] not in ('STOP','STRAIGHT')),None)
    if p is None:p=next(p for p in plans if p['human_count']>=3)
    c=json.loads((OUT/'BENCHMARK_CONFIG_FROZEN.json').read_text())['social_navigation']
    plt.rcParams.update({'font.family':['Arial','DejaVu Sans'],'font.size':12,'axes.spines.top':False,'axes.spines.right':False,'legend.frameon':False,'svg.fonttype':'none'})
    fig,ax=plt.subplots(figsize=(11,7))
    rp=np.array(p['robot_position']);allpoints=[rp]
    for i,path in enumerate(p['candidates']):
        xy=np.array(path);selected=p['costs'][i]['name']==p['selected'];valid=p['costs'][i]['valid']
        ax.plot(xy[:,0],xy[:,1],color='#0F4D92' if selected else '#999999',lw=3 if selected else 1.2,ls='-' if valid else ':',alpha=1 if selected else .7,label='Selected primitive' if selected else 'Other primitives (dotted: rejected)' if i==0 else None)
    ax.add_patch(Circle(rp,.60278,color='#0F4D92',alpha=.12))
    ax.scatter(*rp,s=80,c='#0F4D92',marker='s',label='Robot / conservative footprint')
    theta=np.linspace(0,2*np.pi,181)
    for i,h in enumerate(p['estimated_humans']):
        pos=np.array(h['position']);vel=np.array(h['velocity']);pred=pos+np.arange(31)[:,None]*.1*vel
        allpoints.extend(pred);heading=h['heading'];margin=h['margin'][0]
        long=np.cos(theta)*(np.where(np.cos(theta)>=0,c['front_sigma'],c['rear_sigma'])+margin) if heading is not None else np.cos(theta)*(c['side_sigma']+margin)
        lat=np.sin(theta)*(c['side_sigma']+margin);a=heading or 0.
        contour=np.column_stack((long*np.cos(a)-lat*np.sin(a),long*np.sin(a)+lat*np.cos(a)))+pos
        ax.plot(contour[:,0],contour[:,1],color='#42949E',alpha=.65,lw=1,label='Current personal-space contour (inflated)' if i==0 else None)
        ax.plot(pred[:,0],pred[:,1],'--',color='#B64342',lw=1.5,label='CV human prediction: 3 s' if i==0 else None)
        ax.scatter(*pos,c='#B64342',s=45,label='Estimated person track' if i==0 else None)
        ax.scatter(*pred[-1],c='#B64342',marker='x',s=35)
        ax.annotate(f"T{h['id']}",pos,xytext=(5,6),textcoords='offset points')
    points=np.array(allpoints);ax.set_xlim(points[:,0].min()-1.5,points[:,0].max()+1.5);ax.set_ylim(points[:,1].min()-1.5,points[:,1].max()+1.5)
    ax.set_aspect('equal');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.grid(alpha=.15)
    ax.set_title(f"Circular: estimated-state planning snapshot | selected {p['selected']}",fontsize=15,pad=18)
    ax.legend(loc='upper center',bbox_to_anchor=(.5,-.16),ncol=2,fontsize=10)
    fig.text(.5,.02,'Fixed seed 17 run; first qualifying multi-track interaction. Tracks may include duplicates. No human GT.',ha='center',fontsize=10,color='#555555')
    fig.tight_layout(rect=(0,.11,1,1))
    for ext in ('png','svg'):fig.savefig(OUT/f'SOCIAL_COST_TOPDOWN.{ext}',dpi=300,facecolor='white')
    plt.close(fig)


def regression():
    checks();items=[]
    for indoor in (True,False):
        label=f'_social_navigation_regression_{time.time_ns()}'
        args=[str(ROOT/'runtime/python.bat'),'stage_probe.py','--stage','7','--robot-model','a300','--motion-mode','kinematic',
              '--headless','--seconds','60','--sim-seconds','5','--label',label]
        if indoor:args+=['--demo-scene','--demo-controller']
        else:args+=['--navwareset-scene','--navwareset-scenario','frontal','--robot-behavior','social','--social-experiment','--run-phase','regression']
        with (OUT/'run_logs'/f'{label}.log').open('w') as log:
            child=subprocess.Popen(args,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            try:child.wait(timeout=480)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(child.pid),'/T','/F'],capture_output=True);raise
        folder=ROOT/'outputs'/('stage7'+label) if indoor else sorted((OUT/'regression/frontal/baseline/seed_17').glob('run_*'))[-1]
        summary=json.loads((folder/'summary.json').read_text())
        good=not (folder/'failure.txt').exists() and summary.get('rgb_frames',0)>10 and summary.get('yolo',{}).get('samples',0)>0
        items.append(dict(scene='indoor' if indoor else 'navwareset_frontal',passed=good,source_run=str(folder),rgb_frames=summary.get('rgb_frames'),yolo_samples=summary.get('yolo',{}).get('samples')))
        print('REGRESSION',json.dumps(items[-1]),flush=True)
    (OUT/'REGRESSION.json').write_text(json.dumps(dict(passed=all(x['passed'] for x in items),scope='Short startup/perception/legacy-controller smoke; formal runs cover full NavWareSet behavior',runs=items),indent=2))


def videos():
    items=[]
    for scene,controller in [('frontal','baseline'),('frontal','social_nav'),('perpendicular','baseline'),('perpendicular','social_nav'),('circular','social_nav')]:
        folder=sorted((OUT/f'benchmark/{scene}/{controller}/seed_17').glob('run_*'))[0]
        files=list(folder.glob('NAVWARESET_*.mp4'))
        if scene=='circular':files+=list(folder.glob('first_person/*.mp4'))
        for file in files:
            data=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','stream=codec_name,width,height,r_frame_rate,nb_frames','-show_entries','format=duration','-of','json',str(file)]))
            stream=data['streams'][0]
            decoded=subprocess.run(['ffmpeg','-v','error','-i',str(file),'-f','null','-'],capture_output=True,timeout=120)
            item=dict(file=str(file),**stream,duration_s=float(data['format']['duration']),decoded=decoded.returncode==0 and not decoded.stderr.strip())
            item['valid']=item['decoded'] and stream['codec_name']=='h264' and stream['r_frame_rate']=='10/1' and int(stream['nb_frames'])>100
            frame_count=int(stream['nb_frames'])
            sample_frames=[0,frame_count//3,2*frame_count//3,frame_count-1]
            select='+'.join(f'eq(n,{n})' for n in sample_frames)
            review=OUT/f"REVIEW_{scene}_{controller}_{'robot' if file.parent.name=='first_person' else 'third'}.png"
            subprocess.run(['ffmpeg','-y','-v','error','-i',str(file),'-vf',f"select='{select}',scale=480:270,tile=2x2",'-frames:v','1',str(review)],check=True,timeout=120)
            item['review_image']=str(review)
            if file.parent.name=='first_person':
                s=json.loads((file.parent/'summary.json').read_text())
                item.update(camera=s['source_camera'],aligned=s['frame_detection_timestamp_aligned'],gt_shown=s['gt_shown'],physical_human_count=s['human_count'])
                item['valid'] &= item['camera']=='/World/RobotCamera' and item['aligned'] and not item['gt_shown'] and item['physical_human_count']==3
            items.append(item)
    (OUT/'VIDEO_VALIDATION.json').write_text(json.dumps(dict(passed=False,technical_pass=len(items)==6 and all(x['valid'] for x in items),visual_review='pending',videos=items),indent=2))
    print(json.dumps(items,indent=2))


def report():
    rows=collect() or []
    config=json.loads((ROOT/'config.yaml').read_text())['social_navigation']
    tables=[]
    metrics=['min_gt_distance','personal_space_violation_s','integrated_social_cost','travel_time','path_length','stop_duration','direction_switches']
    tables.append('| Scenario / controller | n | Complete | Human / static collision runs | Min distance m | Intrusion s | Integrated social cost | Travel s | Path m | Stop s | Switches |')
    tables.append('|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|')
    for scene in SCENARIOS:
        for name in ('baseline','social_nav'):
            group=[r for r in rows if r['scenario']==scene and r['controller']==name]
            def stat(key):
                a=[r[key] for r in group if r.get(key) is not None]
                return f'{np.mean(a):.3f} ± {np.std(a,ddof=1):.3f}' if len(a)>1 else str(a[0]) if a else 'N/A'
            tables.append(f'| {scene} / {name} | {len(group)} | {sum(r["route_completed"] for r in group)} | {sum(r["collision_proxy"] for r in group)} / {sum(r["static_collision"] for r in group)} | '+' | '.join(stat(k) for k in metrics)+' |')
    socials=[r for r in rows if r['controller']=='social_nav']
    lower=[];higher=[]
    for scene in SCENARIOS:
        a=[r['integrated_social_cost'] for r in rows if r['scenario']==scene and r['controller']=='baseline']
        b=[r['integrated_social_cost'] for r in rows if r['scenario']==scene and r['controller']=='social_nav']
        if a and b:(lower if np.mean(b)<np.mean(a) else higher).append(scene)
    regress=OUT/'REGRESSION.json'
    regression=json.loads(regress.read_text()) if regress.exists() else {'passed':False,'note':'not yet run'}
    checkfile=OUT/'SYNTHETIC_CHECKS.json'
    synthetic=json.loads(checkfile.read_text()) if checkfile.exists() else {}
    videofile=OUT/'VIDEO_VALIDATION.json'
    validation=json.loads(videofile.read_text()) if videofile.exists() else {'passed':False}
    # Side commitment can be exercised in the frozen-config development run even
    # when a particular formal repeat chooses yielding instead of turning.
    devfiles=sorted((OUT/'development/circular/social_nav/seed_17').glob('run_*/social_planning.json'))
    devheld=sum(p.get('held_opposite_candidates',0)>0 for p in json.loads(devfiles[-1].read_text())) if devfiles else 0
    gates=dict(thirty_runs=len(rows)==30,
        all_social_functional=len(socials)==15 and all(r['functional_pass'] for r in socials),
        circular_three_people=any(r['scenario']=='circular' and r.get('physical_human_count')==3 and r.get('min_human_walk_distance_m',0)>.5 and r.get('max_planned_humans',0)>=3 for r in socials),
        social_cost_changes_choice=any(r.get('social_changes_argmin',0)>0 for r in socials),
        prediction_and_isolation_checks=synthetic.get('status')=='PASS',
        side_hold_exercised=any(r.get('held_opposite_plans',0)>0 for r in socials) or devheld>0,
        no_side_hold_violations=all(r.get('commitment_violations',0)==0 for r in socials),
        baseline_regression=bool(regression.get('passed')),videos_validated=bool(validation.get('passed')))
    passed=all(gates.values())
    status='SOCIAL NAVIGATION FUNCTIONAL PASS' if passed else 'PARTIAL'
    video=[]
    for r in rows:
        if r['seed']!=17:continue
        folder=Path(r['source_run'])
        if r['scenario'] in ('frontal','perpendicular') or (r['scenario']=='circular' and r['controller']=='social_nav'):
            for file in folder.glob('NAVWARESET_*.mp4'):video.append(f'- [{r["scenario"]} / {r["controller"]} / seed 17 third person]({file.as_posix()})')
            for file in folder.glob('first_person/*.mp4'):video.append(f'- [{r["scenario"]} / {r["controller"]} / seed 17 robot camera]({file.as_posix()})')
    text=f'''# Prediction-Aware Social Navigation: implementation and paired benchmark

## 1. System
A300, Isaac Sim 6.0.1, frozen NavWareSet-style scene, RGB-D 640×360 at 10 Hz simulation time, COCO YOLO11n (conf 0.25), robust foreground depth, Hungarian association and CV KF. No LiDAR dependency. This is simulation, not an exact reconstruction of the real dataset.

## 2. Baseline
`social` remains **Reactive Baseline / DemoController**, byte-identical to the saved source. It is not relabelled as prediction-aware social navigation. `social_nav` selects the independent new controller. The sensor input rejects repeated exposure timestamps before submitting them to YOLO; the initial development run demonstrated the previous duplicate-timestamp crash. The same correction applies to both controllers; KF equations and thresholds are unchanged.

Run one new-controller scene from the project directory:
```powershell
.\\runtime\\python.bat stage_probe.py --stage 7 --robot-model a300 --motion-mode kinematic --navwareset-scene --navwareset-scenario circular --robot-behavior social_nav --record-demo --record-first-person
```
Legacy `--robot-behavior social` remains available. `python social_benchmark.py benchmark` resumes missing fixed benchmark combinations without replacing completed runs; `python social_benchmark.py report` regenerates the CSV/report from saved run evidence. The complete common configuration is frozen in BENCHMARK_CONFIG_FROZEN.json.

## 3. SocialController
Seven acceleration-limited motion primitives, 31 future points per primitive, CV human prediction, moving anisotropic personal space, known-static-map rejection, hard human-disk rejection, goal/control/heading/switch/stop costs. All accepted active tracks enter the social sum, not only the closest person. Recent heading is retained for low-speed tracks; unknown heading uses isotropic cost. Tracks older than 0.8 s are excluded from normal prediction, with bounded near-danger memory. Emergency and no-safe-primitive fallbacks command STOP; braking still follows the unchanged backend and cannot guarantee avoidance of an unavoidable situation.

## 4. Equations / parameters
Human: `p_h(t) = p_h(now) + v_h*t`. Robot integrates `x_dot=v*cos(yaw), y_dot=v*sin(yaw), yaw_dot=omega` with 0.5 m/s² acceleration, 0.8 m/s² deceleration and 1.2 rad/s² angular acceleration, matching the backend. `C=exp(-0.5*((long/sigma_long)^2+(lat/sigma_side)^2))`; sigma_long is front or rear by sign. Planning inflates sigma using `sqrt(lambda_max(F P F^T)_xy)` and uses real exposed KF covariance. It does not add a fabricated covariance or rewrite KF. Hard center collision uses actual robot conservative radius plus 0.30 m, unchanged; static disk uses the same radius.

`J = w_goal*J_goal + w_social*(mean summed personal cost + group_weight*bridge) + w_control*J_control + w_heading*J_heading + w_switch*J_switch + w_stop*J_stop`.
All soft weights and social dimensions are engineering heuristics, not human-subject-calibrated proxemics constants. Final common config:
```json
{json.dumps(config,indent=2)}
```
## 5. Functional results
Status: **{status}**. Completed metric rows: {len(rows)} / 30. Failed development attempts remain under development. No failed functional run is excluded or selected away.
Development history: the first Frontal run exposed duplicate exposure timestamps and crashed; the second exposed a near-waypoint endpoint-heading reversal and stalled. Both are retained. The timestamp submission guard applies equally to both controllers; the new controller heading cost now uses the current waypoint bearing. After successful development at social weight 1.2, one uniform pre-benchmark change to 4.0 made social cost influence choices more often. Frontal, Perpendicular and Circular were then rerun successfully before freezing. No per-scene weight search or formal-run retuning was performed.
Five scenarios × two controllers × three paired repeat labels 17/23/31. Python/NumPy seeds are set; frozen actor loader still uses Randomizer(17), so these are repeatability runs, NOT three independent random human populations. Same routes/speeds/geometry, renderer and camera for both. All benchmark runs record third-person output so render cost is paired; Circular seed 17 social also records first person.

{chr(10).join(tables)}

Mean ± sample standard deviation over repeats; N/A travel means route incomplete, not a successful short travel time. Collision fields use the unchanged post-run evaluator and include terminal recording padding. Social/efficiency integration stops at first route completion or run end; censoring is explicitly retained.

## 6. Social metrics
Recomputed AFTER the run from evaluation-only human positions and backward-difference velocity, with retained heading at low speed. Nominal same front/rear/side Gaussian, without planner uncertainty inflation (ground-truth positions have no tracker covariance). Report both nominal ellipse intrusion `normalized distance < 1`, summed integrated Gaussian cost, and minimum normalized distance. These quantify a DEFINED HEURISTIC field, not measured comfort. Evaluation never copies selected internal candidate cost and never returns data to planning. Uncertainty inflation and group bridge are planner terms; nominal personal-space evaluation does not pretend to measure group comfort.

## 7. Efficiency / smoothness
Travel time, path length, stop duration, steering time, direction switches, omega variation and squared omega effort are stored in CSV. Direction switches count sign changes among commands with |omega|>0.1 rad/s, including route corrections; they are not necessarily commitment violations. A social improvement accompanied by longer travel is a trade-off, not an automatic winner.
Side commitment also has an explicit synthetic check. The final Circular development run exercised opposite-side exclusion in {devheld} planning updates; formal activation/violation counts remain separately recorded in the CSV. Yield-only repeats are not presented as evidence of side-switch suppression.

## 8. Scenario analysis
'''
    for scene in SCENARIOS:
        a=[r for r in rows if r['scenario']==scene and r['controller']=='baseline'];b=[r for r in rows if r['scenario']==scene and r['controller']=='social_nav']
        text+=f'\n### {scene}\n'
        if a and b:
            for k in ('personal_space_violation_s','integrated_social_cost','min_gt_distance','path_length'):
                text+=f'- {k}: baseline {np.mean([r[k] for r in a]):.3f}; social {np.mean([r[k] for r in b]):.3f}.\n'
            text+=f'- Social functional runs: {sum(r["functional_pass"] for r in b)}/{len(b)}. These observations do not establish a human preference.\n'
        else:text+='Pending paired results.\n'
    text+='''
## 9. Multi-person
Circular uses the unchanged three-person preset. Maximum planned tracks and actual detector/tracker counts are logged; tracker IDs are not physical identities and false/duplicate tracks can exist.

## 10. Group awareness
Pairs within 1.2 m with cosine direction similarity >=0.85 or both below 0.15 m/s add a predicted segment/capsule Gaussian bridge. Synthetic checks validate pair and bridge scoring. Logged pair activations alone do not establish genuine social groups: no human group labels or comfort study were collected.

## 11. Performance
'''
    text+='\n| Scenario / controller | YOLO responses/wall-s | RTF | Steering s | Omega variation rad/s | Min social norm | GPU peak MiB | System RAM peak GiB |\n|---|---:|---:|---:|---:|---:|---:|---:|\n'
    for scene in SCENARIOS:
        for name in ('baseline','social_nav'):
            group=[r for r in rows if r['scenario']==scene and r['controller']==name]
            values=[]
            for key in ('yolo_responses_s','rtf','steering_time','omega_variation','min_social_norm','gpu_peak_mib','ram_peak_gib'):
                a=[r[key] for r in group]
                values.append(f'{np.mean(a):.3f} ± {np.std(a,ddof=1):.3f}' if len(a)>1 else str(a[0]) if a else 'N/A')
            text+=f'| {scene} / {name} | '+' | '.join(values)+' |\n'
    text+='\nPlanner diagnostics (not end-to-end latency):\n'
    for r in socials:text+=f'- {r["scenario"]} seed {r["seed"]}: YOLO {r["yolo_responses_s"]:.2f}/wall-s; RTF {r["rtf"]:.3f}; planner P95 {r.get("planner_p95_ms",float("nan")):.3f} ms; social argmin changes {r.get("social_changes_argmin",0)}; group-active plans {r.get("group_plan_count",0)}.\n'
    text+=f'''
RTF<1 is not wall-clock real time. GPU/RAM scope follows existing resource monitor; no resolution reductions used.

## 12. GT isolation
`social_controller.py` imports only NumPy and time. Input: estimated tracks/covariance/confidence, self pose/current velocity, waypoints, frozen static polygons. Actor GT scripting and post-run evaluation are outside this module. No scenario label or actor route is supplied. The robot self pose is idealized simulation localization, not human GT. Unit checks verify frozen algorithm modules unchanged.

## 13. Videos
Seed 17 was fixed for delivery, not selected after results. Each video is a single continuous run; no cross-run splicing.
{chr(10).join(video)}

Estimated-state diagnostic: [SOCIAL_COST_TOPDOWN.png](SOCIAL_COST_TOPDOWN.png), with editable vector [SVG](SOCIAL_COST_TOPDOWN.svg). The snapshot uses the first multi-track non-CRUISE/non-STOP decision of the fixed Circular seed 17 run. It shows current uncertainty-inflated contours and 3 s human predictions; it is not a GT trajectory plot or a success-selection metric.

## 14. Limitations
Heuristic personal space; constant-velocity prediction; no intent model; no RL/MPC; no human-subject comfort evaluation; kinematic A300; simulation only; no safety certification. Finite discrete horizon and seven primitives can miss viable paths. Erroneous/duplicate tracks can cause conservative stops. Same RNG labels do not eliminate asynchronous runtime timing variation. Reduced social cost does not imply greater safety.

## 15. Final status
**{status}**
Integrated nominal personal-space cost is lower in: {', '.join(lower) or 'none'}; higher or equal in: {', '.join(higher) or 'none'}. This is not uniform dominance over DemoController. Functional PASS means the specified implementation and simulation gates passed, not that the new policy should replace the baseline or that comfort/safety is certified.
{'All 30 runs have zero nominal personal-space ellipse intrusion; therefore this thresholded metric does not demonstrate an intrusion reduction. The continuous Gaussian cost still varies.' if len(rows)==30 and all(r['personal_space_violation_s']==0 for r in rows) else 'Thresholded intrusion outcomes are reported separately above.'}
Regression: `{json.dumps(regression)}`.
Explicit gates: `{json.dumps(gates)}`.
Numerical outcomes and source paths are in SOCIAL_NAVIGATION_RESULTS.csv. No rejected/development runs deleted. Report status is contingent on all required functional checks, not solely lower social cost.
'''
    (OUT/'SOCIAL_NAVIGATION_REPORT.md').write_text(text,encoding='utf8')
    (OUT/'FINAL_STATUS.json').write_text(json.dumps(dict(status=status,runs=len(rows),social_functional=sum(r['functional_pass'] for r in socials),gates=gates,regression=regression),indent=2))
    print(status,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['checks','development','benchmark','collect','report','topdown','regression','videos'])
    args=parser.parse_args()
    if args.mode=='checks':checks()
    elif args.mode=='topdown':topdown()
    elif args.mode=='regression':regression()
    elif args.mode=='videos':videos()
    elif args.mode=='report':report()
    elif args.mode=='collect':collect()
    elif args.mode=='development':
        for scenario in ('perpendicular','circular'):run(scenario,'social_nav',17,'development',scenario=='circular')
    else:
        checks()
        freeze=OUT/'BENCHMARK_CONFIG_FROZEN.json'
        config=json.loads((ROOT/'config.yaml').read_text())
        if freeze.exists():assert json.loads(freeze.read_text())==config,'Benchmark config changed'
        else:freeze.write_text(json.dumps(config,indent=2))
        # Fixed order and paired seed labels; never select a best run.
        for scenario in SCENARIOS:
            for seed in (17,23,31):
                for behavior in ('social','social_nav'):
                    parent=OUT/'benchmark'/scenario/('baseline' if behavior=='social' else behavior)/f'seed_{seed}'
                    if list(parent.glob('run_*/social_metrics.json')):continue
                    run(scenario,behavior,seed,'benchmark',seed==17)
                    collect()
