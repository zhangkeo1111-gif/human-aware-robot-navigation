"""Classic scene validation and paired evaluation. GT is post-run only."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from classic_single_scene import SCENARIOS,preset
from social_controller import personal_field,static_clearance

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'outputs/classic_single_pedestrian'
DEFINITIONS=dict(lateral_min_m=.5,sustained_side_s=1.,return_route_m=.3,
    side_deadband_m=.15,interaction_distance_m=3.,pass_longitudinal_window_m=2.,
    overtaking_ahead_m=.5,stop_speed_m_s=.03,timeout_s=70.,
    scope='Repeatability labels 17/23/31, not independent human populations',
    no_controller_tuning=True)

def read(p):return json.loads(Path(p).read_text())
def save(p,data):Path(p).write_text(json.dumps(data,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist()),encoding='utf8')

def visibility_sheet(folder,start=10.,end=17.,step=.5):
    from PIL import Image,ImageDraw
    folder=Path(folder);files=sorted((folder/'visibility_rgb').glob('*.png'))
    chosen=[min(files,key=lambda p:abs(float(p.stem)-t)) for t in np.arange(start,end+step/2,step)]
    sheet=Image.new('RGB',(4*320,((len(chosen)+3)//4)*200),'white');draw=ImageDraw.Draw(sheet)
    for i,f in enumerate(chosen):
        x=(i%4)*320;y=(i//4)*200;sheet.paste(Image.open(f).resize((320,180)),(x,y));draw.text((x+5,y+182),f.stem+' s',fill='black')
    file=folder/f'VISIBILITY_{start:g}_{end:g}.png';sheet.save(file);return file

def checks():
    frozen=['social_controller.py','demo_scene.py','navwareset_scene.py','robot_backend.py',
            'tracker.py','yolo_worker.py','walking_actor.py','config.yaml','first_person_video.py',
            'indoor_demo_scene.usd','navwareset_style_scene.usd','VERSION']
    for f in frozen:
        old=subprocess.check_output(['git','show','v0.3.0-social-nav:'+f],cwd=ROOT)
        actual=(ROOT/f).read_bytes()
        assert old==actual if f.endswith('.usd') else old.replace(b'\r\n',b'\n')==actual.replace(b'\r\n',b'\n'),f
    assert subprocess.check_output(['git','rev-parse','main'],cwd=ROOT,text=True).strip().startswith('acafebb')
    # Route-local lateral measurement is invariant under a shared rotation.
    p=np.array([[1.,.6],[2.,-.4]]);a=.7;R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    assert np.allclose((p@R.T)@np.array([-np.sin(a),np.cos(a)]),p[:,1])
    OUT.mkdir(parents=True,exist_ok=True)
    save(OUT/'CHECKS.json',dict(passed=True,frozen_files=frozen,route_local_rotation_test=True))

def evaluate(folder):
    folder=Path(folder);summary=read(folder/'summary.json');meta=read(folder/'meta.json')
    assert summary['camera_resolution']==[640,360] and summary['human_count']==1
    assert summary['classic']['gt_in_control'] is False
    allrows=read(folder/'demo_evaluation.json');end=next((i for i,r in enumerate(allrows) if r['route_done']),len(allrows)-1)
    rows=allrows[:end+1];n=len(rows);t=np.array([r['time'] for r in rows]);dt=np.diff(t,prepend=t[0])
    p=np.array([r['robot_position'][:2] for r in rows]);h=np.array([r['evaluation_people'][0][:2] for r in rows])
    assert all(len(r['evaluation_people'])==1 for r in rows)
    origin=np.array(meta['robot_start'][:2]);axis=np.array(meta['robot_waypoints'][-1])-origin;axis/=np.linalg.norm(axis)
    normal=np.array([-axis[1],axis[0]]);lateral=(p-origin)@normal;along=(p-origin)@axis
    relative=(h-p)@axis;distance=np.linalg.norm(p-h,axis=1)
    interaction=distance<=DEFINITIONS['interaction_distance_m'];indices=np.flatnonzero(interaction)
    first=int(indices[0]) if len(indices) else None;last=int(indices[-1]) if len(indices) else None
    phase=['PRE_INTERACTION' if first is None or i<first else 'INTERACTION' if i<=last else 'POST_INTERACTION' for i in range(n)]
    velocity=np.array([r['speed'] for r in rows]);omega=np.array([r['omega'] for r in rows])
    stop=(velocity<.03)&np.array([not r['route_done'] for r in rows])&(t>1.)
    signs=np.sign(lateral[np.abs(lateral)>.15]);side_switches=int(np.sum(signs[1:]!=signs[:-1]))
    turn=np.sign(omega[np.abs(omega)>.1]);direction_switches=int(np.sum(turn[1:]!=turn[:-1]))
    c=summary['configuration']['social_navigation'];fields=[];norms=[];heading=None
    hv=np.zeros_like(h);hv[1:]=np.diff(h,axis=0)/np.maximum(dt[1:,None],1e-9)
    for i in range(n):
        if np.linalg.norm(hv[i])>=c['heading_min_speed']:heading=np.arctan2(hv[i,1],hv[i,0])
        f,d=personal_field(p[i]-h[i],heading,c);fields.append(float(f));norms.append(float(d))
    norms=np.array(norms);radius=summary['demo']['geometric_collision_threshold']-.3
    geometry=read(folder/'occupancy_xy_points.json')['polygons'];clear=static_clearance(p,geometry,radius)
    collision=bool(distance.min()<radius+.3);static=bool(np.min(clear)<0);done=bool(rows[-1]['route_done'])
    safe=done and not collision and not static
    longest=0.;streak=0.;oldside=0
    for i in range(n):
        side=int(np.sign(lateral[i]));eligible=abs(lateral[i])>=.5 and abs(relative[i])<=2. and velocity[i]>.05
        streak=streak+dt[i] if eligible and side==oldside else dt[i] if eligible else 0.
        longest=max(longest,streak);oldside=side
    passed=bool(np.any(relative<-.5));recovered=abs(lateral[-1])<=.3
    active=bool(safe and passed and recovered and longest>=1.)
    cross_side=(h-origin)@normal
    crossing=np.flatnonzero(cross_side[:-1]*cross_side[1:]<=0)
    cross_index=int(crossing[0]+1) if len(crossing) else None
    slowed=np.array([r['state'] in ('SLOW','STOP') and not r['route_done'] for r in rows])
    reacted=np.array([r['state']!='CRUISE' and not r['route_done'] for r in rows])
    resume=bool(any(velocity[i]>.25 and any(reacted[:i]) for i in range(n)))
    yielded=bool(safe and cross_index is not None and np.any(slowed[:cross_index]&(along[:cross_index]<((h[cross_index]-origin)@axis))) and np.any(velocity[cross_index:]>.25))
    overtook=bool(active and relative[0]>.5 and np.any(relative<-.5))
    pass_end=next((i for i in range(n) if first is not None and i>=first and relative[i]<-.5),None)
    passing_window=np.abs(relative)<=.5
    ds=read(folder/'detections.json');det=next((d for d in ds if d['boxes']),None)
    detection=det.get('exposure_elapsed_s') if det else None
    response=det.get('response_elapsed_s') if det else None
    first_non_cruise=next((r['time'] for r in rows if r['state']!='CRUISE' and not r['route_done']),None)
    plans=read(folder/'social_planning.json') if (folder/'social_planning.json').exists() else []
    plan_times=np.array([x['time']-meta['initial_sim_time'] for x in plans])
    def human_response(r):
        k=int(np.searchsorted(plan_times,r['time']+1e-7,side='right')-1)
        stale_guard=k>=0 and bool(plans[k].get('stale_guard'))
        return r['state']!='CRUISE' and not r['route_done'] and (bool(r['estimated_risks']) or stale_guard)
    reaction=next((r['time'] for r in rows if human_response(r)),None)
    visibility=read(folder/'VISIBILITY_REVIEW.json') if (folder/'VISIBILITY_REVIEW.json').exists() else {}
    visual=visibility.get('first_visible_time')
    blind_ok=bool(safe and resume and visibility.get('early_occlusion_confirmed') and visual is not None and detection is not None
                  and detection>=visual-.051 and reaction is not None and response is not None and reaction>=response-1e-6)
    scene=meta['scenario'];behavior=meta['robot_behavior']
    result=dict(scenario=scene,controller='diagnostic' if summary['classic']['diagnostic'] else 'baseline' if behavior=='social' else 'social_nav',
        repeat=int(folder.parent.name.split('_')[-1]),status='VALID',route_completed=done,collision_proxy=collision,static_collision=static,
        min_gt_distance=float(distance.min()),travel_time=float(t[-1]) if done else None,observed_duration=float(t[-1]),
        path_length=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),stop_count=int(np.sum(stop&~np.r_[False,stop[:-1]])),
        stop_duration=float(dt@stop),max_lateral_displacement=float(np.max(np.abs(lateral))),
        passing_side='NONE' if not len(signs) else 'LEFT' if lateral[np.argmax(np.abs(lateral))]>0 else 'RIGHT',
        side_switches=side_switches,sustained_moving_pass_s=float(longest),return_to_route=bool(recovered),
        route_reference_heading_rad=float(np.arctan2(axis[1],axis[0])),
        interaction_start_s=float(t[first]) if first is not None else None,
        interaction_end_s=float(t[last]) if last is not None else None,
        passing_clearance_m=float(distance[passing_window].min()-radius-.3) if passing_window.any() else None,
        overtaking_duration_s=float(t[pass_end]-t[first]) if scene=='overtaking' and pass_end is not None else None,
        active_passing_success=active if scene in ('headon','static_obstruction') else None,
        yielding_success=yielded if scene=='crossing' else None,overtaking_success=overtook if scene=='overtaking' else None,
        functional_pass=bool(safe and summary['yolo']['samples']>0),
        scenario_pass=active if scene=='static_obstruction' else overtook if scene=='overtaking' else blind_ok if scene=='blind_corner' else safe and resume if scene=='crossing' else safe,
        behavior_label='ACTIVE-PASSING' if active else 'YIELD-ONLY' if safe and stop.any() else 'ROUTE-COMPLETE' if safe else 'FAIL',
        integrated_social_cost=float(dt@np.array(fields)),min_social_norm=float(norms.min()),
        personal_space_violation_s=float(dt@(norms<1)),social_lt_1_5_s=float(dt@(norms<1.5)),
        direction_switches=direction_switches,omega_variation=float(np.abs(np.diff(omega)).sum()),
        yolo_responses_s=summary['yolo']['effective_fps'],rtf=summary['real_time_factor'],resume=resume,
        first_detection_time=detection,first_detection_response_time=response,first_reaction_time=reaction,
        first_non_cruise_time=first_non_cruise,
        detection_to_reaction_latency=reaction-detection if detection is not None and reaction is not None else None,
        first_visible_time=visual,visibility_review_complete=bool(visibility),source_run=str(folder))
    with (folder/'interaction_phases.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['simulation_time','phase','route_along_m','route_lateral_m','human_relative_along_m','speed','state'])
        w.writerows(zip(t,phase,along,lateral,relative,velocity,[r['state'] for r in rows]))
    save(folder/'classic_metrics.json',result);return result

def run(scene,behavior='social',repeat=17,diagnostic=False):
    phase='development' if diagnostic else 'benchmark';name='diagnostic' if diagnostic else 'baseline' if behavior=='social' else behavior
    parent=OUT/phase/scene/name/f'seed_{repeat}';existing=sorted(parent.glob('run_*'))
    if existing:
        valid=[p for p in existing if (p/'summary.json').exists() and not (p/'failure.txt').exists()
               and not (p/'INVALID_GEOMETRY.json').exists() and not (p/'INVALID_INFRASTRUCTURE.json').exists()]
        if valid:return evaluate(valid[-1])
        if not all((p/'INVALID_INFRASTRUCTURE.json').exists() or (p/'INVALID_GEOMETRY.json').exists() for p in existing):
            raise RuntimeError(f'Infrastructure failure needs inspection; not silently replaced: {parent}')
    log=OUT/f'{phase}_{scene}_{name}_{repeat}_run_{len(existing)+1:02d}.log'
    command=[str(ROOT/'runtime/python.bat'),'stage_probe.py','--stage','7','--robot-model','a300','--motion-mode','kinematic',
        '--classic-scene','--classic-scenario',scene,'--robot-behavior',behavior,'--seed',str(repeat),'--run-phase',phase,
        '--headless','--seconds','300','--record-demo']
    if diagnostic:command+=['--classic-diagnostic']
    if scene in ('headon','blind_corner') and (diagnostic or repeat==17):command+=['--record-first-person']
    print('START',scene,name,repeat,flush=True)
    with log.open('w') as f:
        child=subprocess.Popen(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        try:child.wait(timeout=540)
        except subprocess.TimeoutExpired:
            subprocess.run(['taskkill','/PID',str(child.pid),'/T','/F'],capture_output=True)
            raise RuntimeError(f'INVALID_INFRASTRUCTURE timeout: {log}')
    folder=sorted(parent.glob('run_*'))[-1]
    if child.returncode or (folder/'failure.txt').exists() or not (folder/'summary.json').exists():
        save(folder/'INVALID_INFRASTRUCTURE.json',dict(reason='Nonzero exit, missing summary or failure.txt',exit_code=child.returncode,log=str(log)))
        raise RuntimeError(f'INVALID_INFRASTRUCTURE: {folder}')
    result=evaluate(folder);print('DONE',json.dumps(result),flush=True);return result

def geometry_validation():
    checks();results=[]
    for scene in SCENARIOS:
        r=run(scene,diagnostic=True);folder=Path(r['source_run']);rows=read(folder/'demo_evaluation.json')
        h=np.array([x['evaluation_people'][0][:2] for x in rows]);p=np.array([x['robot_position'][:2] for x in rows])
        conditions=dict(real_conflict=r['min_gt_distance']<.90278,one_human=True,
            human_motion=float(np.linalg.norm(np.diff(h,axis=0),axis=1).sum()))
        if scene=='headon':conditions['opposite_motion']=h[-1,0]<h[0,0]-5 and p[-1,0]>p[0,0]+8
        if scene=='crossing':conditions['crossed_centerline']=h[0,1]<-2 and h[-1,1]>2
        if scene=='static_obstruction':
            # Official idle animation has root sway, but must never vacate the route.
            conditions['remained_in_obstruction_zone']=bool(np.max(np.abs(h[:,0]-4.5))<1. and np.max(np.abs(h[:,1]))<.5)
        if scene=='overtaking':conditions['robot_catches_human']=bool(np.any(p[:,0]-h[:,0]>.5))
        if scene=='overtaking':conditions['human_continuous_forward_motion']=bool(h[-1,0]-h[0,0]>4.)
        if scene=='blind_corner':conditions['rgb_occlusion_review']=bool(r['visibility_review_complete'] and r['first_visible_time'] is not None)
        results.append(dict(scenario=scene,source=r['source_run'],checks=conditions))
    save(OUT/'GEOMETRY_VALIDATION.json',results)

def freeze():
    checks();geometry=read(OUT/'GEOMETRY_VALIDATION.json')
    assert len(geometry)==5
    assert all(all(v for k,v in r['checks'].items() if k!='human_motion') for r in geometry),'Geometry validation incomplete'
    data=dict(base='v0.3.0-social-nav',commit='acafebb',scenarios={s:preset(s) for s in SCENARIOS},
        common=read(ROOT/'config.yaml'),camera=dict(resolution=[640,360],HFOV=106.26,clipping=[.05,100.]),
        success_definitions=DEFINITIONS,repeats=[17,23,31],controller_changes=False,
        implementation_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip())
    file=OUT/'CLASSIC_SINGLE_CONFIG_FROZEN.json'
    if file.exists():assert read(file)==data
    else:save(file,data)

def benchmark():
    frozen=read(OUT/'CLASSIC_SINGLE_CONFIG_FROZEN.json');checks()
    assert frozen['scenarios']=={s:preset(s) for s in SCENARIOS}
    assert frozen['common']==read(ROOT/'config.yaml') and frozen['success_definitions']==DEFINITIONS
    for scene in SCENARIOS:
        for repeat in (17,23,31):
            for behavior in ('social','social_nav'):run(scene,behavior,repeat)
    collect()

def collect():
    files=sorted((OUT/'benchmark').glob('*/*/seed_*/run_*/summary.json'))
    rows=[evaluate(f.parent) for f in files if not (f.parent/'INVALID_INFRASTRUCTURE.json').exists()]
    for r in rows:
        meta=read(Path(r['source_run'])/'meta.json');expected=preset(r['scenario'])
        for key in ('humans','robot_start','robot_waypoints','blind_wall','human_navigation'):
            assert meta[key]==expected[key],(r['source_run'],key)
    with (OUT/'CLASSIC_SINGLE_PEDESTRIAN_RESULTS.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    return rows

def figures():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':['Arial','DejaVu Sans'],'font.size':12,
        'axes.spines.top':False,'axes.spines.right':False,'legend.frameon':False,'svg.fonttype':'none'})
    for scene,label in [('headon','HEADON'),('static_obstruction','STATIC')]:
        fig,axes=plt.subplots(2,2,figsize=(12,8),gridspec_kw={'height_ratios':[1.4,1]})
        for j,controller in enumerate(('baseline','social_nav')):
            folder=OUT/'benchmark'/scene/controller/'seed_17/run_01';rows=read(folder/'demo_evaluation.json');m=read(folder/'classic_metrics.json')
            end=next((i for i,r in enumerate(rows) if r['route_done']),len(rows)-1);rows=rows[:end+1]
            p=np.array([r['robot_position'][:2] for r in rows]);h=np.array([r['evaluation_people'][0][:2] for r in rows]);t=np.array([r['time'] for r in rows])
            stop=np.array([r['speed']<.03 and not r['route_done'] and r['time']>1 for r in rows])
            near=np.linalg.norm(p-h,axis=1)<=3.;ax=axes[0,j]
            ax.plot(p[:,0],p[:,1],color='#0F4D92',lw=2.5,label='Robot trajectory')
            ax.plot(h[:,0],h[:,1],'--',color='#B64342',lw=2,label='Human trajectory (evaluation)')
            ax.scatter(p[stop,0],p[stop,1],s=15,color='#272727',marker='x',label='Robot stopped')
            ax.scatter(p[near,0],p[near,1],s=10,color='#42949E',alpha=.15,label='Interaction region')
            ax.scatter(*h[0],s=80,facecolors='none',edgecolors='#B64342')
            ax.axhline(0,color='#aaa',lw=.8);ax.set(xlim=(-.3,9.5),ylim=(-2,2),xlabel='World X (m)',ylabel='World Y (m)')
            ax.set_aspect('equal');ax.set_title(f'{controller} | {m["behavior_label"]}\nmax lateral={m["max_lateral_displacement"]:.2f} m')
            values=list(csv.DictReader((folder/'interaction_phases.csv').open()))
            lat=np.array([float(r['route_lateral_m']) for r in values]);ax=axes[1,j]
            ax.plot(t,lat,color='#0F4D92',lw=2);ax.scatter(t[stop][::12],lat[stop][::12],s=12,color='#272727',marker='x')
            ax.axhline(.5,color='#aaa',ls='--');ax.axhline(-.5,color='#aaa',ls='--')
            ax.set(xlabel='Simulation time (s)',ylabel='Route-local lateral (m)',ylim=(-2,2));ax.grid(alpha=.15)
        common_end=max(ax.lines[0].get_xdata()[-1] for ax in axes[1])
        for ax in axes[1]:ax.set_xlim(0,common_end)
        handles,labels=axes[0,0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=2)
        fig.suptitle(f'Classic {scene.replace("_"," ")}: fixed repeat 17 (no best-run selection)',fontsize=16)
        fig.tight_layout(rect=(0,.10,1,.94));fig.savefig(OUT/f'CLASSIC_{label}_TOPDOWN.png',dpi=300);plt.close(fig)

def video_checks():
    files=[]
    for s in SCENARIOS:
        for c in ('baseline','social_nav'):
            folder=OUT/'benchmark'/s/c/'seed_17/run_01'
            files+=list(folder.glob('CLASSIC_*.mp4'))+list((folder/'first_person').glob('CLASSIC_*.mp4'))
    results=[]
    for i,file in enumerate(files):
        data=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','stream=codec_name,width,height,r_frame_rate,nb_frames','-show_entries','format=duration','-of','json',str(file)]))
        stream=data['streams'][0];duration=float(data['format']['duration'])
        decoded=subprocess.run(['ffmpeg','-v','error','-i',str(file),'-f','null','-'],capture_output=True,timeout=120)
        n=int(stream['nb_frames']);review_indices=[0,n//4,n//2,3*n//4]
        exposure_info={}
        if file.parent.name=='first_person':
            manifest=read(file.parent/'summary.json')['frames_manifest']
            span=manifest[-1]['elapsed']-manifest[0]['elapsed']+.1
            exposure_info=dict(recorded_simulation_span_s=span,playback_duration_ratio=duration/span,
                timing_note='Unchanged first-person exporter encodes returned exposures at 10 fps; missed camera ticks compress playback. Use logged simulation timestamps for event timing.')
            seen=[k for k,f in enumerate(manifest) if f['person_count']]
            if seen:review_indices=sorted(set([0,seen[0],seen[len(seen)//2],3*n//4]))
        select='+'.join(f'eq(n,{k})' for k in review_indices)
        target=OUT/f'VIDEO_REVIEW_{i+1:02d}.png'
        subprocess.run(['ffmpeg','-y','-v','error','-i',str(file),'-vf',f"select='{select}',scale=480:270,tile=2x2",'-frames:v','1',str(target)],check=True,timeout=90)
        ok=stream['codec_name']=='h264' and stream['r_frame_rate']=='10/1' and decoded.returncode==0 and not decoded.stderr
        results.append(dict(file=str(file),duration=duration,**stream,**exposure_info,technical_pass=bool(ok),contact_sheet=str(target)))
    save(OUT/'VIDEO_VALIDATION.json',dict(technical_pass=all(x['technical_pass'] for x in results),visual_review='pending',videos=results))

def report():
    checks()
    rows=collect();assert len(rows)==30 and len({(r['scenario'],r['controller'],r['repeat']) for r in rows})==30
    paired_motion=[]
    for scene in SCENARIOS:
        for repeat in (17,23,31):
            pair=[next(r for r in rows if r['scenario']==scene and r['repeat']==repeat and r['controller']==c)
                  for c in ('baseline','social_nav')]
            streams=[read(Path(r['source_run'])/'demo_evaluation.json') for r in pair]
            ts=[np.array([r['time'] for r in stream]) for stream in streams]
            hs=[np.array([r['evaluation_people'][0][:2] for r in stream]) for stream in streams]
            common=ts[0]<=min(ts[0][-1],ts[1][-1]);target=ts[0][common]
            aligned=np.column_stack([np.interp(target,ts[1],hs[1][:,i]) for i in range(2)])
            paired_motion.append(float(np.max(np.linalg.norm(hs[0][common]-aligned,axis=1))))
    def avg(group,k):
        x=[r[k] for r in group if r.get(k) is not None];return f'{np.mean(x):.3f}' if x else 'N/A'
    text=['# Classic Single-Pedestrian Benchmark',
        '## 1. Purpose','Test the existing controllers on explicit single-person conflicts, without changing controller behavior. Collision avoidance and active passing are different outcomes.',
        '## 2. Frozen System','Base main/tag v0.3.0-social-nav = acafebb. Branch feature/classic-single-pedestrian. VERSION remains 0.3.0. No release or merge. A300 kinematic, robot RGB-D 640x360, existing mount/HFOV/clipping, YOLO11n COCO conf 0.25, robust depth, Hungarian/CV KF, DemoController and v0.3 SocialController are unchanged. No v2 planner or LiDAR.',
        'Single-run CLI: `runtime\\python.bat stage_probe.py --stage 7 --robot-model a300 --motion-mode kinematic --classic-scene --classic-scenario headon --robot-behavior social_nav --record-demo --record-first-person --headless`. The classic CLI does not use `--navwareset-scene`; only its existing fixed-asset loader adapter is shared, not its geometry. Batch order: `python classic_benchmark.py geometry`, manual raw-camera visibility review, `freeze`, `benchmark`, `figures`, `videos`, `report`. Existing completed formal runs are reused, never replaced on algorithm failure.',
        '## 3. Scene','Independent 12 x 6 m open arena: x in [-1,11], y in [-3,3]. Robot (0,0) to (9,0). Perimeter walls only, except Blind Corner adds a 2.8 x 0.15 m wall centered at (1.4,0.825), height 2.6 m. One fixed medical-person asset per run. Standing idle is intentional for Static Obstruction; its animation root sways, but stays within 1 m longitudinal and 0.5 m lateral of the fixed obstruction position.',
        '## 4. Scenario Definitions','All human motion is prescribed and never reads robot state. Exact definitions in CLASSIC_SINGLE_CONFIG_FROZEN.json.',
        '| Scenario | Human start | Goal | Speed m/s | Delay s |','|---|---|---|---:|---:|']
    for s in SCENARIOS:
        h=preset(s)['humans'][0];text.append(f'| {s} | {h["start"][:2]} | {h["goals"][0][:2]} | {h["speed"]:.2f} | {h["delay"]:.3f} |')
    text+=['Overtaking begins at 1.8 m, not 3 m: at nominal speeds 0.30/0.20 m/s the catch point is x=5.4 m, inside the 9 m route. This geometry choice was made before controller benchmarking, not tuned from controller results.',
        'Native human setup is shared by all classic presets: complete-arena navmesh; dynamic robot excluded from the static bake; actor auto/obstacle avoidance disabled; blocked-speed setting reduced from native 30 to 5 so requested 0.20 m/s does not abort. Actual speed was checked from logged motion. These changes affect only new scene actor infrastructure, not robot control or any historical scene.',
        '## 5. Evaluation Definitions',
        '- Collision is the unchanged XY disk proxy: robot conservative radius plus 0.30 m human radius, not PhysX contact or a safety certificate. Static collision uses the unchanged disk-map clearance.',
        '- Route completion uses each frozen controller\'s goal logic. Timeout is 70 simulation seconds; a timeout is retained as failure, not discarded. Metrics end at completion or timeout, excluding terminal video padding.',
        '- Route-local axis is start-to-goal direction; lateral is its signed perpendicular projection, not hard-coded world Y. Interaction begins/ends at the first/last distance <=3 m; phase labels are evaluation-only.',
        '- Active passing: collision-free completion, human becomes >=0.5 m behind along the route, recovered lateral <=0.3 m, and >=1 s continuous same-side motion with |lateral|>=0.5 m, speed>0.05 m/s and |relative longitudinal distance|<=2 m. STOP-only never qualifies.',
        '- Overtaking also requires starting >=0.5 m behind the human and becoming >=0.5 m ahead. Crossing yielding requires slowing/stopping before the crossing point and resuming >0.25 m/s after the human crosses.',
        '- Passing clearance is minimum center distance minus the same robot/human disk radii while longitudinal separation is <=0.5 m; a negative value means proxy overlap. Overtaking duration runs from interaction onset until the robot first becomes >=0.5 m ahead. Side switches count changes of signed lateral beyond +/-0.15 m; direction switches use angular speed beyond +/-0.10 rad/s; omega variation is total absolute step-to-step angular-speed change. Stop duration integrates actual speed <0.03 m/s after the first second, excluding goal stop.',
        '- Blind Corner uses saved raw camera frames for visibility review. Detection is exposure time, reaction is controller time; both exposure-to-reaction and response arrival time are reported. Visibility is never substituted with a GT ray time.',
        '- Repeat labels 17/23/31 are repeatability runs, not independent populations. All paired routes, speeds, delay, start and goal are identical. Asynchronous perception may differ. No failed algorithm runs are replaced.']
    for number,s in enumerate(SCENARIOS,6):
        text += [f'## {number}. {s.replace("_"," ").title()} Results']
        if s in ('headon','static_obstruction'):
            name='HEADON' if s=='headon' else 'STATIC'
            text.append(f'![Fixed repeat 17 trajectories and lateral displacement](CLASSIC_{name}_TOPDOWN.png)')
        for c in ('baseline','social_nav'):
            g=[r for r in rows if r['scenario']==s and r['controller']==c]
            capability='active_passing_success' if s in ('headon','static_obstruction') else 'yielding_success' if s=='crossing' else 'overtaking_success' if s=='overtaking' else 'scenario_pass'
            text.append(f'- {c}: functional {sum(r["functional_pass"] for r in g)}/3; scenario PASS {sum(r["scenario_pass"] for r in g)}/3; {capability} {sum(r[capability] is True for r in g)}/3; labels {[r["behavior_label"] for r in g]}.')
        if s=='blind_corner':
            text+=['| Controller/repeat | First visible | First detection | Detection response | First human response | Exposure-to-response s |','|---|---:|---:|---:|---:|---:|']
            for r in [r for r in rows if r['scenario']==s]:
                text.append(f'| {r["controller"]}/{r["repeat"]} | '+' | '.join('N/A' if r[k] is None else f'{r[k]:.3f}' for k in ('first_visible_time','first_detection_time','first_detection_response_time','first_reaction_time','detection_to_reaction_latency'))+' |')
        if s in ('crossing','blind_corner'):
            text+=['Post-run perception diagnostic (not a changed success gate):',
                '| Controller/repeat | Last person exposure s | Person detections after scripted motion starts | KF track available during GT-labelled interaction |',
                '|---|---:|---:|---:|']
            for r in [r for r in rows if r['scenario']==s]:
                f=Path(r['source_run']);ds=[d for d in read(f/'detections.json') if d['boxes']]
                active=[x for x in read(f/'demo_evaluation.json') if x['evaluation_min_distance']<=3 and not x['route_done']]
                last='N/A' if not ds else f'{ds[-1]["exposure_elapsed_s"]:.3f}'
                after=sum(d['exposure_elapsed_s']>=preset(s)['humans'][0]['delay'] for d in ds)
                coverage=sum(bool(x['estimated_tracks']) for x in active)/len(active) if active else 0.
                text.append(f'| {r["controller"]}/{r["repeat"]} | {last} | {after} | {coverage:.1%} |')
            text.append('Evaluation-extractor correction, documented after formal runs began: the initial extractor called any non-CRUISE state a reaction. That incorrectly included goal slowdowns. The original event is retained as first_non_cruise_time; first_reaction_time now additionally requires logged estimated human risk or an active causal stale-human guard. No GT is used for this response attribution, no scenario/controller parameter changed, and this does not turn any collision failure into a pass. Missing critical-interval detections/tracks mean the result cannot be attributed solely to planner capability.')
    text+=['## 11. Baseline vs v0.3','Arithmetic means of three runs; completion-only travel means are censored by failure, so completion count and observed duration must be read together.',
        '| Scene | Controller | Complete | Collision | Min distance m | Travel s | Observed s | Lateral m | Stop s | Social cost |','|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for s in SCENARIOS:
        for c in ('baseline','social_nav'):
            g=[r for r in rows if r['scenario']==s and r['controller']==c]
            text.append(f'| {s} | {c} | {sum(r["route_completed"] for r in g)}/3 | {sum(r["collision_proxy"] for r in g)}/3 | '+' | '.join(avg(g,k) for k in ('min_gt_distance','travel_time','observed_duration','max_lateral_displacement','stop_duration','integrated_social_cost'))+' |')
    text+=[f'Paired human-motion audit over common simulation timestamps: maximum XY difference across all 15 controller pairs = {max(paired_motion):.6f} m. Preset, robot start/goal, human speed/delay, static geometry and native-human settings are checked identical.',
        '## 12. Social Metrics','Nominal ellipses remain front/rear/side 1.2/0.7/0.8 m. Integrated social cost and normalized distance <1.5 duration are heuristic proxemic metrics, not human comfort labels. A larger minimum distance alone does not prove better social behavior.',
        '| Scene | Controller | Min normalized distance | Time norm <1.5 s | Time norm <1 s | Turn-direction switches | Omega total variation rad/s |',
        '|---|---|---:|---:|---:|---:|---:|']
    for s in SCENARIOS:
        for c in ('baseline','social_nav'):
            g=[r for r in rows if r['scenario']==s and r['controller']==c]
            text.append(f'| {s} | {c} | '+' | '.join(avg(g,k) for k in ('min_social_norm','social_lt_1_5_s','personal_space_violation_s','direction_switches','omega_variation'))+' |')
    text+=['## 13. Videos','Fixed repeat 17 is used for every showcased video; no best-run selection or stitched runs. Third-person for both controllers in all five scenes; robot-camera first-person for Head-on and Blind Corner. First-person boxes use matching exposure IDs, never GT. Encoded 10 fps is not wall-clock throughput. The unchanged first-person exporter encodes returned camera exposures at 10 fps; missed capture ticks can compress playback. VIDEO_VALIDATION.json reports video duration versus recorded simulation span. Event timings and evaluation use simulation logs, not the video player clock.']
    for s in SCENARIOS:
        for c in ('baseline','social_nav'):
            folder=OUT/'benchmark'/s/c/'seed_17/run_01'
            for file in list(folder.glob('CLASSIC_*.mp4'))+list((folder/'first_person').glob('CLASSIC_*.mp4')):
                text.append(f'- [{s} / {c} / {file.name}]({file.as_posix()})')
    text+=['## 14. Performance','Same third-person rendering for paired runs. First-person recording is enabled symmetrically for repeat 17 Head-on/Blind Corner. Timings describe this workstation under its actual load, not a controlled isolated hardware comparison.',
        '| Scene | Controller | YOLO responses / wall-s | RTF |','|---|---|---:|---:|']
    for s in SCENARIOS:
        for c in ('baseline','social_nav'):
            g=[r for r in rows if r['scenario']==s and r['controller']==c];text.append(f'| {s} | {c} | {avg(g,"yolo_responses_s")} | {avg(g,"rtf")} |')
    text+=['## 15. GT Isolation','Only estimated tracks, robot self pose, route and static map reach the controllers. Human GT, interaction phases, passing success and visibility review are post-run evaluation. Human scripts use only own preset and simulation clock.',
        '## 16. Limitations','Kinematic A300; preset single-human routes; heuristic proxemics; no human intent model; no human-subject evaluation; simulation only; no safety certification. Camera sampling and imperfect detection/association can affect controller behavior. Agent arrival tolerance means actual paths are validated from logged motion, not presumed exact from requested endpoints.',
        '## 17. Conclusion']
    v=[r for r in rows if r['controller']=='social_nav'];passing=sum(r['active_passing_success'] is True for r in v)
    text.append(f'v0.3 active passing successes across Head-on/Static: {passing}/6. Overtaking successes: {sum(r["overtaking_success"] is True for r in v)}/3. These are observed capabilities in this suite, not general claims.')
    text.append('The current v0.3 does not demonstrate reliable active social passing in this benchmark; its behavior must be characterized from the yielding/stopping and failure evidence above.' if passing<6 else 'Active passing is demonstrated in these tested repeats only; generalization remains untested.')
    text += [
        'Direct answers for this frozen suite:',
        '1. Head-on: both controllers brake/stop but incur collision-proxy overlap in all three repeats. v0.3 has zero lateral displacement; it does not execute offset-pass-recover. Stopping is not safe yielding when the prescribed human continues straight into the robot.',
        '2. Static obstruction: both controllers stop until the 70 s timeout. Baseline moves laterally about 0.46 m but never completes the pass; v0.3 has zero lateral displacement. Neither navigates around the standing person.',
        '3. Crossing: neither controller yields successfully; all repeats collide. Detections cease before the human begins crossing, and there are no exported tracks in the conflict interval. This is an end-to-end perception/control failure, not an isolated planner comparison.',
        '4. Overtaking: v0.3 safely follows to the goal in all three repeats, but never gets ahead of the human and never creates lateral offset. Baseline times out. The generic YIELD-ONLY label for v0.3 in the CSV denotes stopping/following without passing here, not successful overtaking.',
        '5. Blind corner: initial visual occlusion is confirmed, followed by valid person detections around 6.3 s. The person subsequently leaves the forward view before moving across the route. Neither controller produces a logged human-risk response, and all repeats collide. Goal slowdown is not counted as pedestrian reaction.',
        'Overall: current v0.3 is primarily prediction-aware speed control/stopping/following, not demonstrated active social passing. Reliable yielding under all conflicts is also not established. Safe goal completion is demonstrated only for the overtaking/following setup, without overtaking.',
        'Next-step evidence, not changes made in this task: separate loss of front-camera support in lateral encounters from the inability to generate sustained lateral passing when a track is available. No controller, camera or human-route tuning was performed after the scenario freeze.'
    ]
    invalid=list(OUT.glob('**/INVALID_INFRASTRUCTURE.json'))+list(OUT.glob('**/INVALID_GEOMETRY.json'))
    text+=['## Infrastructure and pre-benchmark geometry record']+[f'- {f.relative_to(OUT)}: {read(f)}' for f in invalid]
    (OUT/'CLASSIC_SINGLE_PEDESTRIAN_REPORT.md').write_text('\n'.join('\n'+line+'\n' if line.startswith('#') else line for line in text),encoding='utf8')
    save(OUT/'FINAL_STATUS.json',dict(formal_runs=len(rows),functional_passes=sum(r['functional_pass'] for r in rows),
        baseline_active_pass=sum(r['active_passing_success'] is True for r in rows if r['controller']=='baseline'),
        v03_active_pass=passing,release=False,main_unchanged=True))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['geometry','freeze','benchmark','collect','checks','figures','videos','report']);a=p.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    {'geometry':geometry_validation,'freeze':freeze,'benchmark':benchmark,'collect':collect,'checks':checks,'figures':figures,'videos':video_checks,'report':report}[a.mode]()
