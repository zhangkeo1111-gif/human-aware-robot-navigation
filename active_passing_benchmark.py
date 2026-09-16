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

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['run','evaluate','figure']);p.add_argument('--scene',default='static_obstruction');p.add_argument('--repeat',type=int,default=17);p.add_argument('--phase',default='development');p.add_argument('--folder');a=p.parse_args()
    if a.mode=='run':run(a.scene,a.repeat,a.phase)
    elif a.mode=='evaluate':evaluate(a.folder)
    else:figure(a.folder)
