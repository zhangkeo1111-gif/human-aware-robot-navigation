"""Independent NavWareSet-inspired arena; GT is confined to scripting/evaluation."""
import csv
import json
import numpy as np
from demo_scene import DemoController

SCENARIOS = ('frontal','obstruction','blind_corner','perpendicular','circular')
GRS_EYE = [-.7,-1.65,2.17]
GRS_TARGET = [5.,.15,.65]

def preset(name):
    def person(start,goals,delay=0):
        return dict(start=[*start,0.],goals=[[*p,0.] for p in goals],speed=.65,delay=delay)
    people={
      'frontal':[person([6.,0.],[[3.,0.],[3.,-1.4],[.3,-1.4]])],
      'obstruction':[person([3.,0.],[[3.,-1.4],[1.,-1.4]],8.)],
      'blind_corner':[person([4.8,4.1],[[4.8,-1.3],[1.,-1.3]],7.)],
      'perpendicular':[person([4.8,1.5],[[4.8,-1.3],[1.,-1.3]],8.)],
      'circular':[person([5.,1.5],[[5.,-1.3],[1.,-1.3]],5.),
                  person([6.,-1.3],[[4.5,.6],[4.8,3.8]],3.),
                  person([7.,0.],[[3.,0.],[2.,1.3]],0.)]}
    return dict(scenario=name,robot_start=[0.,0.,0.],robot_waypoints=[[4.,0.],[8.5,0.]],
                humans=people[name],random_seed=17,scene_dimensions={'main_length_m':10.7,'main_width_m':3.9,'branch_width_m':2.2,'branch_length_m':2.85},
                grs_camera={'eye':GRS_EYE,'target':GRS_TARGET,'resolution':[960,540],'rate_hz':10},enable_grs_lidar=False)

def build(stage,path):
    from pxr import Usd,UsdGeom,UsdPhysics,Gf
    UsdGeom.Imageable(stage.GetPrimAtPath('/World/defaultGroundPlane')).MakeInvisible()
    scene=Usd.Stage.CreateNew(str(path)) if not path.exists() else Usd.Stage.Open(str(path))
    scene.GetRootLayer().Clear();UsdGeom.SetStageUpAxis(scene,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(scene,1.)
    root=UsdGeom.Xform.Define(scene,'/NavWareSetStyle');scene.SetDefaultPrim(root.GetPrim())
    for name,x0,x1,y0,y1 in [('Main',-1,9.7,-1.95,1.95),('Branch',3.8,6.,1.95,4.8)]:
        m=UsdGeom.Mesh.Define(scene,'/NavWareSetStyle/'+name)
        m.CreatePointsAttr([Gf.Vec3f(x0,y0,0),Gf.Vec3f(x1,y0,0),Gf.Vec3f(x1,y1,0),Gf.Vec3f(x0,y1,0)])
        m.CreateFaceVertexCountsAttr([4]);m.CreateFaceVertexIndicesAttr([0,1,2,3]);m.CreateSubdivisionSchemeAttr('none')
        m.CreateDisplayColorAttr([Gf.Vec3f(.57,.60,.62)]);UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    boxes=[('South',[4.35,-2.025,1.3],[10.85,.15,2.6]),('West',[-1.075,0,1.3],[.15,3.9,2.6]),
           ('East',[9.775,0,1.3],[.15,3.9,2.6]),('CornerWall',[1.4,2.025,1.3],[4.8,.15,2.6]),
           ('NorthRight',[7.85,2.025,1.3],[3.7,.15,2.6]),('BranchLeft',[3.725,3.45,1.3],[.15,2.85,2.6]),
           ('BranchRight',[6.075,3.45,1.3],[.15,2.85,2.6]),('BranchEnd',[4.9,4.875,1.3],[2.35,.15,2.6]),
           ('Cabinet',[8.8,1.65,.65],[.65,.45,1.3]),('Crate',[8.8,-1.65,.3],[.6,.45,.6])]
    geometry=[]
    for name,p,s in boxes:
        cube=UsdGeom.Cube.Define(scene,'/NavWareSetStyle/'+name);cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*p));cube.AddScaleOp().Set(Gf.Vec3d(*s))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*([.80,.83,.82] if 'Cabinet'!=name and 'Crate'!=name else [.33,.44,.47]))])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        x,y=p[:2];a,b=np.array(s[:2])/2
        geometry.append(dict(name=name,polygon_xy=[[x-a,y-b],[x+a,y-b],[x+a,y+b],[x-a,y+b]],z_range=[p[2]-s[2]/2,p[2]+s[2]/2]))
    scene.GetRootLayer().Save();stage.DefinePrim('/NavWareSetStyle','Xform').GetReferences().AddReference(str(path))
    return geometry

class NonSocialController(DemoController):
    """Same route; no ordinary social response, perception-only emergency stop."""
    def step(self,now,tracks,measurement_time,position,yaw):
        # Reuse route bookkeeping and perception risk generation, not social actions.
        super().step(now,tracks,measurement_time,position,yaw)
        if self.done or any(r['distance']<1.15 for r in self.risks):
            self.state='STOP';return [0.,0.]
        delta=self.waypoints[self.index]-np.asarray(position)[:2]
        error=np.arctan2(np.sin(np.arctan2(delta[1],delta[0])-yaw),np.cos(np.arctan2(delta[1],delta[0])-yaw))
        self.state='CRUISE';return [.30,float(np.clip(1.2*error,-.5,.5))]

def controller(config,behavior):
    c=(DemoController if behavior=='social' else NonSocialController)()
    c.waypoints=[np.array(p,dtype=float) for p in config['robot_waypoints']]
    return c

class ScriptedPeople:
    """Preset clock and own waypoint arrival only; never reads robot pose."""
    def __init__(self,agents,config):
        import carb
        self.agents=agents;self.spec=config['humans'];self.indices=[-1]*len(agents)
        for a,p in zip(agents,self.spec):
            if not a.teleport(carb.Float3(*p['start'])):raise RuntimeError('Preset teleport failed')
            a.set_speed(p['speed'])
    def step(self,t):
        import carb
        for i,(a,p) in enumerate(zip(self.agents,self.spec)):
            k=self.indices[i]
            if k<0:
                if t<p['delay']:continue
                k=0
            else:
                q=a.get_world_translation()
                if np.linalg.norm(np.array([q.x,q.y])-np.array(p['goals'][k][:2]))>.25:continue
                if k+1>=len(p['goals']):continue
                k+=1
            a.move_to(carb.Float3(*p['goals'][k]),auto_brake=True);self.indices[i]=k

def save_dataset(output,config,geometry,rows,robot_config,behavior,actor_paths):
    config=dict(config,robot_model='a300',motion_mode='kinematic',robot_behavior=behavior,human_count=len(config['humans']),
                camera_parameters={'mount_m':robot_config['camera_mount_m'],'resolution':[640,360],'hfov_deg':106.26,'clipping_m':[.05,100.]},
                actor_paths=actor_paths,gt_used_by_controller=False,description='NavWareSet-style synthetic social-navigation environment; not exact reconstruction')
    (output/'meta.json').write_text(json.dumps(config,indent=2))
    (output/'occupancy_xy_points.json').write_text(json.dumps({'frame':'world','units':'m','source':'synthetic GT geometry; evaluation only','polygons':geometry},indent=2))
    with (output/'robot_and_participants.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['timestamp','robot_x','robot_y','robot_yaw',*[v for i in range(len(config['humans'])) for v in (f'human_{i+1}_x',f'human_{i+1}_y')],'controller_state','robot_v_cmd','robot_omega_cmd'])
        for r in rows:w.writerow([r['time'],*r['robot_position'][:2],r['robot_yaw'],*[v for p in r['evaluation_people'] for v in p[:2]],r['state'],*r['command']])

def evaluate(output):
    """Post-run evaluation only. No result is returned to the live controller."""
    from pathlib import Path
    output=Path(output)
    summary=json.loads((output/'summary.json').read_text())
    rows=json.loads((output/'demo_evaluation.json').read_text())
    detections=json.loads((output/'detections.json').read_text())
    transitions=[r for i,r in enumerate(rows) if i==0 or r['state']!=rows[i-1]['state']]
    active=[r for r in rows if not r['route_done']]
    reactions=[r for r in active if r['state']!='CRUISE']
    resume=bool(reactions and any(r['time']>reactions[0]['time'] and r['state']=='CRUISE' and r['speed']>.25 for r in active))
    people=np.array([r['evaluation_people'] for r in rows]);dt=np.diff([r['time'] for r in rows])
    speeds=np.linalg.norm(np.diff(people[:,:,:2],axis=0),axis=2)/dt[:,None]
    geometry=json.loads((output/'occupancy_xy_points.json').read_text())['polygons']
    static_min=float('inf')
    radius=summary['demo']['geometric_collision_threshold']-.3
    for r in rows:
        p=np.array(r['robot_position'][:2])
        for box in geometry:
            vertices=np.array(box['polygon_xy']);lo=vertices.min(axis=0);hi=vertices.max(axis=0)
            static_min=min(static_min,float(np.linalg.norm(np.maximum(np.maximum(lo-p,p-hi),0.)))-radius)
    metrics=dict(route_done=summary['demo']['route_done'],minimum_gt_distance_m=summary['demo']['min_gt_distance'],
      collision_proxy=summary['demo']['min_gt_distance']<summary['demo']['geometric_collision_threshold'],
      robot_static_disk_clearance_m=static_min,stop_count=sum(r['state']=='STOP' and not r['route_done'] for r in transitions),
      resume=resume,travel_time_s=next((r['time'] for r in rows if r['route_done']),None),
      yolo_responses_per_wall_s=summary['yolo']['effective_fps'],rtf=summary['real_time_factor'],
      max_detections=max(len(d['boxes']) for d in detections),max_tracks=max(len(d.get('tracks',[])) for d in detections),
      walking_path_lengths_m=np.linalg.norm(np.diff(people[:,:,:2],axis=0),axis=2).sum(axis=0).tolist(),
      moving_speed_medians_m_s=[float(np.median(speeds[:,i][speeds[:,i]>.2])) if np.any(speeds[:,i]>.2) else 0. for i in range(people.shape[1])],
      transitions=[{'time':r['time'],'state':r['state']} for r in transitions],
      gpu_peak_mib=summary['gpu_total_memory_peak_mib'],system_ram_peak_gib=summary['system_ram_peak_gib'])
    metadata=json.loads((output/'meta.json').read_text())
    metrics['functional_pass']=bool(metrics['route_done'] and not metrics['collision_proxy'] and static_min>=0 and
        (metadata['robot_behavior']=='non-social' or (reactions and resume)) and metrics['max_detections']>0 and all(x>.5 for x in metrics['walking_path_lengths_m']))
    metrics['performance_pass']=metrics['yolo_responses_per_wall_s']>=5.
    saved=[d for d in detections if d.get('evidence_frame')]
    first_detection=next((d for d in detections if d['boxes']),None)
    reaction_time=reactions[0]['time'] if reactions else None
    evidence={}
    for label,predicate in [('normal_detection',lambda d:bool(d['boxes'])),
      ('interaction',lambda d:d.get('controller_state') in ('SLOW','STOP','AVOID_LEFT','AVOID_RIGHT')),
      ('stop_or_avoid',lambda d:d.get('controller_state') in ('STOP','AVOID_LEFT','AVOID_RIGHT')),
      ('recovery',lambda d:reaction_time is not None and d.get('response_elapsed_s',0)>reaction_time and d.get('controller_state')=='CRUISE')]:
        d=next((d for d in saved if predicate(d)),None)
        evidence[label]=d['evidence_frame']+'_bbox.png' if d else None
    if metadata['scenario']=='blind_corner':
        def blocked(a,b,box):
            polygon=np.array(box['polygon_xy']);lo=np.r_[polygon.min(axis=0),box['z_range'][0]];hi=np.r_[polygon.max(axis=0),box['z_range'][1]]
            low,high=0.,1.
            for j in range(3):
                delta=b[j]-a[j]
                if abs(delta)<1e-9:
                    if a[j]<lo[j] or a[j]>hi[j]:return False
                else:
                    t0,t1=sorted(((lo[j]-a[j])/delta,(hi[j]-a[j])/delta));low=max(low,t0);high=min(high,t1)
                    if low>high:return False
            return high>0 and low<1
        blocked_samples=[]
        for d in detections:
            t=d.get('exposure_elapsed_s',d['sim_time']-metadata.get('initial_sim_time',.5))
            r=min(rows,key=lambda r:abs(r['time']-t));yaw=r['robot_yaw']
            cam=np.array(r['robot_position'])+np.array([.37*np.cos(yaw),.37*np.sin(yaw),.5])
            target=np.array(d['evaluation_gt_people_xyz'][0])+[0.,0.,1.5]
            hidden=any(blocked(cam,target,g) for g in geometry)
            if hidden:blocked_samples.append(d)
        d=next((d for d in blocked_samples if not d['boxes'] and d.get('evidence_frame')),None)
        evidence['wall_occluded']=d['evidence_frame']+'_bbox.png' if d else None
        evidence['first_yolo_detection']=first_detection.get('evidence_frame','')+'_bbox.png' if first_detection else None
        first_t=first_detection.get('exposure_elapsed_s') if first_detection else None
        metrics['blind_corner_evidence']={'geometry_occluded_samples':len(blocked_samples),
          'first_yolo_exposure_elapsed_s':first_t,'first_reaction_elapsed_s':reaction_time,
          'no_reaction_before_detection':bool(first_t is not None and reaction_time is not None and reaction_time>=first_t),
          'visibility_note':'Wall ray test at head height plus saved RGB inspection; first YOLO detection is not necessarily first visible pixel.'}
    (output/'evidence_index.json').write_text(json.dumps(evidence,indent=2))
    (output/'scenario_evaluation.json').write_text(json.dumps(metrics,indent=2))
    return metrics
