"""Physics-only diagnostics and finite command sequences; no perception changes."""
import json
from collections import defaultdict

SHORT = [[.3,0.],[-.3,0.],[.2,.4],[.2,-.4],[0.,.4],[0.,0.]]
MATRIX = [[.3,0.],[-.3,0.]]+[[.2,w] for w in [.2,-.2,.4,-.4,.6,-.6]]+[[0.,.4],[0.,-.4],[0.,0.]]
DEMO = [[.3,0.],[.2,.4],[.3,0.],[.2,-.4],[0.,0.]]

def summarize(directory):
    """Report settled windows (1--3.95 s per 4 s command), not transient peaks."""
    import numpy as np
    rows=json.loads((directory/'robot_backend.json').read_text())
    result=[]
    for k in range(int(max(r['time'] for r in rows)//4)+1):
        sample=[r for r in rows if 4*k+1<r['time']<4*(k+1)-.05]
        if not sample: continue
        mean=lambda key:float(np.mean([r[key] for r in sample]))
        v,w=sample[0]['command']
        actual=np.array([r['actual_joint_velocity'] for r in sample])
        target=np.array([r['requested_wheel_rad_s'] for r in sample])
        dt=np.diff([r['time'] for r in sample])
        pose_yaw=float(np.mean(np.diff(np.unwrap([r['yaw'] for r in sample]))/dt))
        xy=np.asarray([r['position'][:2] for r in sample]);yaw=np.asarray([r['yaw'] for r in sample[:-1]])
        pose_v=float(np.mean(np.sum(np.diff(xy,axis=0)*np.column_stack([np.cos(yaw),np.sin(yaw)]),axis=1)/dt))
        result.append({'segment':k,'v_cmd':v,'omega_cmd':w,'v_actual':mean('actual_forward_m_s'),
          'omega_actual':mean('actual_yaw_rate'),'yaw_gain':mean('actual_yaw_rate')/w if w else None,
          'pose_yaw_rate':pose_yaw,'pose_forward_m_s':pose_v,'linear_error':mean('actual_forward_m_s')-v,
          'lateral_mean':mean('lateral_velocity_m_s') if 'lateral_velocity_m_s' in sample[0] else None,
          'wheel_target':target.mean(axis=0).tolist(),'wheel_actual':actual.mean(axis=0).tolist(),
          'wheel_error_mae':float(np.mean(abs(actual-target))),
          'roll_pitch_max_deg':float(np.rad2deg(np.max([[abs(r.get('roll',0)),abs(r.get('pitch',0))] for r in sample]))),
          'lateral_abs_max':max(abs(r.get('lateral_velocity_m_s',0)) for r in sample),
          'wheel_abs_max':float(abs(actual).max()),'z_min':min(r['position'][2] for r in sample),'z_max':max(r['position'][2] for r in sample)})
    (directory/'motion_summary.json').write_text(json.dumps(result,indent=2))
    return result

class PhysicsAudit:
    def __init__(self,stage,root):
        from pxr import Usd,UsdPhysics,PhysxSchema,PhysicsSchemaTools
        from omni.physx import get_physx_simulation_interface
        self.stage=stage;self.root=root;self.points=[];self.pairs=defaultdict(lambda:{'contacts':0,'normal_impulse_sum':0.,'minimum_separation':1.})
        for p in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
        def callback(headers,data):
            for h in headers:
                names=[str(PhysicsSchemaTools.intToSdfPath(getattr(h,k))) for k in ('collider0','collider1')]
                if not any(root in n for n in names): continue
                key=' | '.join(names);record=self.pairs[key]
                for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                    d=data[i];record['contacts']+=1
                    record['normal_impulse_sum']+=float(d.impulse.GetLength()) if hasattr(d.impulse,'GetLength') else sum(float(v)**2 for v in d.impulse)**.5
                    record['minimum_separation']=min(record['minimum_separation'],float(d.separation))
                    if len(self.points)<200:
                        self.points.append({'pair':names,'position':list(d.position),'normal':list(d.normal),'impulse':list(d.impulse),'separation':float(d.separation)})
        self.subscription=get_physx_simulation_interface().subscribe_contact_report_events(callback)

    def save(self,path,robot,world):
        from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema
        records=[]
        for p in Usd.PrimRange(self.stage.GetPseudoRoot(),Usd.TraverseInstanceProxies()):
            if not (p.HasAPI(UsdPhysics.RigidBodyAPI) or p.HasAPI(UsdPhysics.CollisionAPI) or p.HasAPI(UsdPhysics.MaterialAPI) or p.IsA(UsdPhysics.Joint) or p.IsA(UsdPhysics.Scene)): continue
            record={'path':str(p.GetPath()),'type':p.GetTypeName(),'attributes':{a.GetName():a.Get() for a in p.GetAttributes() if a.GetName().startswith(('physics:','physx','drive:','radius','height','axis','xformOp:'))}}
            if p.HasAPI(UsdPhysics.CollisionAPI):
                material,_=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial(materialPurpose='physics')
                record['bound_material']=str(material.GetPath()) if material else None
                bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy,UsdGeom.Tokens.guide],False,True).ComputeWorldBound(p).ComputeAlignedRange()
                record['world_bounds']=None if bounds.IsEmpty() else [list(bounds.GetMin()),list(bounds.GetMax())]
            records.append(record)
        result={'usd':records,'physics_dt':world.get_physics_dt(),'rendering_dt':world.get_rendering_dt(),'steps_per_world_step':1,
          'runtime_dof_properties':str(robot.dof_properties),'runtime_drive_gains':robot.get_articulation_controller().get_gains(),
          'runtime_solver_iterations':[int(robot.get_solver_position_iteration_count()),int(robot.get_solver_velocity_iteration_count())] if self.stage.GetPrimAtPath(self.root+'/Geometry/base_link').HasAPI(PhysxSchema.PhysxArticulationAPI) else 'Unauthored; engine defaults',
          'material_schema_attributes':list(PhysxSchema.PhysxMaterialAPI.GetSchemaAttributeNames()),
          'contact_pairs':dict(self.pairs),'contact_point_sample':self.points}
        path.write_text(json.dumps(result,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)))
