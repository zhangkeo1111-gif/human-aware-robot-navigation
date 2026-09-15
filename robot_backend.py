"""Small robot geometry and wheel-command adapter; perception stays model-independent."""
import json
from pathlib import Path
import numpy as np

def settings(model):
    if model=='jetbot':
        return {'name':'jetbot','prim_path':'/World/Jetbot','wheel_radius_m':.03,'track_width_m':.1125,
          'joints':['left_wheel_joint','right_wheel_joint'],'side':[-1,1],
          'spawn_z_m':.05,'camera_mount_m':[.2,0.,.95],'conservative_radius_m':.15}
    root=Path(__file__).resolve().parent/'assets/husky_a300'
    p=json.loads((root/'parameters.json').read_text())
    p.update(name='a300',prim_path='/World/A300',asset=str(root/'husky_a300.usd'),
      joints=[w['name'] for w in p['wheels']],side=[-1 if 'left' in w['name'] else 1 for w in p['wheels']])
    return p

def configure_a300(stage,p,with_camera=False):
    """Explicit simulation contact material, not an OEM measured tire model."""
    from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf,Sdf
    if p.get('solver_iterations'):
        api=PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(p['prim_path']+'/Geometry/base_link'))
        api.CreateSolverPositionIterationCountAttr(p['solver_iterations'][0])
        api.CreateSolverVelocityIterationCountAttr(p['solver_iterations'][1])
    material=UsdShade.Material.Define(stage,p['prim_path']+'/TireContact')
    contact=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    sf,df=p.get('tire_friction',[.5,.4])
    contact.CreateStaticFrictionAttr(sf);contact.CreateDynamicFrictionAttr(df);contact.CreateRestitutionAttr(0.)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr('min')
    if p.get('tire_restitution_min',False):
        PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateRestitutionCombineModeAttr('min')
    wheels=[]
    for prim in Usd.PrimRange(stage.GetPrimAtPath(p['prim_path'])):
        if '_wheel_link/' in str(prim.GetPath()) and prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,UsdShade.Tokens.strongerThanDescendants,'physics')
            wheels.append(str(prim.GetPath()))
    p['simulation_tire_contact']={'static':sf,'dynamic':df,'combine':'min','colliders':wheels,'source':'explicit simulation assumption, not OEM measurement'}
    if 'wheel_damping' in p:
        for j in p['joints']:
            UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(p['prim_path']+'/Physics/'+j),'angular').GetDampingAttr().Set(float(p['wheel_damping']*np.pi/180))
    prims=list(Usd.PrimRange(stage.GetPrimAtPath(p['prim_path']),Usd.TraverseInstanceProxies()))
    p['imported_geometry_audit']={
      'mesh_faces':sum(len(UsdGeom.Mesh(a).GetFaceVertexCountsAttr().Get() or []) for a in prims if a.IsA(UsdGeom.Mesh)),
      'collision_shapes':sum(a.HasAPI(UsdPhysics.CollisionAPI) for a in prims),
      'authored_mass_kg':sum(float(UsdPhysics.MassAPI(a).GetMassAttr().Get() or 0.) for a in prims if a.HasAPI(UsdPhysics.MassAPI)),
      'articulation_roots':[str(a.GetPath()) for a in prims if a.HasAPI(UsdPhysics.ArticulationRootAPI)]}
    if with_camera:
        base=p['prim_path']+'/Geometry/base_link'
        # Display-only mast/housing: represents the added RGB-D sensor, not OEM geometry.
        for name,xyz,size in [('SensorMast',[.325,0.,.36],[.025,.025,.24]),('RGBDHousing',[.325,0.,.5],[.09,.12,.05])]:
            cube=UsdGeom.Cube.Define(stage,base+'/'+name)
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*xyz));cube.AddScaleOp().Set(Gf.Vec3d(*size))
            cube.CreateDisplayColorAttr([Gf.Vec3f(.08,.08,.08)])

class WheelAdapter:
    def __init__(self,parameters,limits):
        self.p=parameters
        self.limits=np.asarray(limits,dtype=float)
        self.requested=self.clipped=np.zeros(len(limits))

    def forward(self,command):
        from isaacsim.core.utils.types import ArticulationAction
        v,w=command
        self.requested=(v+np.asarray(self.p['side'])*w*self.p['track_width_m']/2)/self.p['wheel_radius_m']
        self.clipped=np.clip(self.requested,-self.limits,self.limits)
        if not np.array_equal(self.requested,self.clipped):
            print('WHEEL_CLIPPING',self.requested.tolist(),self.clipped.tolist(),flush=True)
        return ArticulationAction(joint_velocities=self.clipped)


class KinematicA300:
    """Ideal planar motion; USD geometry only, no concurrent wheel physics drive."""
    def __init__(self,stage,p):
        from pxr import Usd,UsdPhysics,UsdGeom,Gf
        self.p=p;self.stage=stage;self.position=np.array([0.,0.,.13597]);self.yaw=0.;self.v=0.;self.w=0.
        self.angles=np.zeros(4);self.rates=np.zeros(4);self.dof_names=p['joints'];self.dof_properties={'maxVelocity':np.full(4,12.)}
        p['kinematic_model']={'z_m':.13597,'linear_accel_m_s2':.5,'linear_decel_m_s2':.8,'angular_accel_rad_s2':1.2,'wheel_contact_controls_chassis':False,'collision_geometry_retained':True,'rigid_body_enabled':False,'joint_enabled':False}
        for prim in Usd.PrimRange(stage.GetPrimAtPath(p['prim_path'])):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
            if prim.IsA(UsdPhysics.Joint):UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(False)
        self.base=stage.GetPrimAtPath(p['prim_path']+'/Geometry/base_link')
        self.wheels=[stage.GetPrimAtPath(p['prim_path']+'/Geometry/base_link/'+j.replace('_joint','_link')) for j in p['joints']]
        self.base_xform=UsdGeom.Xformable(self.base);self.base_xform.ClearXformOpOrder()
        self.translation=self.base_xform.AddTranslateOp();self.orientation=self.base_xform.AddOrientOp()
        self.wheel_orient=[]
        for prim in self.wheels:
            attr=prim.GetAttribute('xformOp:orient')
            self.wheel_orient.append(attr if attr else UsdGeom.Xformable(prim).AddOrientOp().GetAttr())
        self._write()
    def _write(self):
        from pxr import Gf
        self.translation.Set(Gf.Vec3d(*self.position))
        self.orientation.Set(Gf.Quatf(float(np.cos(self.yaw/2)),0.,0.,float(np.sin(self.yaw/2))))
        for attr,angle in zip(self.wheel_orient,self.angles):
            attr.Set(Gf.Quatf(float(np.cos(angle/2)),0.,float(np.sin(angle/2)),0.))
    def advance(self,command,dt):
        target_v,target_w=command
        dv=(.8 if abs(target_v)<abs(self.v) else .5)*dt
        self.v+=float(np.clip(target_v-self.v,-dv,dv));self.w+=float(np.clip(target_w-self.w,-1.2*dt,1.2*dt))
        self.position[:2]+=self.v*np.array([np.cos(self.yaw),np.sin(self.yaw)])*dt
        self.yaw+=self.w*dt
        self.rates=(self.v+np.asarray(self.p['side'])*self.w*self.p['track_width_m']/2)/self.p['wheel_radius_m']
        self.angles+=self.rates*dt;self._write()
    def get_world_pose(self):return self.position.copy(),np.array([np.cos(self.yaw/2),0.,0.,np.sin(self.yaw/2)])
    def read_usd_pose(self):
        from pxr import UsdGeom
        matrix=UsdGeom.XformCache().GetLocalToWorldTransform(self.base);q=matrix.ExtractRotationQuat()
        return np.array(matrix.ExtractTranslation()),np.array([q.GetReal(),*q.GetImaginary()])
    def get_linear_velocity(self):return np.array([self.v*np.cos(self.yaw),self.v*np.sin(self.yaw),0.])
    def get_angular_velocity(self):return np.array([0.,0.,self.w])
    def get_joint_velocities(self):return self.rates.copy()
    def get_dof_index(self,name):return self.dof_names.index(name)
    def apply_wheel_actions(self,action):pass  # Never apply a PhysX wheel drive in this mode.
