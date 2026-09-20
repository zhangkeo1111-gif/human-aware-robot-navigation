"""Independent simple arena and prescribed human motion; no human GT in control."""
import json
import numpy as np
from demo_scene import DemoController

SCENARIOS=('headon','crossing','static_obstruction','overtaking','blind_corner',
           'diagonal_crossing','side_offset_static','cut_in')

def preset(name):
    # Crossing reaches y=0 at t=15 s, matching unimpeded robot x=4.5 m.
    specs={'headon':([7.,0.],[.5,0.],.65,0.),
           'crossing':([4.5,-2.5],[4.5,2.5],.65,15.-2.5/.65),
           'static_obstruction':([4.5,0.],[4.5,0.],0.,0.),
           'overtaking':([1.8,0.],[10.,0.],.20,0.),
           'blind_corner':([4.5,2.5],[4.5,-2.5],.65,15.-2.5/.65),
           # Nominal 0.35 m/s robot reaches the diagonal midpoint at ~14.3 s.
           'diagonal_crossing':([3.5,-2.3],[6.5,2.3],.60,5./.35-np.hypot(1.5,2.3)/.60),
           'side_offset_static':([4.5,-.6],[4.5,-.6],0.,0.),
           'cut_in':([2.5,-2.0],[5.5,.2],.50,7.)}
    start,goal,speed,delay=specs[name]
    return dict(scenario=name,robot_start=[0.,0.,0.],robot_waypoints=[[9.,0.]],timeout_s=70.,
        humans=[dict(start=start+[0.],goals=[goal+[0.]]+([[10.5,.2,0.]] if name=='cut_in' else []),speed=speed,delay=delay)],
        bounds_xy=[[-1.,-3.],[11.,3.]],
        grs_camera=dict(eye=[-1.,-5.,5.],target=[5.,0.,0.],resolution=[960,540],rate_hz=10),
        blind_wall=dict(center=[1.4,.825,1.3],size=[2.8,.15,2.6]) if name=='blind_corner' else None,
        actor='Isaac/People/Characters/original_male_adult_medical_01',
        human_navigation=dict(blocked_speed_threshold=5.,auto_avoidance=False,obstacle_avoidance=False,exclude_dynamic_robot_from_static_bake=True),
        description='Independent classic arena; not NavWareSet geometry')

def build(stage,path,config):
    from pxr import Usd,UsdGeom,UsdPhysics,Gf
    UsdGeom.Imageable(stage.GetPrimAtPath('/World/defaultGroundPlane')).MakeInvisible()
    # Base asset is invariant; the optional corner wall is authored in the live stage only.
    scene=Usd.Stage.CreateInMemory();UsdGeom.SetStageUpAxis(scene,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(scene,1.)
    root=UsdGeom.Xform.Define(scene,'/ClassicSingle');scene.SetDefaultPrim(root.GetPrim())
    floor=UsdGeom.Mesh.Define(scene,'/ClassicSingle/Floor')
    floor.CreatePointsAttr([Gf.Vec3f(-1,-3,0),Gf.Vec3f(11,-3,0),Gf.Vec3f(11,3,0),Gf.Vec3f(-1,3,0)])
    floor.CreateFaceVertexCountsAttr([4]);floor.CreateFaceVertexIndicesAttr([0,1,2,3]);floor.CreateSubdivisionSchemeAttr('none')
    floor.CreateDisplayColorAttr([Gf.Vec3f(.57,.60,.62)]);UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    geometry=[]
    def box(target,name,p,s):
        cube=UsdGeom.Cube.Define(target,'/ClassicSingle/'+name);cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*p));cube.AddScaleOp().Set(Gf.Vec3d(*s))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.80,.83,.82)]);UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        x,y=p[:2];a,b=np.array(s[:2])/2
        geometry.append(dict(name=name,polygon_xy=[[x-a,y-b],[x+a,y-b],[x+a,y+b],[x-a,y+b]],z_range=[0.,2.6]))
    for name,p,s in [('South',[5,-3.075,1.3],[12.15,.15,2.6]),('North',[5,3.075,1.3],[12.15,.15,2.6]),
                     ('West',[-1.075,0,1.3],[.15,6,2.6]),('East',[11.075,0,1.3],[.15,6,2.6])]:box(scene,name,p,s)
    if not path.exists():scene.GetRootLayer().Export(str(path))
    stage.DefinePrim('/ClassicSingle','Xform').GetReferences().AddReference(str(path))
    if config['blind_wall']:box(stage,'BlindWall',config['blind_wall']['center'],config['blind_wall']['size'])
    # Include the complete arena before the unchanged actor loader bakes its mesh.
    # Its default 10 m volume centered at x=3 otherwise rejects the x=10 goal.
    import NavSchema
    import carb
    carb.settings.get_settings().set('/exts/omni.anim.behavior.core/navBlockedSpeedThreshold',config['human_navigation']['blocked_speed_threshold'])
    volume=NavSchema.NavMeshVolume.Define(stage,'/ClassicSingle/NavigationVolume')
    volume.GetNavVolumeTypeAttr().Set('Include')
    UsdGeom.Boundable(volume.GetPrim()).GetExtentAttr().Set([Gf.Vec3f(-1,-3,-1),Gf.Vec3f(11,3,3)])
    return geometry

class ScriptedPeople:
    def __init__(self,agents,config):
        import carb
        import omni.anim.navigation.core as navigation
        assert len(agents)==1
        self.agent=agents[0];self.spec=config['humans'][0];self.started=False;self.goal_index=0
        self.agent.set_auto_avoidance_enabled(False)
        self.agent.set_obstacle_avoidance_enabled(False)
        if not self.agent.teleport(carb.Float3(*self.spec['start'])):raise RuntimeError('Classic teleport failed')
        self.agent.set_speed(max(self.spec['speed'],.01))
        mesh=navigation.acquire_interface().get_navmesh()
        print('CLASSIC_HUMAN_TARGET',self.spec['goals'][0],str(mesh.query_closest_point(carb.Float3(*self.spec['goals'][0]))),flush=True)
    def step(self,t):
        import carb
        if not self.started and self.spec['speed']>0 and t>=self.spec['delay']:
            result=self.agent.move_to(carb.Float3(*self.spec['goals'][0]),auto_brake=True);self.started=True
            print('CLASSIC_MOVE_RESULT',str(result),flush=True)
            if result == -1:raise RuntimeError('Classic human goal rejected by navigation mesh')
        elif self.started and self.goal_index+1<len(self.spec['goals']):
            q=self.agent.get_world_translation()
            if np.linalg.norm(np.array([q.x,q.y])-np.array(self.spec['goals'][self.goal_index][:2]))<.25:
                self.goal_index+=1
                result=self.agent.move_to(carb.Float3(*self.spec['goals'][self.goal_index]),auto_brake=True)
                if result == -1:raise RuntimeError('Classic human waypoint rejected')

class DiagnosticRoute:
    """Open-loop scene validation: no perception or human state used for commands."""
    def __init__(self,goal):self.goal=np.array(goal);self.done=False;self.state='CRUISE';self.risks=[]
    def step(self,now,tracks,measurement_time,position,yaw):
        self.done=bool(np.linalg.norm(self.goal-np.asarray(position)[:2])<.3)
        self.state='STOP' if self.done else 'CRUISE'
        return [0. if self.done else .30,0.]

def make_controller(config,geometry,common,robot,behavior,diagnostic):
    if diagnostic:return DiagnosticRoute(config['robot_waypoints'][-1])
    if behavior=='social_nav':
        from social_controller import SocialController
        return SocialController(common['social_navigation'],config['robot_waypoints'],geometry,
            robot['conservative_radius_m'],dict(linear_accel_m_s2=.5,linear_decel_m_s2=.8,angular_accel_rad_s2=1.2))
    c=DemoController();c.waypoints=[np.array(p,dtype=float) for p in config['robot_waypoints']];return c

def save_dataset(output,config,geometry,rows,robot,behavior,paths):
    (output/'meta.json').write_text(json.dumps(dict(config,robot_behavior=behavior,actor_paths=paths,
        camera=dict(mount=robot['camera_mount_m'],resolution=[640,360],hfov_deg=106.26,clipping=[.05,100.]),
        gt_used_by_controller=False),indent=2))
    (output/'occupancy_xy_points.json').write_text(json.dumps(dict(polygons=geometry,source='known static map; no human state'),indent=2))
