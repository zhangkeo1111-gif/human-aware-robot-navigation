"""Small indoor scene and perception-only reactive demo controller."""
import numpy as np

def build(stage,path):
    from pxr import Usd,UsdGeom,UsdPhysics,Gf,UsdLux
    UsdGeom.Imageable(stage.GetPrimAtPath('/World/defaultGroundPlane')).MakeInvisible()
    scene=Usd.Stage.CreateNew(str(path)) if not path.exists() else Usd.Stage.Open(str(path))
    scene.GetRootLayer().Clear()
    UsdGeom.SetStageUpAxis(scene,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(scene,1.)
    root=UsdGeom.Xform.Define(scene,'/Indoor');scene.SetDefaultPrim(root.GetPrim())
    def box(name,p,s,c):
        b=UsdGeom.Cube.Define(scene,'/Indoor/'+name);b.CreateSizeAttr(1.)
        b.AddTranslateOp().Set(Gf.Vec3d(*p));b.AddScaleOp().Set(Gf.Vec3d(*s));b.CreateDisplayColorAttr([Gf.Vec3f(*c)])
        UsdPhysics.CollisionAPI.Apply(b.GetPrim())
    floor=UsdGeom.Mesh.Define(scene,'/Indoor/Floor')
    floor.CreatePointsAttr([Gf.Vec3f(-2,-4,0),Gf.Vec3f(12,-4,0),Gf.Vec3f(12,4,0),Gf.Vec3f(-2,4,0)])
    floor.CreateFaceVertexCountsAttr([4]);floor.CreateFaceVertexIndicesAttr([0,1,2,3])
    floor.CreateSubdivisionSchemeAttr('none');floor.CreateDisplayColorAttr([Gf.Vec3f(.58,.62,.64)])
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    box('RearWall',[12,0,1.6],[.15,8,3.2],[.83,.85,.84])
    box('NorthWall',[5,4,1.6],[14,.15,3.2],[.8,.84,.86])
    for y in [-2.5,2.5]:box('DoorSide'+('L' if y<0 else 'R'),[-2,y,1.6],[.15,3,3.2],[.8,.84,.86])
    box('DoorLintel',[-2,0,2.9],[.15,2,.6],[.3,.4,.45])
    for i,x in enumerate([3.,8.]):
        box(f'Table{i}',[x,3,.78],[1.6,.75,.10],[.48,.36,.23])
        for k,(dx,dy) in enumerate([(-.65,-.25),(.65,-.25),(-.65,.25),(.65,.25)]):box(f'Leg{i}_{k}',[x+dx,3+dy,.36],[.06,.06,.72],[.2,.24,.26])
        box(f'Chair{i}',[x,2.3,.45],[.48,.48,.08],[.16,.28,.35]);box(f'ChairBack{i}',[x,2.55,.72],[.48,.08,.6],[.16,.28,.35])
        box(f'ChairStem{i}',[x,2.3,.22],[.08,.08,.44],[.2,.24,.26])
        box(f'LabEquipment{i}',[x,3,.99],[.45,.35,.32],[.2,.3,.36])
    box('Cabinet',[10.7,3,.9],[1.1,.7,1.8],[.34,.42,.45])
    box('StorageBox',[10.7,-3,.3],[.7,.7,.6],[.55,.43,.29])
    scene.GetRootLayer().Save()
    stage.DefinePrim('/Indoor','Xform').GetReferences().AddReference(str(path))

class DemoController:
    """No simulator/person GT access. Inputs are estimated tracks and self pose."""
    def __init__(self):
        self.state='CRUISE';self.side=1;self.side_until=0.;self.done=False;self.risks=[]
        self.last_risk_time=-float('inf')
        self.waypoints=[np.array([3.5,0.]),np.array([7.,0.])];self.index=0
    def step(self,now,tracks,measurement_time,position,yaw):
        p=np.asarray(position)[:2]
        if np.linalg.norm(self.waypoints[self.index]-p)<.3:
            if self.index+1<len(self.waypoints):self.index+=1
            else:self.done=True
        delta=self.waypoints[self.index]-p;err=np.arctan2(np.sin(np.arctan2(delta[1],delta[0])-yaw),np.cos(np.arctan2(delta[1],delta[0])-yaw))
        nominal=float(np.clip(1.2*err,-.5,.5));self.risks=[]
        for t in tracks:
            if now-t['last_observed_time']>.6:continue
            s=np.asarray(t['state']);rel=s[:2]+s[2:]*max(0.,now-measurement_time)-p
            xf=float(rel@[np.cos(yaw),np.sin(yaw)]);yl=float(rel@[-np.sin(yaw),np.cos(yaw)])
            distance=float(np.linalg.norm(rel));closing=float(-(rel@s[2:])/max(distance,.01))
            if xf>0 and abs(yl)<1.65:self.risks.append({'track_id':t['track_id'],'distance':distance,'x':xf,'y':yl,'closing':closing})
        if self.done:self.state='STOP';return [0.,0.]
        if not self.risks:
            if self.state=='STOP' and now-self.last_risk_time<.6:return [0.,0.]
            self.state='CRUISE';return [.30,nominal]
        self.last_risk_time=now
        r=min(self.risks,key=lambda r:r['distance']-.25*max(0,r['closing']));d=r['distance']
        if d<1.55 or (self.state=='STOP' and d<1.75):self.state='STOP';return [0.,0.]
        if d<2.0 and abs(r['y'])<1.25:
            if now>=self.side_until:self.side=-1 if r['y']>=0 else 1;self.side_until=now+.9
            self.state='AVOID_LEFT' if self.side>0 else 'AVOID_RIGHT';return [.12,.4*self.side]
        if d<3.:self.state='SLOW';return [.18,nominal]
        self.state='CRUISE';return [.30,nominal]
