"""Route-relative persistent passing; inputs are estimated tracks, ego and map only."""
import time
import numpy as np
from social_controller import SocialController, angle, personal_field, static_clearance


class ActivePassingController(SocialController):
    OFFSETS = (.6, .8, 1.0, -.6, -.8, -1.0)

    def __init__(self, config, waypoints, geometry, radius, motion_limits):
        super().__init__(config, waypoints, geometry, radius, motion_limits)
        self.times = np.arange(61)*.1
        self.phase = 'NAVIGATE'
        self.origin = self.axis = self.normal = None
        self.offset = 0.
        self.target = None
        self.last_target = None
        self.phase_start = None
        self.stationary_since = {}
        self.replans = 0
        self.stop_reason = None
        self.interrupted_phase = None

    def local(self, p):
        q=np.asarray(p)[:2]-self.origin
        return np.array([q@self.axis, q@self.normal])

    @staticmethod
    def smooth(u):
        u=np.clip(u,0.,1.)
        return 10*u**3-15*u**4+6*u**5

    def geometry_path(self, start, offset, human_s, human_v):
        # Full geometric maneuver, not a fixed-omega primitive. Temporal safety
        # is checked over a bounded 6 s dynamic rollout, then rechecked online.
        length=max(1.6,abs(offset-start[1])*2.)
        offset_end=start[0]+length
        closing=max(.05,self.c['cruise_speed']-human_v)
        pass_s=max(offset_end+.8,start[0]+self.c['cruise_speed']*max(0.,human_s-start[0]+.7)/closing)
        recover_end=pass_s+length
        s=np.arange(start[0],recover_end+.051,.05)
        d=start[1]+(offset-start[1])*self.smooth((s-start[0])/length)
        d=np.where(s>=offset_end,offset,d)
        d=np.where(s>=pass_s,offset*(1-self.smooth((s-pass_s)/length)),d)
        return self.origin+s[:,None]*self.axis+d[:,None]*self.normal

    def command_to_path(self, p, yaw, path):
        k=int(np.argmin(np.linalg.norm(path-p,axis=1)))
        target=path[min(k+8,len(path)-1)]
        delta=target-p;err=angle(np.arctan2(delta[1],delta[0])-yaw)
        speed=self.c['cruise_speed']*max(.35,float(np.cos(err)))
        return np.array([speed,np.clip(2*speed*np.sin(err)/max(.25,np.linalg.norm(delta)),-.8,.8)])

    def path_rollout(self,p,yaw,v,w,paths):
        n=len(paths);xy=np.tile(np.asarray(p,float),(n,1));hd=np.full(n,yaw,dtype=float);vs=np.full(n,v,dtype=float);ws=np.full(n,w,dtype=float)
        out=[xy.copy()];commands=[]
        for _ in self.times[1:]:
            cmd=np.array([self.command_to_path(xy[i],hd[i],paths[i]) for i in range(n)])
            a=np.where(np.abs(cmd[:,0])<np.abs(vs),self.limits['linear_decel_m_s2'],self.limits['linear_accel_m_s2'])
            vs+=np.clip(cmd[:,0]-vs,-a*.1,a*.1)
            ws+=np.clip(cmd[:,1]-ws,-self.limits['angular_accel_rad_s2']*.1,self.limits['angular_accel_rad_s2']*.1)
            xy+=vs[:,None]*np.column_stack([np.cos(hd),np.sin(hd)])*.1
            hd+=ws*.1
            out.append(xy.copy());commands.append(np.column_stack([vs,ws]))
        return np.stack(out,axis=1),np.stack(commands,axis=1)

    def assess(self,p,yaw,v,w,paths,humans):
        xy,cmd=self.path_rollout(p,yaw,v,w,paths)
        dist=np.full(len(paths),np.inf);social=np.zeros(len(paths))
        for h in humans:
            delta=xy-h['predicted'][None]
            dist=np.minimum(dist,np.linalg.norm(delta,axis=-1).min(axis=1))
            social+=personal_field(delta,h['heading'],self.c,h['margin'][None])[0].mean(axis=1)*6
        static=static_clearance(xy,self.geometry,self.radius).min(axis=1)
        full_static=np.array([static_clearance(path,self.geometry,self.radius).min() for path in paths])
        length=np.array([np.linalg.norm(np.diff(path,axis=0),axis=1).sum() for path in paths])
        effort=(cmd[:,:,1]**2).sum(axis=1)*.1
        progress=(xy[:,-1]-p)@self.axis
        final=np.array([abs(self.local(path[-1])[1]) for path in paths])
        valid=(dist>=self.radius+self.c['human_radius'])&(np.minimum(static,full_static)>=self.c['static_extra_margin'])
        # A short temporal window must not certify a narrow parallel corridor
        # that inevitably overlaps a pedestrian later in the full maneuver.
        if len(paths)==6:
            for h in humans:
                lateral=self.local(h['pos'])[1]+self.times*(h['velocity']@self.normal)
                corridor=np.min(np.abs(np.array(self.OFFSETS)[:,None]-lateral[None]),axis=1)
                valid &= corridor>=self.radius+self.c['human_radius']
        score=4*social-progress+.12*(length+effort)+.15*final
        diagnostics=[dict(offset=float(self.OFFSETS[i]) if len(paths)==6 else self.offset,valid=bool(valid[i]),
            min_clearance=float(dist[i]-self.radius-self.c['human_radius']),static_clearance=float(min(static[i],full_static[i])),
            social_cost=float(social[i]),path_length=float(length[i]),steering_effort=float(effort[i]),
            final_lateral_error=float(final[i]),total_score=float(score[i])) for i in range(len(paths))]
        return valid,score,diagnostics

    def step(self,now,tracks,measurement_time,position,yaw,v=0.,omega=0.):
        started=time.perf_counter();p=np.asarray(position)[:2]
        if self.origin is None:
            self.origin=p.copy();self.axis=self.waypoints[-1]-p;self.axis/=np.linalg.norm(self.axis)
            self.normal=np.array([-self.axis[1],self.axis[0]])
        if np.linalg.norm(self.waypoints[-1]-p)<.3 and self.phase in ('NAVIGATE','PASS_DONE'):
            self.done=True;self.state='STOP';self.selected='GOAL_STOP';return [0.,0.]
        if now-self.last_plan<.1-1e-7:return self.command.tolist()
        self.last_plan=now;humans=self.humans(now,tracks,measurement_time);sd=self.local(p)
        self.risks=[dict(track_id=h['id'],distance=float(np.linalg.norm(h['pos']-p))) for h in humans]
        for h in humans:
            if np.linalg.norm(h['velocity'])<.12:self.stationary_since.setdefault(h['id'],now)
            else:self.stationary_since.pop(h['id'],None)
        candidate_log=[];self.stop_reason=None
        target=next((h for h in humans if h['id']==self.target),None)
        if target is None and self.last_target is not None:
            stamp,old=self.last_target;age=now-stamp
            if old['age']+age<=self.c['stale_age']:
                predicted=old['pos']+old['velocity']*age
                matches=[h for h in humans if np.linalg.norm(h['pos']-predicted)<.8]
                if len(matches)==1:
                    target=matches[0];self.target=target['id']
                elif not matches:
                    target=dict(old,pos=predicted,age=old['age']+age,predicted=predicted+self.times[:,None]*old['velocity'])
                    humans.append(target)
        if target is not None:self.last_target=(now,target)
        if self.phase=='EMERGENCY_STOP' and target is not None and abs(v)<.03:
            self.replans+=1
            if self.interrupted_phase is not None:
                valid,_,candidate_log=self.assess(p,yaw,v,omega,[self.path],humans)
                if valid[0]:self.phase=self.interrupted_phase;self.interrupted_phase=None
            elif self.offset==0:self.phase='NAVIGATE'
        if self.phase in ('NAVIGATE','PASS_DONE'):
            possible=[h for h in humans if 0<self.local(h['pos'])[0]-sd[0]<6 and abs(self.local(h['pos'])[1])<self.radius+self.c['human_radius']]
            if possible:
                target=min(possible,key=lambda h:np.linalg.norm(h['pos']-p))
                self.target=target['id'];self.last_target=(now,target)
                hs=self.local(target['pos']);hv=target['velocity']@self.axis
                paths=[self.geometry_path(sd,d,hs[0],hv) for d in self.OFFSETS]
                self.phase='PASS_INIT'
                valid,score,candidate_log=self.assess(p,yaw,v,omega,paths,humans)
                if valid.any():
                    i=int(np.argmin(np.where(valid,np.round(score,9),np.inf)));self.offset=self.OFFSETS[i]
                    self.full_path=paths[i]
                    ss=np.arange(sd[0],self.local(self.waypoints[-1])[0]+.051,.05)
                    dd=sd[1]+(self.offset-sd[1])*self.smooth((ss-sd[0])/max(1.6,abs(self.offset-sd[1])*2.))
                    self.path=self.origin+ss[:,None]*self.axis+dd[:,None]*self.normal
                    self.phase='OFFSET';self.phase_start=sd.copy()
                else:self.phase='EMERGENCY_STOP';self.stop_reason='NO_VALID_FULL_PASS_CANDIDATE';self.replans+=1
        heading_error=abs(angle(yaw-np.arctan2(self.axis[1],self.axis[0])))
        if self.phase=='OFFSET' and abs(sd[1]-self.offset)<.12 and heading_error<.15:self.phase='PARALLEL'
        if target is not None and self.phase=='PARALLEL' and sd[0]-self.local(target['pos'])[0]>=.7:
            self.phase='RECOVER';self.phase_start=sd.copy()
            length=max(1.6,2*abs(sd[1]));ss=np.arange(sd[0],sd[0]+length+.051,.05)
            dd=sd[1]*(1-self.smooth((ss-sd[0])/length))
            self.path=self.origin+ss[:,None]*self.axis+dd[:,None]*self.normal
        if self.phase=='RECOVER' and abs(sd[1])<=.3 and heading_error<.15:
            self.phase='PASS_DONE';self.target=None
        cmd=np.array([0.,0.]);predicted=None
        if self.phase in ('OFFSET','PARALLEL','RECOVER'):
            if target is None and self.phase!='RECOVER':
                self.interrupted_phase=self.phase
                self.stop_reason='TARGET_STALE_DURING_CONFLICT';self.phase='EMERGENCY_STOP'
            else:
                valid,_,diag=self.assess(p,yaw,v,omega,[self.path],humans);predicted=diag[0]['min_clearance']
                if valid[0]:cmd=self.command_to_path(p,yaw,self.path)
                else:
                    self.interrupted_phase=self.phase
                    self.stop_reason='CHOSEN_SIDE_PREDICTED_COLLISION';self.phase='EMERGENCY_STOP'
        elif self.phase in ('NAVIGATE','PASS_DONE'):
            cmd=self.command_to_path(p,yaw,np.linspace(p,self.waypoints[-1],100))
        if self.phase=='EMERGENCY_STOP':
            cmd[:]=0
            if self.stop_reason is None:self.stop_reason='LATCHED_STOP_REQUIRES_SAFE_REPLAN'
        self.command=cmd;self.state='STOP' if cmd[0]==0 else 'CRUISE' if self.phase in ('NAVIGATE','PASS_DONE') else 'AVOID_LEFT' if self.offset>0 else 'AVOID_RIGHT'
        side='LEFT' if self.offset>0 else 'RIGHT' if self.offset<0 else 'NONE'
        self.selected=f'{self.phase} {side}'
        hs=self.local(target['pos']) if target is not None else [None,None]
        self.logs.append(dict(time=float(now),state=self.state,controller_state=self.state,passing_phase=self.phase,
            selected=self.selected,selected_side=side,selected_offset=self.offset,route_s=float(sd[0]),route_d=float(sd[1]),
            human_s=hs[0],human_d=hs[1],target_lateral=0 if self.phase=='RECOVER' else self.offset,
            current_clearance=min((r['distance']-self.radius-self.c['human_radius'] for r in self.risks),default=None),
            predicted_min_clearance=predicted,track_age=target['age'] if target is not None else None,
            stationary_confirmed=bool(target is not None and now-self.stationary_since.get(target['id'],now)>=.5),
            v_cmd=float(cmd[0]),omega_cmd=float(cmd[1]),stop_reason=self.stop_reason,pass_replan_count=self.replans,
            candidates=candidate_log,planning_ms=(time.perf_counter()-started)*1000))
        return cmd.tolist()
