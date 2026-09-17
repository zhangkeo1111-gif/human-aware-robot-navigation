"""Visibility-aware route-relative passing. No scene labels or actor inputs."""
import time
import numpy as np
from active_passing_controller import ActivePassingController
from social_controller import angle, personal_field, static_clearance


class ActivePassingControllerV2(ActivePassingController):
    def __init__(self,*args,camera_mount=(.37,0,.5),**kwargs):
        super().__init__(*args,**kwargs)
        # Receding-horizon safety check. Six seconds is the lower bound of
        # the protocol's 6--10 s window and avoids certifying a maneuver from
        # an unreliable long-range velocity extrapolation.
        self.times=np.arange(61)*.1
        self.camera_mount=np.asarray(camera_mount,float)
        self.camera_model=None;self.chosen=None;self.behind_confirmed=False
        self.last_reshape=-np.inf;self.candidate_history=[]

    def humans(self, now, tracks, measurement_time):
        """Build the same causal tracks with an uncertainty-aware planner velocity.

        The tracker output is kept intact.  For passing safety only, a velocity
        estimate that is not separated from zero by 1.5 standard deviations is
        treated as unconfirmed and its short-horizon prediction is held.  This
        prevents a static person from being projected through the robot solely
        from depth jitter, while confirmed motion is passed through unchanged.
        """
        result=super().humans(now,tracks,measurement_time)
        by_id={int(t['track_id']):t for t in tracks}
        for h in result:
            raw=np.asarray(h['velocity'],float); item=by_id.get(int(h['id']),{})
            cov=np.asarray(item.get('covariance'),float)
            sigma=float(np.sqrt(max(0.,np.linalg.eigvalsh(cov[2:,2:]).max()))) if cov.shape==(4,4) and np.all(np.isfinite(cov)) else 0.
            confirmed=bool(np.linalg.norm(raw)>max(.15,1.5*sigma))
            planner=raw if confirmed else np.zeros(2)
            h['velocity_sigma']=sigma;h['velocity_confirmed']=confirmed;h['planner_velocity']=planner
            h['predicted']=h['pos']+self.times[:,None]*planner
            if not confirmed:h['heading']=None
        return result

    def set_camera(self,params):
        aperture=np.asarray(params['cameraAperture']);focal=float(params['cameraFocalLength'])
        resolution=np.asarray(params['renderProductResolution']);offset=np.asarray(params['cameraApertureOffset'])
        fx=float(resolution[0]*focal/aperture[0]);fy=float(resolution[1]*focal/aperture[1])
        half=float(np.arctan(aperture[0]/(2*focal)))
        self.camera_model=dict(fx=fx,fy=fy,cx=float(resolution[0]/2-resolution[0]*offset[0]/aperture[0]),
            cy=float(resolution[1]/2+resolution[1]*offset[1]/aperture[1]),hfov_deg=float(np.degrees(half)*2),
            soft_limit=float(half-np.radians(8)),hard_limit=float(half-np.radians(2)),source='runtime CameraParams',
            # One stale-track window plus one 10 Hz sample is the maximum
            # period for which the causal frontend can bridge a brief hard-FOV
            # loss before the online stale guard must take over.
            hard_duration_limit_s=float(self.c.get('stale_age',.8)+.1),resolution=resolution.tolist(),mount=self.camera_mount.tolist())

    def candidate(self,sd,offset,length,h):
        hs=self.local(h['pos']);hv=float(h.get('planner_velocity',h['velocity'])@self.axis)
        end=sd[0]+length
        # Nominal time parameterization for complete geometry; temporal safety
        # is explicitly bounded by the six-second receding horizon, not full
        # certification of the entire route.
        meet=sd[0]+.30*max(0.,hs[0]-sd[0]+.7)/max(.05,.30-hv)
        recover=max(end+.7,meet)
        s=np.arange(sd[0],recover+length+.051,.05)
        d=sd[1]+(offset-sd[1])*self.smooth((s-sd[0])/length)
        d=np.where(s>=end,offset,d)
        d=np.where(s>=recover,offset*(1-self.smooth((s-recover)/length)),d)
        slope=np.gradient(d,s);theta=np.arctan(slope);curv=np.gradient(slope,s)/(1+slope*slope)**1.5
        full=self.origin+s[:,None]*self.axis+d[:,None]*self.normal
        # Hold parallel until a fresh behind confirmation; never recover merely
        # because the original geometric estimate said it was time.
        ss=np.arange(sd[0],max(self.local(self.waypoints[-1])[0],end+.5)+.051,.05)
        dd=sd[1]+(offset-sd[1])*self.smooth((ss-sd[0])/length)
        path=self.origin+ss[:,None]*self.axis+dd[:,None]*self.normal
        return dict(side='LEFT' if offset>0 else 'RIGHT',offset=offset,length=length,end=end,recover=recover,path=path,full=full,
            maximum_heading_deviation=float(np.max(np.abs(theta))),maximum_curvature=float(np.max(np.abs(curv))),
            geometric_length=float(np.linalg.norm(np.diff(full,axis=0),axis=1).sum()),final_lateral_error=float(abs(d[-1])))

    def rollout_paths(self,p,yaw,v,w,candidates,recovery=False):
        paths=[c['path'] for c in candidates];n=len(paths);count=max(map(len,paths))
        array=np.stack([np.vstack([path,np.repeat(path[-1,None],count-len(path),axis=0)]) for path in paths])
        last=np.array([len(path)-1 for path in paths]);idx=np.arange(n)
        xy=np.tile(p,(n,1)).astype(float);hd=np.full(n,yaw,dtype=float)
        vs=np.full(n,v,dtype=float);ws=np.full(n,w,dtype=float);poses=[xy.copy()];yaws=[hd.copy()];cmds=[]
        for _ in self.times[1:]:
            nearest=np.argmin(np.sum((array-xy[:,None])**2,axis=-1),axis=1)
            target=array[idx,np.minimum(nearest+10,last)]
            diff=target-xy;err=angle(np.arctan2(diff[:,1],diff[:,0])-hd)
            s=(xy-self.origin)@self.axis
            base=np.full(n,.23) if recovery else np.where(s<np.array([c['end'] for c in candidates]),.22,.30)
            speed=base*np.maximum(.8,np.cos(err))
            turn=np.clip(2*speed*np.sin(err)/np.maximum(.3,np.linalg.norm(diff,axis=1)),-.8,.8)
            a=np.where(speed<vs,self.limits['linear_decel_m_s2'],self.limits['linear_accel_m_s2'])
            vs+=np.clip(speed-vs,-a*.1,a*.1);ws+=np.clip(turn-ws,-self.limits['angular_accel_rad_s2']*.1,self.limits['angular_accel_rad_s2']*.1)
            xy+=vs[:,None]*np.column_stack([np.cos(hd),np.sin(hd)])*.1;hd+=ws*.1
            poses.append(xy.copy());yaws.append(hd.copy());cmds.append(np.column_stack([speed,turn]))
        return np.stack(poses,axis=1),np.stack(yaws,axis=1),np.stack(cmds,axis=1)

    @staticmethod
    def longest(mask):
        run=np.zeros(len(mask));longest=run.copy()
        for col in mask.T:
            run=np.where(col,run+.1,0);longest=np.maximum(longest,run)
        return longest

    def assess_v2(self,p,yaw,v,w,candidates,humans):
        xy,hd,cmd=self.rollout_paths(p,yaw,v,w,candidates,self.phase=='RECOVER')
        n=len(candidates);min_dist=np.full(n,np.inf);social=np.zeros(n)
        max_bearing=np.zeros(n);soft_time=np.zeros(n);hard_time=np.zeros(n);visible=np.ones(n);visibility_cost=np.zeros(n)
        cam=xy+np.stack([np.cos(hd)*self.camera_mount[0]-np.sin(hd)*self.camera_mount[1],
                        np.sin(hd)*self.camera_mount[0]+np.cos(hd)*self.camera_mount[1]],axis=-1)
        corridor=np.full(n,np.inf)
        for h in humans:
            delta=xy-h['predicted'][None]
            min_dist=np.minimum(min_dist,np.linalg.norm(delta,axis=-1).min(axis=1))
            social+=personal_field(delta,h['heading'],self.c,h['margin'][None])[0].mean(axis=1)*10
            if h['id']==self.target and not self.behind_confirmed:
                ray=h['predicted'][None]-cam
                bearing=np.abs(angle(np.arctan2(ray[:,:,1],ray[:,:,0])-hd))
                soft=bearing>self.camera_model['soft_limit'];hard=bearing>self.camera_model['hard_limit']
                max_bearing=np.max(bearing,axis=1);soft_time=self.longest(soft);hard_time=self.longest(hard)
                visible=np.mean(~hard,axis=1)
                visibility_cost=np.mean(np.maximum(0,(bearing-self.camera_model['soft_limit'])/(self.camera_model['hard_limit']-self.camera_model['soft_limit']))**2,axis=1)*10
                # A lateral corridor is only meaningful while the rollout
                # is longitudinally close to the person.  Checking the
                # entire 10 s prediction would reject a safe candidate just
                # because a small velocity estimate drifts laterally while
                # the robot is still metres behind the person.  Euclidean
                # distance remains the hard safety check over the full
                # rollout; this corridor adds the parallel-pass constraint
                # only in the interaction window.
                hvel=h.get('planner_velocity',h['velocity'])
                human_s=self.local(h['pos'])[0]+self.times*(hvel@self.axis)
                robot_s=(xy-self.origin)@self.axis
                near=np.abs(robot_s-human_s[None])<=1.2
                lateral=self.local(h['pos'])[1]+self.times*(hvel@self.normal)
                diff=np.abs(np.array([c['offset'] for c in candidates])[:,None]-lateral[None])
                corridor=np.min(np.where(near,diff,np.inf),axis=1)
        static=static_clearance(xy,self.geometry,self.radius).min(axis=1)
        fullstatic=np.array([static_clearance(c['full'],self.geometry,self.radius).min() for c in candidates])
        human_valid=(min_dist>=self.radius+self.c['human_radius'])
        if self.phase!='RECOVER':human_valid &= corridor>=self.radius+self.c['human_radius']
        static_valid=np.minimum(static,fullstatic)>=self.c['static_extra_margin']
        visibility_valid=hard_time<=self.camera_model.get('hard_duration_limit_s',.5)+1e-8
        valid=human_valid&static_valid&visibility_valid
        length=np.array([c['geometric_length'] for c in candidates]);effort=np.sum(cmd[:,:,1]**2,axis=1)*.1
        progress=(xy[:,-1]-p)@self.axis;final=np.array([c['final_lateral_error'] for c in candidates])
        score=4*social-progress+.12*(length+effort)+.15*final+visibility_cost
        diag=[]
        for i,c in enumerate(candidates):
            diag.append(dict(side=c['side'],lateral_offset=c['offset'],offset_length=c['length'],parallel_start_s=c['end'],recover_start_s=c['recover'],
                maximum_heading_deviation=c['maximum_heading_deviation'],maximum_curvature_proxy=c['maximum_curvature'],
                total_geometric_length=c['geometric_length'],final_lateral_error=c['final_lateral_error'],
                valid=bool(valid[i]),collision_valid=bool(human_valid[i]),static_valid=bool(static_valid[i]),visibility_valid=bool(visibility_valid[i]),
                min_clearance=float(min_dist[i]-self.radius-self.c['human_radius']),parallel_corridor_clearance=float(corridor[i]-self.radius-self.c['human_radius']),
                visible_fraction=float(visible[i]),max_abs_bearing_deg=float(np.degrees(max_bearing[i])),
                longest_predicted_out_of_soft_fov_s=float(soft_time[i]),longest_predicted_out_of_hard_fov_s=float(hard_time[i]),
                social_cost=float(social[i]),progress=float(progress[i]),path_length=float(length[i]),steering_effort=float(effort[i]),
                visibility_cost=float(visibility_cost[i]),total_score=float(score[i]),rollout_horizon_s=float(self.times[-1]),
                nominal_full_duration_s=float(length[i]/.22),full_path_completed_in_rollout=bool(np.linalg.norm(xy[i,-1]-c['full'][-1])<.3)))
        return valid,score,diag,cmd[:,0]

    def choose(self,p,yaw,v,w,sd,target,humans,now):
        signs=[np.sign(self.offset)] if self.offset else [1,-1]
        candidates=[self.candidate(sd,float(sign*d),length,target) for sign in signs for d in (.65,.8,.95) for length in (2.,2.5,3.)]
        valid,score,diag,_=self.assess_v2(p,yaw,v,w,candidates,humans)
        index=int(np.argmin(np.where(valid,np.round(score,9),np.inf))) if valid.any() else None
        self.candidate_history.append(dict(time=now,candidate_count=len(diag),collision_invalid=sum(not d['collision_valid'] for d in diag),
            static_invalid=sum(not d['static_valid'] for d in diag),visibility_invalid=sum(not d['visibility_valid'] for d in diag),valid_count=int(valid.sum()),
            selected_candidate=index,diagnostics=diag,geometric_paths=[c['full'].tolist() for c in candidates]))
        if index is None:return False
        self.chosen=candidates[index];self.offset=self.chosen['offset'];self.phase='OFFSET';return True

    def step(self,now,tracks,measurement_time,position,yaw,v=0.,omega=0.):
        started=time.perf_counter();p=np.asarray(position)[:2]
        if self.origin is None:
            self.origin=p.copy();self.axis=self.waypoints[-1]-p;self.axis/=np.linalg.norm(self.axis);self.normal=np.array([-self.axis[1],self.axis[0]])
        if self.camera_model is None:self.state='STOP';return [0.,0.]
        if now-self.last_plan<.1-1e-7:return self.command.tolist()
        self.last_plan=now;sd=self.local(p);humans=self.humans(now,tracks,measurement_time)
        target=next((h for h in humans if h['id']==self.target),None)
        if target is None and self.last_target:
            stamp,old=self.last_target;dt=now-stamp
            if old['age']+dt<=.8:
                oldvel=old.get('planner_velocity',old['velocity'])
                pos=old['pos']+oldvel*dt;matches=[h for h in humans if np.linalg.norm(h['pos']-pos)<.8]
                if len(matches)==1:target=matches[0];self.target=target['id']
                elif not matches:target=dict(old,pos=pos,age=old['age']+dt,planner_velocity=oldvel,predicted=pos+self.times[:,None]*oldvel);humans.append(target)
        if target is not None:self.last_target=(now,target)
        self.risks=[dict(track_id=h['id'],distance=float(np.linalg.norm(h['pos']-p))) for h in humans]
        self.stop_reason=None;diag=[];command=np.zeros(2)
        if self.phase in ('NAVIGATE','PASS_DONE'):
            possible=[h for h in humans if 0<self.local(h['pos'])[0]-sd[0]<6 and abs(self.local(h['pos'])[1])<self.radius+self.c['human_radius']]
            if possible:
                target=min(possible,key=lambda h:np.linalg.norm(h['pos']-p));self.target=target['id'];self.last_target=(now,target)
                self.phase='PASS_INIT';self.behind_confirmed=False;self.replans=0
                if not self.choose(p,yaw,v,omega,sd,target,humans,now):self.phase='EMERGENCY_STOP';self.stop_reason='NO_HARD_VALID_CANDIDATE'
        if self.phase=='OFFSET' and sd[0]>=self.chosen['end']-.1 and abs(sd[1]-self.offset)<.15:self.phase='PARALLEL'
        if self.phase=='PARALLEL' and target is not None and target['age']<=self.c['fresh_age'] and sd[0]-self.local(target['pos'])[0]>=.7:
            self.behind_confirmed=True;self.phase='RECOVER'
            ss=np.arange(sd[0],sd[0]+self.chosen['length']+.051,.05);dd=sd[1]*(1-self.smooth((ss-sd[0])/self.chosen['length']))
            self.chosen=dict(self.chosen,path=self.origin+ss[:,None]*self.axis+dd[:,None]*self.normal)
            self.chosen['full']=self.chosen['path']
        if self.phase=='RECOVER' and self.behind_confirmed and abs(sd[1])<=.3 and abs(angle(yaw-np.arctan2(self.axis[1],self.axis[0])))<.15:self.phase='PASS_DONE';self.target=None
        if self.phase in ('OFFSET','PARALLEL','RECOVER'):
            if target is None and not self.behind_confirmed:self.phase='EMERGENCY_STOP';self.stop_reason='TARGET_STALE_VISIBILITY_INTERRUPTED'
            else:
                valid,_,diag,commands=self.assess_v2(p,yaw,v,omega,[self.chosen],humans)
                # Receding-horizon visibility is a planning warning, not a
                # reason to stop while the current causal measurement is
                # fresh and still inside the physical hard FOV.  The initial
                # choice and every replan retain the hard predicted-FOV gate;
                # this continuation hold prevents a moving six-second window
                # from invalidating an already safe maneuver one sample later.
                if (not valid[0] and target is not None and diag and
                    diag[0]['collision_valid'] and diag[0]['static_valid'] and
                    target['age']<=self.c['fresh_age']):
                    cam=p+np.array([np.cos(yaw)*self.camera_mount[0]-np.sin(yaw)*self.camera_mount[1],
                                    np.sin(yaw)*self.camera_mount[0]+np.cos(yaw)*self.camera_mount[1]])
                    ray=target['pos']-cam
                    current_bearing=abs(float(angle(np.arctan2(ray[1],ray[0])-yaw)))
                    if current_bearing<=self.camera_model['hard_limit']:
                        valid[0]=True;diag[0]['valid']=True;diag[0]['visibility_deferred']=True
                if valid[0]:command=commands[0]
                elif target is not None and self.replans<3 and now-self.last_reshape>=1.:
                    self.replans+=1;self.last_reshape=now
                    if not self.choose(p,yaw,v,omega,sd,target,humans,now):self.phase='EMERGENCY_STOP';self.stop_reason='SAME_SIDE_REPLAN_INVALID'
                else:self.phase='EMERGENCY_STOP';self.stop_reason='CONTINUATION_HARD_INVALID'
        elif self.phase=='EMERGENCY_STOP' and target is not None and self.replans<3 and now-self.last_reshape>=1. and abs(v)<.03:
            self.replans+=1;self.last_reshape=now
            if not self.choose(p,yaw,v,omega,sd,target,humans,now):self.stop_reason='REPLAN_BUDGET_OR_FEASIBILITY'
        if self.phase in ('NAVIGATE','PASS_DONE'):
            command=self.command_to_path(p,yaw,np.linspace(p,self.waypoints[-1],100))
            if np.linalg.norm(p-self.waypoints[-1])<.3:self.done=True;command[:]=0
        if self.phase=='EMERGENCY_STOP':command[:]=0;self.stop_reason=self.stop_reason or 'LATCHED_AFTER_INVALID_OR_STALE'
        self.command=command;side='LEFT' if self.offset>0 else 'RIGHT' if self.offset<0 else 'NONE'
        self.state='STOP' if command[0]==0 else 'CRUISE' if self.phase in ('NAVIGATE','PASS_DONE') else 'AVOID_'+side
        self.selected=f'{self.phase} {side} {abs(self.offset):.2f}m'
        bearing=None
        if target is not None:
            cam=p+np.array([np.cos(yaw)*self.camera_mount[0]-np.sin(yaw)*self.camera_mount[1],np.sin(yaw)*self.camera_mount[0]+np.cos(yaw)*self.camera_mount[1]])
            ray=target['pos']-cam;bearing=float(np.degrees(angle(np.arctan2(ray[1],ray[0])-yaw)))
        self.logs.append(dict(time=now,state=self.state,selected=self.selected,passing_phase=self.phase,selected_side=side,
            selected_offset=self.offset,selected_offset_length=self.chosen['length'] if self.chosen else None,route_s=float(sd[0]),route_d=float(sd[1]),
            track_age=target['age'] if target is not None else None,actual_estimated_bearing_deg=bearing,
            behind_confirmed=self.behind_confirmed,pass_replan_count=self.replans,stop_reason=self.stop_reason,
            v_cmd=float(command[0]),omega_cmd=float(command[1]),continuation_diagnostics=diag,planning_ms=(time.perf_counter()-started)*1000))
        return command.tolist()
