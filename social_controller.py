"""Prediction-aware local primitives. Only estimated tracks and known map enter here.

No simulator imports, human GT, scenario name or future scripted trajectory.
"""
import time
import numpy as np


def angle(x):
    return np.arctan2(np.sin(x), np.cos(x))


def personal_field(delta, heading, config, margin=0.):
    """Nominal anisotropic Gaussian, or isotropic when direction is unknown."""
    if heading is None:
        norm2 = np.sum(delta**2, axis=-1)/(config['side_sigma']+margin)**2
    else:
        c, s = np.cos(heading), np.sin(heading)
        longitudinal = delta[..., 0]*c+delta[..., 1]*s
        lateral = -delta[..., 0]*s+delta[..., 1]*c
        sigma = np.where(longitudinal >= 0, config['front_sigma'], config['rear_sigma'])+margin
        norm2 = (longitudinal/sigma)**2+(lateral/(config['side_sigma']+margin))**2
    return np.exp(-.5*norm2), np.sqrt(norm2)


def static_clearance(points, geometry, radius):
    result = np.full(points.shape[:-1], np.inf)
    for box in geometry:
        vertices = np.asarray(box['polygon_xy'])
        lo, hi = vertices.min(axis=0), vertices.max(axis=0)
        # All frozen scene obstacles are axis-aligned rectangles.
        distance = np.linalg.norm(np.maximum(np.maximum(lo-points, points-hi), 0.), axis=-1)
        result = np.minimum(result, distance-radius)
    return result


class SocialController:
    def __init__(self, config, waypoints, geometry, radius, motion_limits):
        self.c = dict(config)
        self.waypoints = [np.asarray(p, float) for p in waypoints]
        self.geometry, self.radius, self.limits = geometry, radius, motion_limits
        self.index, self.done, self.state = 0, False, 'CRUISE'
        self.command = np.zeros(2)
        self.committed_side, self.commit_until = 0, -np.inf
        self.headings, self.memory = {}, {}
        self.last_plan, self.risks, self.logs = -np.inf, [], []
        self.selected = 'STRAIGHT'
        self.times = np.arange(round(self.c['prediction_horizon']/self.c['prediction_dt'])+1)*self.c['prediction_dt']

    def humans(self, now, tracks, measurement_time):
        result=[]
        for t in tracks:
            age=now-t['last_observed_time']
            state=np.asarray(t['state'],float)
            if age<0 or age>self.c['stale_age'] or not np.all(np.isfinite(state)):
                continue
            if float(t.get('confidence',1.))<self.c['min_confidence']:
                continue
            lag=max(0.,now-measurement_time) if measurement_time is not None else age
            pos=state[:2]+state[2:]*lag
            speed=np.linalg.norm(state[2:]);tid=t['track_id']
            if speed>=self.c['heading_min_speed']:
                self.headings[tid]=float(np.arctan2(state[3],state[2]))
            heading=self.headings.get(tid)
            predicted=pos+self.times[:,None]*state[2:]
            margin=np.zeros(len(self.times))
            if 'covariance' in t:
                p=np.asarray(t['covariance'],float)
                if p.shape==(4,4) and np.all(np.isfinite(p)):
                    dt=self.times+lag
                    xy=p[:2,:2]+dt[:,None,None]*(p[:2,2:]+p[2:,:2])+dt[:,None,None]**2*p[2:,2:]
                    margin=self.c['k_sigma']*np.sqrt(np.maximum(0.,np.linalg.eigvalsh(xy)[:,-1]))
            item=dict(id=tid,pos=pos,velocity=state[2:],heading=heading,predicted=predicted,
                      margin=margin,age=float(age),short_prediction=age>self.c['fresh_age'])
            result.append(item)
            self.memory[tid]=(now,pos.copy())
        self.memory={k:v for k,v in self.memory.items() if now-v[0]<=self.c['stale_guard_hold']}
        return result

    def rollout(self, position, yaw, v, omega, commands):
        n=len(commands);xy=np.tile(np.asarray(position)[:2],(n,1));heading=np.full(n,yaw)
        speed=np.full(n,v);turn=np.full(n,omega)
        positions=[xy.copy()];headings=[heading.copy()]
        dt=self.c['prediction_dt']
        for _ in self.times[1:]:
            acceleration=np.where(np.abs(commands[:,0])<np.abs(speed),self.limits['linear_decel_m_s2'],self.limits['linear_accel_m_s2'])
            speed+=np.clip(commands[:,0]-speed,-acceleration*dt,acceleration*dt)
            turn+=np.clip(commands[:,1]-turn,-self.limits['angular_accel_rad_s2']*dt,self.limits['angular_accel_rad_s2']*dt)
            xy=xy+speed[:,None]*np.column_stack((np.cos(heading),np.sin(heading)))*dt
            heading=heading+turn*dt
            positions.append(xy.copy());headings.append(heading.copy())
        return np.stack(positions,axis=1),np.stack(headings,axis=1)

    def group_pairs(self, humans):
        pairs=[]
        for i,a in enumerate(humans):
            for j in range(i+1,len(humans)):
                b=humans[j];va=np.linalg.norm(a['velocity']);vb=np.linalg.norm(b['velocity'])
                parallel=va>=.15 and vb>=.15 and a['velocity']@b['velocity']/(va*vb)>=self.c['group_direction_cos']
                slow=va<.15 and vb<.15
                if np.linalg.norm(a['pos']-b['pos'])<self.c['group_distance'] and (parallel or slow):pairs.append((i,j))
        return pairs

    def step(self, now, tracks, measurement_time, position, yaw, v=0., omega=0.):
        started=time.perf_counter();p=np.asarray(position)[:2]
        if np.linalg.norm(self.waypoints[self.index]-p)<.3:
            if self.index+1<len(self.waypoints):self.index+=1
            else:self.done=True
        if self.done:self.state='STOP';self.selected='GOAL_STOP';return [0.,0.]
        humans=self.humans(now,tracks,measurement_time)
        self.risks=[dict(track_id=h['id'],distance=float(np.linalg.norm(h['pos']-p))) for h in humans]
        emergency=any(r['distance']<self.radius+self.c['human_radius']+self.c['emergency_margin'] for r in self.risks)
        active_ids={h['id'] for h in humans}
        stale_danger=any(k not in active_ids and np.linalg.norm(q-p)<self.c['stale_near_distance'] for k,(_,q) in self.memory.items())
        if now-self.last_plan<self.c['prediction_dt']-1e-7:
            if emergency or stale_danger:self.state='STOP';return [0.,0.]
            return self.command.tolist()
        self.last_plan=now
        goal=self.waypoints[self.index];target_heading=np.arctan2(*(goal-p)[::-1])
        nominal=float(np.clip(1.2*angle(target_heading-yaw),-.5,.5))
        names=['STOP','SLOW_STRAIGHT','STRAIGHT','GENTLE_LEFT','LEFT','GENTLE_RIGHT','RIGHT']
        commands=np.array([[0,0],[.12,nominal],[self.c['cruise_speed'],nominal],
                           [.16,nominal+.225],[.12,nominal+.425],[.16,nominal-.225],[.12,nominal-.425]])
        commands[:,1]=np.clip(commands[:,1],-.8,.8)
        sides=np.array([0,0,0,1,1,-1,-1])
        xy,hd=self.rollout(p,yaw,v,omega,commands)
        clearance=static_clearance(xy,self.geometry,self.radius).min(axis=1)
        min_dist=np.full(7,np.inf);social=np.zeros(7)
        for h in humans:
            delta=xy-h['predicted'][None,:,:]
            min_dist=np.minimum(min_dist,np.linalg.norm(delta,axis=-1).min(axis=1))
            cost,_=personal_field(delta,h['heading'],self.c,h['margin'][None,:])
            social+=cost.mean(axis=1)
        group=np.zeros(7);pairs=self.group_pairs(humans)
        for i,j in pairs:
            a=humans[i]['predicted'];b=humans[j]['predicted'];ab=b-a
            t=np.clip(np.sum((xy-a)*ab,axis=-1)/np.maximum(np.sum(ab*ab,axis=-1),1e-8),0,1)
            distance=np.linalg.norm(xy-(a+t[...,None]*ab),axis=-1)
            group+=np.exp(-.5*(distance/self.c['group_sigma'])**2).mean(axis=1)
        hard=(min_dist>=self.radius+self.c['human_radius']) & (clearance>=self.c['static_extra_margin'])
        valid=hard.copy()
        switching=(sides*self.committed_side<0)
        side_before=self.committed_side
        side_safe=np.any(hard & (sides==self.committed_side))
        if now<self.commit_until and side_safe:valid[switching]=False
        goal_cost=np.linalg.norm(xy[:,-1]-goal,axis=1)
        control_cost=(commands[:,1]**2+np.sum((commands-self.command)**2,axis=1))
        # Use the current route bearing: an endpoint just beyond the waypoint
        # must not incur an artificial pi-radian turn-back penalty.
        heading_cost=np.abs(angle(hd[:,-1]-target_heading))
        parts=dict(goal=goal_cost,social=social+self.c['group_weight']*group,
                   control=control_cost,heading=heading_cost,switch=switching.astype(float),stop=(commands[:,0]==0).astype(float))
        total=sum(self.c['weights'][k]*value for k,value in parts.items())
        chosen=int(np.argmin(np.where(valid,total,np.inf))) if valid.any() else 0
        if emergency or stale_danger:chosen=0
        side=int(sides[chosen])
        if side and side!=self.committed_side:
            self.committed_side=side;self.commit_until=now+self.c['commit_hold']
        if not humans and now>=self.commit_until:self.committed_side=0
        self.selected=names[chosen];self.command=commands[chosen].copy()
        self.state=('STOP' if chosen==0 else 'SLOW' if chosen==1 else 'CRUISE' if chosen==2 else 'AVOID_LEFT' if side>0 else 'AVOID_RIGHT')
        self.logs.append(dict(time=float(now),selected=self.selected,state=self.state,command=self.command.tolist(),
            committed_side=self.committed_side,commit_until=float(self.commit_until),emergency=emergency,stale_guard=stale_danger,
            side_before=side_before,previous_side_hard_safe=bool(side_safe),held_opposite_candidates=int(np.sum(hard & ~valid)),
            no_safe_candidate=not bool(valid.any()),human_count=len(humans),group_count=len(pairs),
            estimated_humans=[dict(id=h['id'],position=h['pos'].tolist(),velocity=h['velocity'].tolist(),heading=h['heading'],age=h['age'],margin=h['margin'].tolist()) for h in humans],
            robot_position=p.tolist(),candidates=xy.tolist(),
            costs=[dict(name=names[i],total=float(total[i]),valid=bool(valid[i]),hard_valid=bool(hard[i]),
                        min_human_distance=float(min_dist[i]),static_clearance=float(clearance[i]),
                        **{k:float(value[i]) for k,value in parts.items()}) for i in range(7)],
            planning_ms=(time.perf_counter()-started)*1000))
        return self.command.tolist()
