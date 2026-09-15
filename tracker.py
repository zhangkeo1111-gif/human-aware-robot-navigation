"""Causal world-XY CV Kalman tracker. No simulator or GT inputs."""
import numpy as np
from scipy.optimize import linear_sum_assignment


def person_depth(depth, box, config, intrinsics=(240.,240.,320.,180.)):
    """Supported foreground torso depth; no identity, simulator state or GT input."""
    h,w = depth.shape
    x1,y1,x2,y2 = map(float,box)
    bw,bh = x2-x1,y2-y1
    margin = config['border_margin_px']
    truncated = x1<margin or y1<margin or x2>w-margin or y2>h-margin
    # Visible torso, excluding floor/feet. Side-truncated boxes retain their visible width.
    left,right = (.15,.85) if truncated else (.25,.75)
    # A clipped detector box describes the visible fragment, not the full person.
    # Over-shrinking its border-facing side can remove the only foreground pixels.
    top,bottom = (.20,.55) if truncated else (.20,.65)
    xa,xb = np.clip([int(x1+left*bw),int(x1+right*bw)+1],0,w)
    ya,yb = np.clip([int(y1+top*bh),int(y1+bottom*bh)+1],0,h)
    roi = depth[ya:yb,xa:xb]
    mask = np.isfinite(roi)&(roi>.05)&(roi<100.)
    values = roi[mask]
    result = {'roi':[int(xa),int(ya),int(xb),int(yb)],'truncated':bool(truncated),
        'depth_valid_ratio':float(mask.mean()) if mask.size else 0.,
        'depth_spread':None,'depth_confidence':0.,'accepted':False,'reason':'insufficient_depth'}
    if len(values)<config['min_pixels']:
        return result
    # A nearer quantile anchors a supported band, never the minimum-depth pixel.
    anchor = np.percentile(values,config['foreground_percentile'] if truncated else 50)
    tolerance = max(config['band_floor_m'],config['band_fraction']*anchor)
    selected = mask & (np.abs(roi-anchor)<=tolerance)
    foreground = roi[selected]
    if len(foreground)<config['min_pixels']:
        return result
    z = float(np.median(foreground))
    spread = float(np.percentile(foreground,90)-np.percentile(foreground,10))
    yy,xx = np.nonzero(selected)
    support = len(foreground)/len(values)
    confidence = min(1.,support/config['target_support'])*max(0.,1-spread/max(tolerance*2,1e-6))
    accepted = support>=config['min_support'] and confidence>=config['min_confidence']
    fx,fy,cx,cy = intrinsics
    points = np.column_stack([((xx+xa)-cx)*foreground/fx,((yy+ya)-cy)*foreground/fy,foreground])
    centered = points-points.mean(axis=0)
    # Near-horizontal ground creates constant vertical camera coordinates, unlike a torso.
    # Use all supported pixels, not simulator ground height or person GT.
    _,singular,axes = np.linalg.svd(centered[::max(1,len(centered)//512)],full_matrices=False)
    normal = axes[-1]
    planar_residual = float(np.median(np.abs(centered @ normal)))
    ground_like = abs(normal[1])>config['ground_normal_y_min'] and planar_residual<config['plane_residual_max_m'] and np.ptp(yy)>=2
    accepted = accepted and not ground_like
    result.update(depth_m=z,u=float(np.median(xx)+xa),v=float(np.median(yy)+ya),
        depth_spread=spread,depth_confidence=float(confidence),foreground_support=float(support),
        ground_like=bool(ground_like),plane_residual_m=planar_residual,
        accepted=bool(accepted),reason='accepted' if accepted else ('ground_like_surface' if ground_like else 'weak_foreground_support'))
    return result


class Tracker:
    def __init__(self, config):
        self.config = config
        self.tracks = []
        self.time = None
        self.next_id = 1
        self.rejected = []

    def update(self, sim_time, measurements):
        if self.time is not None and sim_time <= self.time:
            raise ValueError('Measurements must have unique increasing simulation timestamps')
        dt = 0 if self.time is None else sim_time-self.time
        self.time = sim_time
        self.rejected = []
        f = np.eye(4)
        f[:2,2:] = np.eye(2)*dt
        g = np.vstack([np.eye(2)*dt*dt/2,np.eye(2)*dt])
        q = g @ g.T*self.config['acceleration_sigma_m_s2']**2
        r = np.eye(2)*self.config['measurement_sigma_m']**2
        for t in self.tracks:
            t['state'] = f @ t['state']
            t['covariance'] = f @ t['covariance'] @ f.T+q
            t['age'] += 1
            t['miss_count'] += 1
        used = set()
        if self.tracks and measurements:
            costs = np.array([[np.linalg.norm(t['state'][:2]-p) for p,c in measurements] for t in self.tracks])
            gate = self.config['association_gate_m']
            # Explicit dummy columns allow unmatched tracks instead of forcing invalid matches.
            padded = np.column_stack([np.where(costs<=gate,costs,1e6),np.full((len(self.tracks),len(self.tracks)),gate+1e-6)])
            rows,cols = linear_sum_assignment(padded)
            for i,j in zip(rows,cols):
                if j>=len(measurements) or costs[i,j]>gate:
                    continue
                t = self.tracks[i]
                p = t['covariance']
                innovation = measurements[j][0]-t['state'][:2]
                mahalanobis = float(innovation @ np.linalg.solve(p[:2,:2]+r,innovation))
                if mahalanobis>self.config.get('innovation_gate_chi2',float('inf')):
                    self.rejected.append({'measurement_index':int(j),'reason':'innovation_gate','mahalanobis_squared':mahalanobis})
                    used.add(j)  # Rejected update must not become a spurious new birth.
                    continue
                k = np.linalg.solve(p[:2,:2]+r,p[:,:2].T).T
                t['state'] += k @ (measurements[j][0]-t['state'][:2])
                kh = np.zeros((4,4)); kh[:,:2] = k
                a = np.eye(4)-kh
                t['covariance'] = a @ p @ a.T+k @ r @ k.T
                t['miss_count'] = 0
                t['last_observed_time'] = sim_time
                t['confidence'] = measurements[j][1]
                used.add(j)
        self.tracks = [t for t in self.tracks if t['miss_count']<=self.config['max_missed_frames']]
        for j,(position,confidence) in enumerate(measurements):
            if j not in used:
                self.tracks.append({'track_id':self.next_id,'state':np.r_[position,0.,0.],
                    'covariance':np.diag([self.config['measurement_sigma_m']**2]*2+[self.config['initial_velocity_sigma_m_s']**2]*2),
                    'age':1,'miss_count':0,'confidence':confidence,'last_observed_time':sim_time})
                self.next_id += 1
        return [{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in t.items()} for t in self.tracks]


def relative_risk(human_state, robot_position, robot_velocity, safe_radius):
    """All inputs in world XY; times seconds, distances meters. No GT access."""
    r = np.asarray(human_state[:2])-robot_position
    v = np.asarray(human_state[2:])-robot_velocity
    distance = float(np.linalg.norm(r))
    vv = float(v @ v)
    t_cpa = -float(r @ v)/vv if vv>1e-8 else float('inf')
    d_cpa = float(np.linalg.norm(r+v*t_cpa)) if np.isfinite(t_cpa) else distance
    # First intersection of the relative-motion ray with the safety circle.
    b = float(r @ v)
    c = distance*distance-safe_radius*safe_radius
    discriminant = b*b-vv*c
    ttc = 0. if c<=0 else float('inf')
    if c>0 and vv>1e-8 and b<0 and discriminant>=0:
        ttc = (-b-np.sqrt(discriminant))/vv
    return {'distance_m':distance,'closing_speed_m_s':-b/max(distance,1e-8),
            't_cpa_s':t_cpa,'d_cpa_m':d_cpa,'ttc_s':float(ttc)}


class BrakeController:
    def __init__(self, config):
        self.c = config
        self.state = 'STOP'
        self.clear_cycles = 0
        self.command = 0.
        self.last_measurement = None

    def step(self, now, dt, tracks, measurement_time, robot_position, robot_velocity):
        c = self.c
        safe_radius = c['robot_radius_m']+c['human_radius_m']+c['extra_margin_m']
        fresh_cycle = measurement_time is not None and measurement_time!=self.last_measurement
        stale = measurement_time is None or now-measurement_time>c['stale_timeout_s']
        self.last_measurement = measurement_time
        risks = []
        for t in tracks:
            x = np.array(t['state'],dtype=float)
            x[:2] += x[2:]*max(0.,now-measurement_time)
            risks.append({'track_id':t['track_id'],**relative_risk(x,robot_position,robot_velocity,safe_radius)})
        emergency = any(r['distance_m']<c['hard_stop_distance_m'] for r in risks)
        stop = stale or emergency or any(r['ttc_s']<c['stop_ttc_s'] for r in risks)
        warning = any(r['ttc_s']<c['warning_ttc_s'] or (r['distance_m']<c['warning_distance_m'] and r['closing_speed_m_s']>0) for r in risks)
        clear = not stale and all(r['distance_m']>c['resume_distance_m'] and r['ttc_s']>=c['warning_ttc_s'] for r in risks)
        if stop:
            self.state,self.clear_cycles = 'STOP',0
        elif self.state=='STOP':
            if fresh_cycle:
                self.clear_cycles = self.clear_cycles+1 if clear else 0
            if self.clear_cycles>=c['resume_cycles']:
                self.state = 'BRAKE' if warning else 'CRUISE'
        else:
            self.state = 'BRAKE' if warning else 'CRUISE'
        target = 0. if self.state=='STOP' else c['nominal_speed_m_s']*(.5 if self.state=='BRAKE' else 1.)
        if emergency:
            self.command = 0.
        else:
            self.command += float(np.clip(target-self.command,-c['deceleration_m_s2']*dt,c['acceleration_m_s2']*dt))
        return self.command,risks


def stopping_rollout(position, velocity, heading, humans, config):
    """Brake with the measured forward speed, then hold; CV people, no GT."""
    axis = np.array([np.cos(heading),np.sin(heading)])
    speed = max(0.,float(np.asarray(velocity) @ axis))
    decel = config['deceleration_m_s2']
    halt = np.asarray(position)+axis*speed*speed/(2*decel)
    pass_times = [max(0.,-float((h[:2]-halt)@h[2:])/max(float(h[2:]@h[2:]),1e-8)) for h in humans]
    horizon = max(config['stop_horizon_s'],speed/decel,max(pass_times,default=0.)+1.)
    horizon = min(horizon,config['rollout_max_s'])
    ts = np.arange(0,horizon+config['candidate_step_s']/2,config['candidate_step_s'])
    braking = np.minimum(ts,speed/decel)
    path = np.asarray(position)+(speed*braking-.5*decel*braking**2)[:,None]*axis
    distances = [np.linalg.norm(path-(h[:2]+ts[:,None]*h[2:]),axis=1) for h in humans]
    ds = np.min(distances,axis=0) if distances else np.full(len(ts),np.inf)
    i = int(np.argmin(ds))
    radius = config['robot_radius_m']+config['human_radius_m']+config['extra_margin_m']
    return {'stop_candidate_min_distance':float(ds[i]),'stop_candidate_t_min':float(ts[i]),
        'stop_candidate_safe':bool(ds[i]>=radius),'stop_horizon_s':horizon}


class ArcController(BrakeController):
    """Two short unicycle candidates, using estimated human states only."""
    def __init__(self, config):
        super().__init__(config)
        self.side = 0
        self.side_since = 0.
        self.reference_heading = None
        self.omega = 0.
        self.safe_cycles = 0
        self.diagnostics = {}

    @staticmethod
    def angle(value):
        return float(np.arctan2(np.sin(value),np.cos(value)))

    def candidate(self, side, position, heading, humans):
        c = self.c
        delta = self.angle(self.reference_heading+side*c['avoid_heading_rad']-heading)
        omega = float(np.clip(2*delta,-c['avoid_omega_rad_s'],c['avoid_omega_rad_s']))
        ts = np.arange(0,c['candidate_horizon_s']+1e-8,c['candidate_step_s'])
        v = c['avoid_speed_m_s']
        turn = np.minimum(ts,abs(delta/omega)) if abs(omega)>1e-6 else np.zeros_like(ts)
        yaw = heading+omega*turn
        if abs(omega)>1e-6:
            dx = v/omega*(np.sin(yaw)-np.sin(heading))
            dy = v/omega*(np.cos(heading)-np.cos(yaw))
        else:
            dx,dy = np.zeros_like(ts),np.zeros_like(ts)
        path = np.asarray(position)+np.column_stack([dx+v*(ts-turn)*np.cos(yaw),dy+v*(ts-turn)*np.sin(yaw)])
        minimum = min((float(np.linalg.norm(path-(h[:2]+ts[:,None]*h[2:]),axis=1).min()) for h in humans),default=float('inf'))
        return minimum,omega

    def step(self, now, dt, tracks, measurement_time, robot_position, robot_velocity, heading, depth_ok=True):
        fresh = measurement_time is not None and measurement_time!=self.last_measurement
        previous_command = self.command
        if self.reference_heading is None:
            self.reference_heading = heading
        speed,risks = super().step(now,dt,tracks,measurement_time,robot_position,robot_velocity)
        c = self.c
        radius = c['robot_radius_m']+c['human_radius_m']+c['extra_margin_m']
        humans = []
        for t in tracks:
            h = np.asarray(t['state'],dtype=float).copy()
            h[:2] += h[2:]*max(0.,now-measurement_time)
            humans.append(h)
        left,lw = self.candidate(1,robot_position,heading,humans)
        right,rw = self.candidate(-1,robot_position,heading,humans)
        stopped = [relative_risk(h,robot_position,np.zeros(2),radius) for h in humans]
        conflict = any(r['t_cpa_s']>0 and r['d_cpa_m']<radius for r in risks)
        stop_conflict = any(r['t_cpa_s']>0 and r['d_cpa_m']<radius for r in stopped)
        stale = measurement_time is None or now-measurement_time>c['stale_timeout_s'] or any(now-t['last_observed_time']>c['stale_timeout_s'] for t in tracks)
        emergency = any(r['distance_m']<c['hard_stop_distance_m'] for r in risks)
        blocked = stale or not depth_ok or emergency or (left<radius and right<radius) or (self.side!=0 and not tracks)
        if not self.side and conflict and stop_conflict and any(t['age']>=3 for t in tracks) and not blocked:
            self.side = 1 if left>=right else -1
            self.side_since = now
            self.safe_cycles = 0
        target_omega = 0.
        if self.side:
            selected = left if self.side==1 else right
            other = right if self.side==1 else left
            # Hold direction unless unsafe; never trade safety for hysteresis.
            if selected<radius and other>=radius:
                self.side *= -1
                self.side_since = now
            clear = bool(tracks) and all(r['distance_m']>c['resume_distance_m'] and (r['t_cpa_s']<=0 or r['d_cpa_m']>=radius) for r in risks)
            if fresh:
                self.safe_cycles = self.safe_cycles+1 if clear else 0
            if self.safe_cycles>=c['resume_cycles'] and now-self.side_since>=c['avoid_hold_s']:
                self.side = 0
                self.state = 'CRUISE'
            elif not blocked:
                self.state = 'AVOID_LEFT' if self.side==1 else 'AVOID_RIGHT'
                self.command = previous_command
                self.command += float(np.clip(c['avoid_speed_m_s']-self.command,-c['deceleration_m_s2']*dt,c['acceleration_m_s2']*dt))
                target_omega = lw if self.side==1 else rw
        if blocked:
            self.state,self.command = 'STOP',0.
            self.omega = 0.
        elif not self.side and self.state=='CRUISE':
            target_omega = float(np.clip(self.angle(self.reference_heading-heading),-.3,.3))
        self.omega += float(np.clip(target_omega-self.omega,-c['angular_acceleration_rad_s2']*dt,c['angular_acceleration_rad_s2']*dt))
        self.diagnostics = {'risk_state':'BLOCKED' if blocked else ('CPA_CONFLICT' if conflict else 'CLEAR'),
            'blocked_stale':bool(stale),'blocked_depth':not depth_ok,'blocked_hard_distance':bool(emergency),
            'blocked_candidates':bool(left<radius and right<radius),'blocked_lost_target':bool(self.side!=0 and not tracks),
            'selected_action':self.state,'candidate_left_min_distance':left,'candidate_right_min_distance':right,
            'selected_omega':self.omega,'selected_v':self.command,'stop_corridor_conflict':stop_conflict}
        return self.command,risks


class LateralController(BrakeController):
    """STOP-first staged escape. Only estimated states and robot self-state enter."""
    angle = staticmethod(ArcController.angle)

    def __init__(self, config):
        super().__init__(config)
        self.omega = 0.
        self.side = 0
        self.active_id = None
        self.cache = {}
        self.origin = None
        self.origin_heading = None
        self.hold_until = 0.
        self.clear_count = 0
        self.escape_phase = None
        self.side_switch_count = 0
        self.phase_start_time = 0.
        self.escape_angle = config['escape_heading_rad']
        self.candidate_audit = []
        self.cleared_ids = set()
        self.recovery_completed = False
        self.active_anchor = None
        self.diagnostics = {}

    def rollout(self, side, position, heading, speed, humans, angle=None, detailed=False):
        c = self.c
        angle = self.escape_angle if angle is None else angle
        if not humans:
            if detailed:
                return {'side':side,'angle':float(angle),'minimum':float('inf'),'robust_minimum':float('inf'),
                    't_min':0.,'completion_time':0.,'prior_margin':float('inf'),'visible_fraction':0.,'longest_blind_s':0.,'max_abs_bearing_rad':0.,
                    'wheel_peak_rad_s':0.,'final_pose':[float(position[0]),float(position[1]),float(heading)],'horizon':0.}
            return float('inf'),0.,0.
        origin = np.asarray(position) if self.origin is None else self.origin
        reference = heading if self.origin_heading is None else self.origin_heading
        lateral = np.array([-np.sin(reference),np.cos(reference)])
        target = reference+side*angle
        p = np.asarray(position).copy()
        yaw,v,w = heading,speed,self.omega
        remaining = max(0.,c['target_lateral_m']-side*float((p-origin)@lateral))
        horizon = max(c['stop_horizon_s'],remaining/(c['translate_speed_m_s']*np.sin(angle))+abs(self.angle(target-heading))/c['avoid_omega_rad_s']+2.)
        horizon = min(c['rollout_max_s'],horizon)
        dt = c['candidate_step_s']
        minimum,visibility = float('inf'),0
        blind = np.zeros(len(humans)); longest = 0.; max_bearing = 0.; wheel_peak = 0.; robust_min = float('inf'); t_min = 0.
        covs = [x.copy() for x in getattr(self,'candidate_covariances',[])]
        prior_f = np.column_stack([np.eye(2),np.eye(2)*c['prediction_max_age_s']])
        prior_covs = [prior_f@x@prior_f.T for x in covs]
        prior_margin = float('inf')
        transition = np.eye(4);transition[:2,2:] = np.eye(2)*dt
        noise_map = np.vstack([np.eye(2)*dt*dt/2,np.eye(2)*dt])
        translating,complete = False,False
        completion_time = float('inf')
        for tick,stamp in enumerate(np.arange(0,horizon+dt/2,dt)):
            error = self.angle(target-yaw)
            translating = translating or abs(error)<c['heading_tolerance_rad']
            reached = side*float((p-origin)@lateral)>=c['target_lateral_m']
            if reached and not np.isfinite(completion_time):
                completion_time = float(stamp)
            if reached:
                stop_clear = all(np.linalg.norm((h[:2]+stamp*h[2:]-p)+max(0.,-float((h[:2]+stamp*h[2:]-p)@h[2:])/max(float(h[2:]@h[2:]),1e-8))*h[2:])>=c['robot_radius_m']+c['human_radius_m']+c['extra_margin_m'] for h in humans)
                complete = complete or stop_clear
            goal_v = 0. if complete else (c['translate_speed_m_s'] if translating else c['turn_speed_m_s'])
            goal_w = 0. if complete else float(np.clip(3*error,-c['avoid_omega_rad_s'],c['avoid_omega_rad_s']))
            for index,h in enumerate(humans):
                delta = h[:2]+stamp*h[2:]-p
                distance = float(np.linalg.norm(delta))
                if distance<minimum:
                    minimum,t_min = distance,stamp
                bearing = abs(self.angle(np.arctan2(delta[1],delta[0])-yaw))
                visible = any(abs(self.angle(np.arctan2(delta[1],delta[0])-yaw-offset))<c['camera_half_fov_rad'] for offset in c.get('camera_yaw_offsets_rad',[0.]))
                visibility += visible
                # Visibility is required while approaching, not indefinitely after passage.
                approaching = delta@h[2:]<0
                blind[index] = blind[index]+dt if not visible and approaching else 0.
                longest = max(longest,blind[index]);max_bearing=max(max_bearing,bearing)
                inflation = 0.
                if index<len(covs):
                    covariance = transition@covs[index]@transition.T+noise_map@noise_map.T*c['prediction_acceleration_sigma']**2
                    if visible and tick%2==0:
                        gain = np.linalg.solve(covariance[:2,:2]+np.eye(2)*c.get('measurement_sigma_m',.15)**2,covariance[:,:2].T).T
                        covariance = covariance-gain@covariance[:2,:]
                    covs[index] = covariance
                    normal = delta/max(distance,1e-8)
                    inflation = np.sqrt(max(0.,float(normal@covariance[:2,:2]@normal)))
                    prior_margin = min(prior_margin,distance-np.sqrt(max(0.,float(normal@prior_covs[index]@normal))))
                robust_min = min(robust_min,distance-inflation)
            v += float(np.clip(goal_v-v,-c['deceleration_m_s2']*dt,c['acceleration_m_s2']*dt))
            w += float(np.clip(goal_w-w,-c['angular_acceleration_rad_s2']*dt,c['angular_acceleration_rad_s2']*dt))
            p += v*dt*np.array([np.cos(yaw),np.sin(yaw)])
            yaw += w*dt
            wheel_peak = max(wheel_peak,abs((v-w*c.get('track_width_m',.1125)/2)/c.get('wheel_radius_m',.03)),abs((v+w*c.get('track_width_m',.1125)/2)/c.get('wheel_radius_m',.03)))
        if detailed:
            return {'side':side,'angle':angle,'minimum':minimum,'robust_minimum':robust_min,'prior_margin':float(prior_margin),'t_min':t_min,'completion_time':completion_time,
                'visible_fraction':visibility/max(1,(tick+1)*len(humans)),'longest_blind_s':longest,
                'max_abs_bearing_rad':float(max_bearing),'wheel_peak_rad_s':float(wheel_peak),'final_pose':[float(p[0]),float(p[1]),float(yaw)],'horizon':float(horizon)}
        return minimum,visibility/max(1,(tick+1)*len(humans)),horizon

    def step(self, now, dt, tracks, measurement_time, robot_position, robot_velocity, heading, depth_ok=True):
        c = self.c
        radius = c['robot_radius_m']+c['human_radius_m']+c['extra_margin_m']
        fresh = measurement_time is not None and measurement_time!=self.last_measurement
        self.last_measurement = measurement_time
        if fresh:
            for t in tracks:
                self.cache[t['track_id']] = (t,measurement_time)
        # Preserve a short-lived active KF state after the frontend lifecycle removes it.
        self.cache = {k:v for k,v in self.cache.items() if now-v[0]['last_observed_time']<=c['prediction_max_age_s']}
        predicted,all_predictions = {},{}
        for k,(t,stamp) in self.cache.items():
            age = max(0.,now-stamp)
            f = np.eye(4);f[:2,2:] = np.eye(2)*age
            g = np.vstack([np.eye(2)*age*age/2,np.eye(2)*age])
            cov = f@np.asarray(t['covariance'])@f.T+g@g.T*c['prediction_acceleration_sigma']**2
            all_predictions[k] = (f@np.asarray(t['state']),cov,t)
            valid = np.linalg.eigvalsh(cov[:2,:2]).max()<=c['prediction_position_variance_max'] and np.linalg.eigvalsh(cov[2:,2:]).max()<=c['prediction_velocity_variance_max']
            if valid:
                predicted[k] = (f@np.asarray(t['state']),cov,t)
        if fresh:
            for k,v in predicted.items():
                renewed = relative_risk(v[0],robot_position,robot_velocity,radius)
                if 0<renewed['t_cpa_s']<=c['stop_horizon_s'] and renewed['d_cpa_m']<radius and renewed['closing_speed_m_s']>0:
                    self.cleared_ids.discard(k)
        uncleared = {k:v for k,v in predicted.items() if k not in self.cleared_ids}
        if fresh and self.active_id is not None and self.active_id not in predicted and self.active_anchor is not None:
            anchor,anchor_time = self.active_anchor
            expected = anchor[:2]+anchor[2:]*max(0.,now-anchor_time)
            matches = [k for k,v in uncleared.items() if np.linalg.norm(v[0][:2]-expected)<=c.get('association_gate_m',.8)]
            if len(matches)==1:
                self.active_id = matches[0]
        if self.active_id is None and uncleared:
            self.active_id = min(uncleared,key=lambda k:np.linalg.norm(uncleared[k][0][:2]-robot_position))
        if self.active_id in predicted:
            self.active_anchor = (predicted[self.active_id][0].copy(),now)
        # Never switch active identity to a duplicate merely because it is fresher.
        humans = [v[0] for v in predicted.values()]
        risks = [{'track_id':k,**relative_risk(v[0],robot_position,robot_velocity,radius)} for k,v in predicted.items()]
        stop = stopping_rollout(robot_position,robot_velocity,heading,humans,c)
        forward,lateral = 0.,0.
        if self.origin is not None:
            delta = np.asarray(robot_position)-self.origin
            forward = float(delta@np.array([np.cos(self.origin_heading),np.sin(self.origin_heading)]))
            lateral = float(delta@np.array([-np.sin(self.origin_heading),np.cos(self.origin_heading)]))
        active = predicted.get(self.active_id)
        # Heavy candidate evaluation is limited to fresh sensor cycles.
        if fresh or not self.diagnostics:
            self.candidate_covariances = [v[1] for v in predicted.values()]
            self.candidate_audit = [self.rollout(side,robot_position,heading,max(0.,float(np.linalg.norm(robot_velocity))),humans,angle,True)
                for side in (1,-1) for angle in (np.pi/6,c['escape_heading_rad'],np.radians(80.))]
            choices = []
            for side in (1,-1):
                group = [x for x in self.candidate_audit if x['side']==side]
                timely = [x for x in group if stop['stop_candidate_safe'] or x['completion_time']<=stop['stop_candidate_t_min']]
                feasible = [x for x in timely if x['minimum']>=radius and x['robust_minimum']>=c['robot_radius_m']+c['human_radius_m'] and x['longest_blind_s']<=c['prediction_max_age_s'] and x['wheel_peak_rad_s']<=c.get('wheel_velocity_limit_rad_s',12.56)]
                if not feasible and not stop['stop_candidate_safe']:
                    feasible = [x for x in timely if x['robust_minimum']>=c['robot_radius_m']+c['human_radius_m'] and x['longest_blind_s']<=c['prediction_max_age_s'] and x['wheel_peak_rad_s']<=c.get('wheel_velocity_limit_rad_s',12.56)]
                    for x in feasible:
                        x['risk_mitigation_only'] = True
                retained = next((x for x in feasible if self.side==side and abs(x['angle']-self.escape_angle)<1e-6),None)
                best = retained or (min(feasible,key=lambda x:(x['prior_margin']<radius,abs(self.angle((self.origin_heading if self.origin_heading is not None else heading)+side*x['angle']-heading)))) if feasible else max(group,key=lambda x:x['robust_minimum']))
                choices.append((best['minimum'] if feasible else -1.,best['visible_fraction'],best['horizon']))
                if feasible:
                    best['feasible'] = True
            left,right = choices
            self.candidates = (left,right)
        left,right = self.candidates
        near_distance_guard = any(r['distance_m']<c['hard_stop_distance_m'] for r in risks)
        verified_escape = bool(self.side and active and risks and all(r['distance_m']>=radius for r in risks) and
            any(x['side']==self.side and abs(x['angle']-self.escape_angle)<1e-6 and x.get('feasible') and x['minimum']>=radius for x in self.candidate_audit))
        stationary_clear = bool(self.state in ('CLEAR','RECOVER','REACQUIRE') and stop['stop_candidate_safe'] and risks and all(r['distance_m']>=radius for r in risks))
        departing_clear = bool((self.state in ('RECOVER','RESUME') or self.recovery_completed) and stop['stop_candidate_safe'] and risks and all(r['distance_m']>=radius and (r['closing_speed_m_s']<=0 or r['t_cpa_s']>c['stop_horizon_s'] or r['d_cpa_m']>=radius) for r in risks))
        emergency = near_distance_guard and not (verified_escape or stationary_clear or departing_clear)
        stale_sensor = measurement_time is None or now-measurement_time>c['stale_timeout_s']
        uncertain = (self.active_id is not None and active is None) or (bool(tracks) and not predicted)
        blocked = emergency or stale_sensor or uncertain or not depth_ok
        phase = self.escape_phase if self.side and self.escape_phase else self.state
        if self.state=='REACQUIRE' and self.side and active and stop['stop_candidate_safe'] and abs(self.angle(self.origin_heading-heading))<c['heading_tolerance_rad']:
            phase = 'CLEAR';self.clear_count = 0
        both_unsafe = left[0]<radius and right[0]<radius
        available_sides = {x['side'] for x in self.candidate_audit if x.get('feasible')}
        if not self.side and not blocked and active and not stop['stop_candidate_safe']:
            if not available_sides:
                blocked = True
            else:
                scores = [x[0]-c['visibility_penalty_m']*(1-x[1]) if side in available_sides else -np.inf for side,x in zip((1,-1),[left,right])]
                self.side = 1 if scores[0]>=scores[1] else -1
                self.escape_angle = next(x['angle'] for x in self.candidate_audit if x['side']==self.side and x.get('feasible'))
                self.origin = np.asarray(robot_position).copy()
                self.origin_heading = heading
                self.hold_until = now+c['avoid_hold_s']
                self.side_switch_count = 0
                self.recovery_completed = False
                phase = 'TURN_LEFT' if self.side==1 else 'TURN_RIGHT'
        goal_v,goal_w = 0.,0.
        switch_allowed = False
        switch_reason = None
        switch_block = None
        current_min = opposite_min = None
        decision_side = self.side
        if self.side:
            if fresh:
                chosen = next((x for x in self.candidate_audit if x['side']==self.side and x.get('feasible')),None)
                if chosen is not None and abs(chosen['angle']-self.escape_angle)>1e-6:
                    self.escape_angle = chosen['angle']
                    self.hold_until = now+c['avoid_hold_s']
                    phase = 'TURN_LEFT' if self.side==1 else 'TURN_RIGHT'
            selected = left if self.side==1 else right
            opposite = right if self.side==1 else left
            current_min,opposite_min = selected[0],opposite[0]
            if self.side not in available_sides and phase not in ('CLEAR','RECOVER','REACQUIRE'):
                if blocked:
                    switch_block = 'hard_distance' if emergency else ('uncertainty' if uncertain else 'sensor_or_depth')
                elif self.side_switch_count:
                    switch_block = 'switched_side_became_unsafe'
                elif not fresh:
                    switch_block = 'await_fresh_candidates'
                elif opposite[0]<radius:
                    switch_block = 'both_candidates_unsafe'
                elif opposite[0]-selected[0]<c['side_switch_improvement_margin_m']:
                    switch_block = 'insufficient_improvement'
                elif now<self.hold_until and selected[0]>=c['robot_radius_m']+c['human_radius_m']:
                    switch_block = 'side_hold'
                else:
                    switch_allowed = True
                    switch_reason = 'selected_side_unsafe_opposite_safe'
                    self.side *= -1
                    self.escape_angle = next(x['angle'] for x in self.candidate_audit if x['side']==self.side and x.get('feasible'))
                    self.side_switch_count += 1
                    self.hold_until = now+c['avoid_hold_s']
                    phase = 'SWITCH_TO_LEFT' if self.side==1 else 'SWITCH_TO_RIGHT'
                if not switch_allowed:
                    blocked = True
            target = self.origin_heading+self.side*self.escape_angle
            error = self.angle(target-heading)
            if phase.startswith('SWITCH') and not switch_allowed:
                phase = 'TURN_LEFT' if self.side==1 else 'TURN_RIGHT'
            if phase.startswith('TURN') and abs(error)<=c['heading_tolerance_rad']:
                phase = 'TRANSLATE_LEFT' if self.side==1 else 'TRANSLATE_RIGHT'
            clear_geometry = bool(risks) and all(r['distance_m']>=radius and r['d_cpa_m']>=radius for r in risks)
            confidence_f = np.column_stack([np.eye(2),np.eye(2)*c['prediction_max_age_s']])
            stop_uncertainty = max((np.sqrt(max(0.,float(np.linalg.eigvalsh(confidence_f@v[1]@confidence_f.T).max()))) for v in predicted.values()),default=float('inf'))
            stop_confident = stop['stop_candidate_min_distance']-stop_uncertainty>=radius
            if phase=='CLEAR' and not stop['stop_candidate_safe']:
                phase = 'TRANSLATE_LEFT' if self.side==1 else 'TRANSLATE_RIGHT'
            clear_ready = stop_confident or (self.side*lateral>=c['target_lateral_m'] and stop['stop_candidate_safe'])
            if phase.startswith('TRANSLATE') and self.side*lateral>=min(c['target_lateral_m'],.60) and clear_ready and clear_geometry:
                phase = 'CLEAR';self.clear_count = 0
            if phase.startswith(('TURN','SWITCH')):
                goal_v = c['turn_speed_m_s'];goal_w = np.clip(3*error,-c['avoid_omega_rad_s'],c['avoid_omega_rad_s'])
            elif phase.startswith('TRANSLATE'):
                goal_v = c['translate_speed_m_s'];goal_w = np.clip(3*error,-.4,.4)
            elif phase=='CLEAR':
                goal_w = np.clip(2*self.angle(self.origin_heading-heading),-.5,.5)
                if fresh:
                    self.clear_count = self.clear_count+1 if stop['stop_candidate_safe'] and active else 0
                if self.clear_count>=c['resume_cycles']:
                    phase='RECOVER'
                    self.cleared_ids.add(self.active_id)
                    self.active_id = None
            elif phase=='RECOVER':
                error = self.angle(self.origin_heading-heading)
                goal_v=0.;goal_w=np.clip(2*error,-.5,.5)
                if abs(error)<=c['heading_tolerance_rad']:
                    phase='RESUME';self.side=0;self.active_id=None
                    self.recovery_completed = True
            elif phase=='REACQUIRE':
                goal_w=np.clip(3*self.angle(self.origin_heading-heading),-c['avoid_omega_rad_s'],c['avoid_omega_rad_s'])
        else:
            departing = bool(risks) and all(r['closing_speed_m_s']<=0 and (r['t_cpa_s']<=0 or r['d_cpa_m']>=radius) for r in risks)
            if fresh:
                self.clear_count = self.clear_count+1 if stop['stop_candidate_safe'] and (departing or self.active_id is None) else 0
            if self.clear_count>=c['resume_cycles']:
                phase='RESUME';self.active_id=None
            elif risks:
                phase='BRAKE' if self.command>.01 else 'STOP'
            if phase in ('CRUISE','RESUME'):
                goal_v=c['nominal_speed_m_s']
        if blocked:
            if self.side and phase!='STOP':
                self.escape_phase = phase
            phase='STOP';goal_v=goal_w=0.
            if self.side and abs(lateral)>=c['target_lateral_m'] and all(r['distance_m']>=c['robot_radius_m']+c['human_radius_m'] for r in risks):
                phase='REACQUIRE'
                goal_w=float(np.clip(3*self.angle(self.origin_heading-heading),-c['avoid_omega_rad_s'],c['avoid_omega_rad_s']))
        elif self.side:
            self.escape_phase = phase
        else:
            self.escape_phase = None
        self.command += float(np.clip(goal_v-self.command,-c['deceleration_m_s2']*dt,c['acceleration_m_s2']*dt))
        if emergency or uncertain:
            self.command=0.
        self.omega += float(np.clip(goal_w-self.omega,-c['angular_acceleration_rad_s2']*dt,c['angular_acceleration_rad_s2']*dt))
        if phase!=self.state:
            self.phase_start_time = now
        self.state = phase
        recorded = all_predictions.get(self.active_id)
        self.diagnostics = {**stop,'left_candidate_min_distance':left[0],'right_candidate_min_distance':right[0],
            'motion_candidates':self.candidate_audit,
            'near_distance_guard':near_distance_guard,'verified_escape_continuation':verified_escape,
            'candidate_visibility':[left[1],right[1]],'candidate_horizon_s':max(left[2],right[2]),
            'active_track_id':self.active_id,'prediction_age':now-recorded[2]['last_observed_time'] if recorded else None,
            'prediction_covariance':recorded[1].tolist() if recorded else None,'track_age':recorded[2]['age'] if recorded else None,
            'selected_side':self.side,'avoid_phase':phase,'heading':heading,
            'decision_current_side':decision_side,'side_switch_count':self.side_switch_count,
            'side_switch_allowed':switch_allowed,'side_switch_reason':switch_reason,
            'side_switch_block_reason':switch_block,
            'stop_reason':('hard_distance' if emergency else 'uncertainty' if uncertain else 'stale_sensor' if stale_sensor else 'depth_invalid' if not depth_ok else switch_block) if blocked else None,
            'current_candidate_min_distance':current_min,'opposite_candidate_min_distance':opposite_min,
            'current_candidate_safe':current_min>=radius if current_min is not None else None,
            'opposite_candidate_safe':opposite_min>=radius if opposite_min is not None else None,
            'phase_start_time':self.phase_start_time,
            'desired_heading':self.origin_heading+self.side*self.escape_angle if self.side else heading,
            'position_variance':float(np.linalg.eigvalsh(recorded[1][:2,:2]).max()) if recorded else None,
            'velocity_variance':float(np.linalg.eigvalsh(recorded[1][2:,2:]).max()) if recorded else None,
            'heading_error':self.angle((self.origin_heading+self.side*self.escape_angle)-heading) if self.side else 0.,
            'forward_displacement':forward,'lateral_displacement':lateral,'v_cmd':self.command,'omega_cmd':self.omega,
            'blocked_uncertainty':uncertain,'blocked_hard_distance':emergency,'blocked_candidates':both_unsafe,
            'fresh_risk_evaluation':fresh,'selected_action':phase}
        return self.command,risks
