"""Minimal fixed-right passing. Only estimated tracks, ego pose and map inputs."""
import numpy as np
from social_controller import angle, static_clearance


class ActivePassingSimple:
    def __init__(self, config, waypoints, geometry, radius):
        self.c, self.goal = dict(config), np.asarray(waypoints[-1], float)
        self.geometry, self.radius = geometry, radius
        self.state = self.selected = 'APPROACH'
        self.done = False
        self.logs, self.risks = [], []
        self.origin = self.axis = self.normal = None
        self.target = None
        self.last_plan = -np.inf
        self.command = [0., 0.]
        self.prediction_max_age = 3.0  # Existing active-passing finite age limit.
        self.target_d = -1.4
        self.interrupted = None
        self.pass_committed = False
        self.pass_start_s = None
        self.committed_pass_distance = 3.0
        self.yield_side = None
        self.yield_completed = False

    def local(self, p):
        q = np.asarray(p)[:2] - self.origin
        return np.array([q @ self.axis, q @ self.normal])

    def target_position(self, now):
        target = self.target
        if target is None:
            return None
        # pos is already propagated to stamp, not last_observed_time.
        dt = max(0., min(now, target['observed']+self.prediction_max_age)-target['stamp'])
        return target['pos'] + dt*target['velocity']

    def step(self, now, tracks, measurement_time, position, yaw, v=0., omega=0.):
        p = np.asarray(position, float)[:2]
        if self.origin is None:
            self.origin = p.copy()
            self.axis = self.goal - p
            self.axis /= np.linalg.norm(self.axis)
            self.normal = np.array([-self.axis[1], self.axis[0]])
        if now - self.last_plan < .1 - 1e-7:
            return self.command
        self.last_plan = now
        sd = self.local(p)
        rebound = False
        humans = []
        for t in tracks:
            x = np.asarray(t['state'], float)
            age = now - t['last_observed_time']
            if not np.all(np.isfinite(x)) or age < 0 or age > self.c['stale_age']:
                continue
            if t.get('confidence', 1.) < self.c['min_confidence']:
                continue
            lag = max(0., now - measurement_time) if measurement_time is not None else age
            humans.append(dict(id=t['track_id'], pos=x[:2]+lag*x[2:], velocity=x[2:],
                               stamp=now, observed=t['last_observed_time']))
        if self.target is not None:
            old = self.target
            expected = self.target_position(now)
            matches = [h for h in humans if h['id'] == old['id']]
            if not matches:
                replacements = [h for h in humans if measurement_time is not None
                    and abs(h['observed']-measurement_time) < 1e-6
                    and h['observed'] > old['observed']
                    and np.linalg.norm(h['pos']-expected) <= .8]
                matches = replacements if len(replacements) == 1 else []
            if matches:
                newest = min(matches, key=lambda h: np.linalg.norm(h['pos']-expected))
                if newest['observed'] > old['observed']:
                    rebound = newest['id'] != old['id']
                    self.target = newest
        if self.state == 'APPROACH':
            front = [h for h in humans if 0 < self.local(h['pos'])[0]-sd[0]
                     and abs(self.local(h['pos'])[1]) < self.radius+self.c['human_radius']]
            if front:
                self.target = min(front, key=lambda h: np.linalg.norm(h['pos']-p))
                # Keep RIGHT, but leave footprint clearance to an off-centre person.
                self.target_d = min(-1.4, self.local(self.target['pos'])[1]
                                    -self.radius-self.c['human_radius']-.4)
                self.state = 'OFFSET'
                self.pass_committed = True
            else:
                side = [h for h in humans if 0 < self.local(h['pos'])[0]-sd[0] < 3.
                        and self.radius+self.c['human_radius'] <= abs(self.local(h['pos'])[1]) < 3.]
                if side and not self.yield_completed:
                    self.target = min(side, key=lambda h: np.linalg.norm(h['pos']-p))
                    self.yield_side = np.sign(self.local(self.target['pos'])[1])
                    self.state = 'YIELD'
        target = self.target
        age = now-target['observed'] if target is not None else None
        hp = self.target_position(now)
        hs = self.local(hp) if hp is not None else [None, None]
        distance = float(np.linalg.norm(hp-p)) if hp is not None else None
        merged_ahead = (target is not None and abs(hs[1]) < self.radius+self.c['human_radius']
                        and hs[0]-sd[0] > 3.
                        and target['velocity'] @ self.axis > max(.15, abs(target['velocity'] @ self.normal)))
        if (self.state == 'YIELD' and age is not None and age <= self.c['stale_age']
                and (hs[1]*self.yield_side < -(self.radius+self.c['human_radius']+.5) or merged_ahead)):
            self.state = 'APPROACH'
            self.yield_completed = True
        if self.state == 'EMERGENCY_STOP':
            # Retry the interrupted action; fresh-human and wall guards run below.
            self.state = self.interrupted
        entered_pass = self.state == 'OFFSET' and abs(sd[1]-self.target_d) < .15
        emergency = self.radius+self.c['human_radius']+.20
        if entered_pass:
            self.state = 'PASS'
            self.pass_start_s = float(sd[0])
        committed_distance = float(sd[0]-self.pass_start_s) if self.pass_start_s is not None else 0.
        if self.state == 'PASS' and committed_distance >= self.committed_pass_distance:
            self.state = 'RECOVER'
        if self.state == 'RECOVER' and abs(sd[1]) < .25:
            self.state = 'DONE'
        desired_d = self.target_d if self.state in ('OFFSET', 'PASS') else 0.
        route_heading = np.arctan2(self.axis[1], self.axis[0])
        heading = route_heading + np.clip(np.arctan(1.5*(desired_d-sd[1])), -.65, .65)
        if self.state == 'DONE':
            heading = np.arctan2(*(self.goal-p)[::-1])
        err = angle(heading-yaw)
        speed = .80 if self.state == 'PASS' else .35
        cmd = [speed*max(.65, float(np.cos(err))), float(np.clip(1.8*err, -.65, .65))]
        if self.state == 'DONE':
            cmd[0] = min(speed, float(np.linalg.norm(self.goal-p)))*max(0., float(np.cos(err)))
        if self.state == 'YIELD':
            cmd = [0., 0.]
        reason = None
        self.risks = [dict(track_id=h['id'], distance=float(np.linalg.norm(h['pos']-p))) for h in humans]
        distance_age_limit = self.c['stale_age']
        if any(h['distance'] < emergency for h in self.risks) or (age is not None and age <= distance_age_limit and distance < emergency):
            reason = 'HUMAN_EMERGENCY_DISTANCE'
        # Short physical motion guard, not a full-maneuver planner.
        q, hd = p.copy(), yaw
        near = [q.copy()]
        for _ in range(10):
            q += max(v, cmd[0])*np.array([np.cos(hd), np.sin(hd)])*.1
            hd += cmd[1]*.1
            near.append(q.copy())
        if static_clearance(np.asarray(near), self.geometry, self.radius).min() < self.c['static_extra_margin']:
            reason = 'STATIC_COLLISION_RISK'
        if reason is not None or self.state == 'EMERGENCY_STOP':
            if self.state != 'EMERGENCY_STOP':
                self.interrupted = self.state
            self.state = 'EMERGENCY_STOP'
            cmd = [0., 0.]
        if self.state in ('APPROACH','DONE') and np.linalg.norm(self.goal-p) < .25:
            self.state = 'DONE'
            self.done = True
            cmd = [0., 0.]
        self.selected = self.state
        self.command = cmd
        self.logs.append(dict(time=float(now), state=self.state, robot_s=float(sd[0]), robot_d=float(sd[1]),
                              human_est_s=hs[0], human_est_d=hs[1], track_age=age, distance=distance,
                              target_id=target['id'] if target is not None else None,
                              fresh_track_rebind=rebound,
                              target_cv_dt=max(0., min(now, target['observed']+self.prediction_max_age)-target['stamp']) if target is not None else 0.,
                              pass_committed=self.pass_committed, pass_start_s=self.pass_start_s,
                              committed_distance=committed_distance,
                              v_cmd=cmd[0], omega_cmd=cmd[1], stop_reason=reason))
        return cmd
