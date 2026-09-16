"""Small deterministic mechanism checks, not substitutes for Isaac gates."""
import json
import unittest
from pathlib import Path
import numpy as np
from active_passing_controller import ActivePassingController


class PassingChecks(unittest.TestCase):
    def make(self,goal=(9,0),geometry=()):
        return ActivePassingController(json.loads(Path('config.yaml').read_text())['social_navigation'],[goal],geometry,.6027795979748837,
            dict(linear_accel_m_s2=.5,linear_decel_m_s2=.8,angular_accel_rad_s2=1.2))

    def track(self,p=(4.5,0),tid=1,t=0):
        return dict(track_id=tid,state=[*p,0.,0.],last_observed_time=t,confidence=1.)

    def test_corridor_hard_rejects_narrow_offsets(self):
        c=self.make();c.step(0,[self.track()],0,[0,0],0)
        self.assertEqual(abs(c.offset),1.)
        self.assertEqual([r['valid'] for r in c.logs[-1]['candidates']],[False,False,True,False,False,True])

    def test_rotated_route(self):
        a=self.make();b=self.make((0,9))
        ca=a.step(0,[self.track()],0,[0,0],0)
        cb=b.step(0,[self.track((0,4.5))],0,[0,0],np.pi/2)
        np.testing.assert_allclose(ca,cb,atol=1e-8);self.assertEqual(a.offset,b.offset)

    def test_fresh_id_rebind_does_not_switch_side(self):
        c=self.make();c.step(0,[self.track()],0,[0,0],0);offset=c.offset
        c.step(.1,[self.track(tid=2,t=.1)],.1,[.01,0],0)
        self.assertEqual(c.target,2);self.assertEqual(c.offset,offset)
        self.assertNotEqual(c.phase,'EMERGENCY_STOP')

    def test_stale_stops(self):
        c=self.make();c.step(0,[self.track()],0,[0,0],0)
        command=c.step(1,[],None,[.1,0],0)
        self.assertEqual(command,[0.,0.]);self.assertEqual(c.phase,'EMERGENCY_STOP')

    def test_dynamics_acceleration(self):
        c=self.make();path=np.array([[x,0] for x in np.arange(0,5,.05)])
        xy,commands=c.path_rollout(np.zeros(2),0,0,0,[path])
        self.assertLessEqual(np.max(np.diff(np.r_[0,commands[0,:,0]])),.05+1e-9)
        self.assertLessEqual(np.max(np.abs(np.diff(np.r_[0,commands[0,:,1]]))),.12+1e-9)
        self.assertEqual(xy.shape,(1,61,2))

if __name__=='__main__':unittest.main()
