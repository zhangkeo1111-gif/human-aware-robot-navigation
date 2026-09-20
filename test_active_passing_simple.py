import unittest
import numpy as np
from active_passing_simple import ActivePassingSimple


class SimpleTest(unittest.TestCase):
    def controller(self):
        return ActivePassingSimple(dict(stale_age=.8, min_confidence=.25,
            human_radius=.3, static_extra_margin=.03), [[9, 0]], [], .603)

    def track(self, stamp):
        return [dict(track_id=1, state=[4.5, 0, 0, 0], last_observed_time=stamp, confidence=.9)]

    def test_phases(self):
        c = self.controller()
        for t, p, phase in [(0,[0,0],'OFFSET'),(.1,[3,-1.3],'PASS'),
                             (.2,[6.1,-1.3],'RECOVER'),(.3,[7,-.2],'DONE')]:
            c.step(t,self.track(t),t,p,0)
            self.assertEqual(c.state,phase)
        c.step(.4,self.track(.4),.4,[8.9,0],0)
        self.assertTrue(c.done)

    def test_stale_does_not_stop_committed_offset(self):
        c=self.controller()
        c.step(0,self.track(0),0,[0,0],0)
        c.step(3.1,[],0,[2,-1.0],0)
        self.assertEqual(c.state,'OFFSET')
        self.assertGreater(c.command[0],0)

    def test_human_emergency(self):
        c=self.controller()
        c.step(0,self.track(0),0,[0,0],0)
        c.step(.1,self.track(.1),.1,[4,0],0)
        self.assertEqual(c.state,'EMERGENCY_STOP')

    def test_wall_stop(self):
        c=self.controller()
        c.geometry=[dict(polygon_xy=[[.8,-2],[1,-2],[1,2],[.8,2]])]
        c.step(0,[],0,[0,0],0)
        self.assertEqual(c.logs[-1]['stop_reason'],'STATIC_COLLISION_RISK')

    def test_yield_then_resume_without_reentering(self):
        c=self.controller()
        c.step(0,[],0,[0,0],0)
        t=self.track(.1);t[0]['state'][:2]=[2.5,-2]
        c.step(.1,t,.1,[0,0],0)
        self.assertEqual(c.state,'YIELD')
        t=self.track(.2);t[0]['state'][:2]=[2.5,1.6]
        c.step(.2,t,.2,[0,0],0)
        self.assertEqual(c.state,'APPROACH')
        c.step(.3,t,.2,[.01,0],0)
        self.assertEqual(c.state,'APPROACH')

    def test_direct_route_goal(self):
        c=self.controller()
        c.step(0,[],0,[0,0],0)
        c.step(1,[],1,[8.9,0],0)
        self.assertTrue(c.done)

    def test_unique_replacement_near_prediction(self):
        c=self.controller()
        t=self.track(0)
        t[0]['state'][2]=-.3
        c.step(0,t,0,[0,0],0)
        replacement=self.track(2.9)
        replacement[0]['track_id']=2
        replacement[0]['state'][0]=3.63
        c.step(2.9,replacement,2.9,[1,-.7],0)
        self.assertEqual(c.target['id'],2)
        self.assertEqual(c.logs[-1]['track_age'],0)

    def test_bounded_cv_uses_cache_stamp_without_refresh(self):
        c=self.controller()
        t=self.track(0)
        t[0]['state'][2]=.2
        c.step(.2,t,0,[0,0],0)
        c.step(2,[],0,[1,-.5],0)
        self.assertAlmostEqual(c.logs[-1]['human_est_s'],4.9)
        self.assertEqual(c.target['observed'],0)
        c.step(3.1,[],0,[1,-.5],0)
        self.assertAlmostEqual(c.logs[-1]['human_est_s'],5.1)
        self.assertIsNone(c.logs[-1]['stop_reason'])

    def test_ambiguous_rebind_rejected(self):
        c=self.controller()
        c.step(0,self.track(0),0,[0,0],0)
        a=self.track(.2)[0]; a['track_id']=2
        b=dict(a); b['track_id']=3
        c.step(.2,[a,b],.2,[0,0],0)
        self.assertEqual(c.target['id'],1)

    def test_prediction_does_not_refresh_last_measurement(self):
        c=self.controller()
        c.step(0,self.track(0),0,[0,0],0)
        t=self.track(0)
        t[0]['state'][0]=4.8
        c.step(.5,t,.5,[.1,-.1],0)
        self.assertEqual(c.target['pos'][0],4.5)
        self.assertEqual(c.logs[-1]['track_age'],.5)

    def test_observed_distant_front_person_triggers_offset(self):
        c=self.controller()
        t=self.track(0)
        t[0]['state'][0]=7
        c.step(0,t,0,[0,0],0)
        self.assertEqual(c.state,'OFFSET')

    def committed(self):
        c=self.controller()
        c.step(0,self.track(0),0,[0,0],0)
        c.step(2.9,[],0,[2,-1.3],0)
        self.assertEqual(c.state,'PASS')
        c.step(3.1,[],0,[2.1,-1.35],0)
        self.assertEqual(c.state,'PASS')
        return c

    def test_committed_bounded_completion_without_new_measurement(self):
        c=self.committed()
        c.step(6,[],0,[5.05,-1.4],0)
        self.assertEqual(c.state,'RECOVER')

    def test_pass_does_not_recover_early(self):
        c=self.committed()
        c.step(6,[],0,[4.9,-1.4],0)
        self.assertEqual(c.state,'PASS')

    def test_committed_fresh_collision_guard(self):
        c=self.committed()
        t=self.track(3.2)
        t[0]['state'][:2]=[2.3,-1.3]
        c.step(3.2,t,3.2,[2.2,-1.4],0)
        self.assertEqual(c.state,'EMERGENCY_STOP')
        self.assertEqual(c.logs[-1]['stop_reason'],'HUMAN_EMERGENCY_DISTANCE')


if __name__ == '__main__':
    unittest.main()
