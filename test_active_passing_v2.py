import json
import unittest
from pathlib import Path
import numpy as np
from active_passing_v2 import ActivePassingControllerV2

class V2Checks(unittest.TestCase):
    def make(self,goal=(9,0),fov=106.26):
        c=ActivePassingControllerV2(json.loads(Path('config.yaml').read_text())['social_navigation'],[goal],[],.6027795979748837,
            dict(linear_accel_m_s2=.5,linear_decel_m_s2=.8,angular_accel_rad_s2=1.2))
        c.set_camera(dict(cameraAperture=[18*np.tan(np.radians(fov/2)),13.5],cameraFocalLength=9,
            renderProductResolution=[640,360],cameraApertureOffset=[0,0]))
        return c

    def test_camera_derived(self):
        c=self.make(fov=120)
        self.assertAlmostEqual(c.camera_model['hfov_deg'],120)
        self.assertAlmostEqual(np.degrees(c.camera_model['hard_limit']),58)
        self.assertAlmostEqual(c.camera_model['fx'],640/(2*np.tan(np.radians(60))))

    def test_geometry_and_candidate_count(self):
        c=self.make();c.step(0,[dict(track_id=1,state=[4.5,0,0,0],last_observed_time=0,confidence=1)],0,[0,0],0)
        self.assertEqual(c.candidate_history[0]['candidate_count'],18)
        g=c.candidate_history[0]['diagnostics']
        self.assertLess(g[2]['maximum_heading_deviation'],g[0]['maximum_heading_deviation'])
        self.assertTrue(all(x['final_lateral_error']<1e-8 for x in g))

    def test_no_recover_on_stale_behind(self):
        c=self.make();c.step(0,[dict(track_id=1,state=[4.5,0,0,0],last_observed_time=0,confidence=1)],0,[0,0],0)
        self.assertFalse(c.behind_confirmed)
        c.step(1,[],None,[.1,0],0)
        self.assertEqual(c.phase,'EMERGENCY_STOP')
        self.assertFalse(c.behind_confirmed)

    def test_rotation(self):
        a=self.make();b=self.make((0,9))
        a.step(0,[dict(track_id=1,state=[4.5,0,0,0],last_observed_time=0,confidence=1)],0,[0,0],0)
        b.step(0,[dict(track_id=1,state=[0,4.5,0,0],last_observed_time=0,confidence=1)],0,[0,0],np.pi/2)
        self.assertEqual(a.offset,b.offset)
        np.testing.assert_allclose([r['max_abs_bearing_deg'] for r in a.candidate_history[0]['diagnostics']],
            [r['max_abs_bearing_deg'] for r in b.candidate_history[0]['diagnostics']],atol=1e-7)

if __name__=='__main__':unittest.main()
