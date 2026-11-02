import math
import unittest
from delayed_bearing_filter import Alignment, ClockMapping, Pose, DelayedBearingFilter, observe_box, pose_at


class DelayedTests(unittest.TestCase):
    def setUp(self):
        self.a=Alignment('mono-v1:test','synthetic-test-only',True,1)

    def make(self):
        f=DelayedBearingFilter(self.a);f.reset('pink');return f

    def test_delayed_out_of_order_matches_chronological_posterior(self):
        a,b=self.make(),self.make()
        for t in [1000,1100,1200,1300]:a.observe(t,1500,(t-1000)*.01,0,'pink',self.a.clock_id)
        for t in [1000,1300,1100,1200]:b.observe(t,1500,(t-1000)*.01,0,'pink',self.a.clock_id)
        self.assertEqual(a.predict(1500),b.predict(1500))

    def test_query_is_not_double_integration(self):
        f=self.make()
        for t in range(1000,1400,100):f.observe(t,t,(t-1000)*.01,0,'pink',self.a.clock_id)
        first=f.predict(1500)
        for _ in range(100):self.assertEqual(first,f.predict(1500))
        self.assertEqual(first['total_propagation_ms'],350)
        self.assertGreater(first['yaw_variance_deg2'],0)

    def test_stationary_world_target_turning_camera(self):
        f=self.make();poses=[]
        for i in range(10):
            t=1000+i*100;yaw=i*2
            poses.append(Pose(t,t,yaw,0,self.a.clock_id))
            observe_box(f,dict(timestamp_kind='mapped_measurement',capture_ms=t,received_ms=t+100,
                u=320+500*math.tan(math.radians(10-yaw)),v=240,cx=320,cy=240,fx=500,fy=500,
                target_id='pink',clock_id=self.a.clock_id),poses)
        self.assertAlmostEqual(f.predict(2000)['world_yaw_deg'],10,places=6)

    def test_unverified_clocks_missing_pose_and_future_pose_blocked(self):
        with self.assertRaises(ValueError):DelayedBearingFilter(Alignment('mono-v1:test','',False,0)).predict(1)
        with self.assertRaises(ValueError):pose_at([Pose(1000,1300,0,0,self.a.clock_id)],1000,1100,self.a)
        f=self.make()
        with self.assertRaises(ValueError):f.observe(1000,1100,0,0,'pink','mono-v1:other')
        with self.assertRaises(ValueError):f.observe(1200,1100,0,0,'pink',self.a.clock_id)

    def test_duplicates_target_switch_and_stale(self):
        f=self.make();f.observe(1000,1100,0,0,'pink',self.a.clock_id)
        with self.assertRaises(ValueError):f.observe(1000,1200,0,0,'pink',self.a.clock_id)
        with self.assertRaises(ValueError):f.observe(1100,1200,0,0,'yellow',self.a.clock_id)
        with self.assertRaises(ValueError):f.predict(1600)
        f.reset('yellow');f.observe(1100,1200,90,0,'yellow',self.a.clock_id)
        self.assertEqual(f.predict(1200)['world_yaw_deg'],90)

    def test_outlier_does_not_refresh_age(self):
        f=self.make()
        for t in range(1000,1500,100):f.observe(t,t,0,0,'pink',self.a.clock_id)
        f.observe(1900,1900,150,0,'pink',self.a.clock_id)
        self.assertEqual(f.predict(1900)['rejected_in_window'],1)
        with self.assertRaises(ValueError):f.predict(1901)

    def test_window_bounded_and_very_late_data_rejected(self):
        f=self.make()
        for i in range(100):f.observe(1000+i*10,1000+i*10,0,0,'pink',self.a.clock_id)
        self.assertEqual(len(f.measurements),64)
        with self.assertRaises(ValueError):f.observe(1100,2000,0,0,'pink',self.a.clock_id)

    def test_wrap_interpolation_and_clock_mapping(self):
        poses=[Pose(1000,1000,179,0,self.a.clock_id),Pose(1100,1100,-179,0,self.a.clock_id)]
        self.assertEqual(abs(pose_at(poses,1050,1100,self.a)[0]),180)
        mapping=ClockMapping(self.a,'sdk-test',1,100,0,2000)
        self.assertEqual(mapping.map(1000,'sdk-test',1200),1100)
        with self.assertRaises(ValueError):mapping.map(1000,'other',1200)
        with self.assertRaises(ValueError):mapping.map(1500,'sdk-test',1200)


if __name__=='__main__':unittest.main()
