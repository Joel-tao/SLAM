import unittest
import numpy as np
from construction_slam.core import transform, compose, relative, icp, ChangeDetector
from construction_slam.simulation import rectangle, scan


class GeometryTests(unittest.TestCase):
    def test_pose_roundtrip(self):
        a=np.array([2.,-1.,.8]); b=np.array([-3.,2.,-.4])
        np.testing.assert_allclose(compose(a,relative(a,b)),b,atol=1e-10)

    def test_icp_recovers_known_transform(self):
        points=np.random.default_rng(10).uniform(-3,3,(500,2))
        pose=np.array([.08,-.06,.02])
        result,ok=icp(points,transform(points,pose),np.zeros(3))
        self.assertTrue(ok)
        np.testing.assert_allclose(result,pose,atol=1e-4)

    def test_reject_no_overlap(self):
        p=np.random.default_rng(1).normal(size=(100,2))
        _,ok=icp(p,p+100,np.zeros(3))
        self.assertFalse(ok)

    def test_lidar_wall_distance(self):
        p=scan(np.zeros(3),rectangle(-2,-2,2,2),np.random.default_rng(1),4)
        np.testing.assert_allclose(np.linalg.norm(p,axis=1),2,atol=.04)

    def test_change_requires_consecutive_observations(self):
        d=ChangeDetector(np.array([[0.,0.],[0.,1.]]))
        new=np.array([[2.,2.],[2.01,2.01]])
        self.assertEqual(len(d.update(new)),0)
        self.assertEqual(len(d.update(new)),0)
        self.assertEqual(len(d.update(new)),1)
        d.update(np.empty((0,2)))
        self.assertEqual(len(d.update(new)),0)


if __name__=='__main__': unittest.main()
