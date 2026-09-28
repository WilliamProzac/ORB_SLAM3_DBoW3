import unittest
import numpy as np
from analyze import alignment
from trajectory_se3 import evaluate, associate

class AccuracyTests(unittest.TestCase):
    def test_sim3_scale_rotation_translation_and_rpe(self):
        rng=np.random.RandomState(19);p=np.cumsum(rng.normal(size=(80,3)),axis=0)
        R=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
        gp=2.4*p@R.T+np.array([3,-4,7]);t=np.arange(80)*.1
        s,r,shift=alignment(p,gp,True)
        self.assertAlmostEqual(s,2.4,places=12)
        er=np.repeat(np.eye(3)[None],80,axis=0);gr=R@er
        result=evaluate((t,s*p@r.T+shift,r@er),(t,gp,gr))
        self.assertLess(result['ate_rmse_m'],1e-11)
        self.assertLess(result['rpe_translation_rmse_m'],1e-11)
        self.assertLess(result['rpe_rotation_rmse_deg'],1e-4)
    def test_rgbd_never_fits_scale(self):
        p=np.random.RandomState(3).normal(size=(30,3));s,r,t=alignment(p,2*p+4,False)
        self.assertEqual(s,1)
        self.assertGreater(np.mean(np.linalg.norm(s*p@r.T+t-(2*p+4),axis=1)),.5)
    def test_truth_gap_is_not_interpolated(self):
        et=np.arange(100)*.1;gt=np.r_[et[:10],et[-10:]]
        pairs=associate(et,gt,.02)
        self.assertEqual(len(pairs),20)
        self.assertEqual(len({j for i,j in pairs}),20)
if __name__=='__main__':unittest.main()
