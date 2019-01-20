import unittest
import os.path as osp
import pcl
import pcl.sample_consensus as ps

class TestSac(unittest.TestCase):
    def test_ransac_plane(self):
        cloud = pcl.load_pcd(osp.dirname(__file__) + "/data/sac_plane_test.pcd")
        model = ps.SampleConsensusModelPlane(cloud)
        sac = ps.RandomSampleConsensus(model, 0.03)
        sac.computeModel()
        coeff = sac.ModelCoefficients
