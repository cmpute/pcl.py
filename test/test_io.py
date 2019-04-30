import os.path as osp
import unittest
from pcl import load_pcd

class TestLoaders(unittest.TestCase):
    def test_load_pcd(self):
        cloud = load_pcd(osp.dirname(__file__) + "/data/car6.pcd")
        assert len(cloud) == 10031
        with self.assertRaises(FileNotFoundError):
            load_pcd("some/path/to/nothing.pcd")
