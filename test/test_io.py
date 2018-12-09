from pcl import load_pcd
import unittest

class TestWrapper(unittest.TestCase):
    def test_load_car6(self):
        cloud = load_pcd("data/car6.pcd")
        assert len(cloud) == 10031
