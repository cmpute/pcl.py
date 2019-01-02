import pcl
import numpy as np
import unittest

class TestNumpyInitialize(unittest.TestCase):
    def test_normal_init(self):
        cloud_array = np.array([[1,2,3],[2,3,4]], dtype='f4')
        cloud = pcl.PointCloud(cloud_array)
        assert len(cloud) == 2
        assert np.all(cloud.xyz == cloud_array)
        assert len(cloud.fields) == 3
        assert cloud.names == ['x', 'y', 'z']

    def test_list_init(self):
        cloud = pcl.PointCloud([(1,2,3),(2,3,4)], 'xyz')
        assert len(cloud) == 2
        assert cloud.xyz[1,1] == 3
        assert cloud.names == ['x', 'y', 'z']

    def test_copy_init(self):
        cloud_array = np.array([[1,2,3],[2,3,4]], dtype='f4')
        cloud = pcl.PointCloud(cloud_array)
        copy = pcl.PointCloud(cloud)
        assert np.all(copy.xyz == cloud_array)
        cloud.xyz[0,0] = 0
        assert np.any(cloud.xyz != cloud_array)
        assert np.all(copy.xyz == cloud_array)

    def test_point_type(self):
        cloud = pcl.PointCloud([(1,2,3,255),(2,3,4,255)], 'xyzrgb')
        assert len(cloud) == 2
        assert cloud.names == ['x', 'y', 'z', 'rgb']

    def test_origin(self):
        cloud = pcl.PointCloud([(1,2,3),(2,3,4)])
        assert np.all(cloud.sensor_origin == np.zeros(3))
        cloud.sensor_origin = [1, 2, 3]
        assert np.all(cloud.sensor_origin == np.array([1,2,3]))

    def test_orientation(self):
        cloud = pcl.PointCloud([(1,2,3),(2,3,4)])
        assert np.all(cloud.sensor_orientation == np.array([0,0,0,1]))
        cloud.sensor_orientation = [1, 2, 3, 1]
        assert np.all(cloud.sensor_orientation == np.array([1,2,3,1]))
