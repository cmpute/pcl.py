import unittest
import numpy as np
import pcl

class TestLoaders(unittest.TestCase):
    def test_load_pcd(self):
        cloud1 = pcl.PointCloud(np.random.rand(100, 3).astype('f4'))
        cloud2 = pcl.PointCloud(np.random.rand(100, 3).astype('f4'))
        
        viewer = pcl.Visualizer()
        viewer.addPointCloud(cloud1)
        viewer.addCoordinateSystem()
        viewer.removeCoordinateSystem()
        
        v1 = viewer.createViewPort(0, 0, 0.5, 1)
        v2 = viewer.createViewPort(0.5, 0, 1, 1)
        viewer.addPointCloud(cloud1, viewpoint=v1)
        viewer.addPointCloud(cloud2, viewpoint=v2)
        
        viewer.spinOnce(time=1000)
        viewer.close()
