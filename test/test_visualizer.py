import unittest
import numpy as np
import pcl

class TestLoaders(unittest.TestCase):
    def test_load_pointcloud(self):
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

    def test_add_callback(self):
        def key_callback(event):
            print(event.KeyCode, event.KeySym)
        def mouse_callback(event):
            print(event.X, event.Y, event.Type, event.Button)
        def picking_callback(event):
            print(event.Point, event.PointIndex)
        def area_callback(event):
            print(event.PointsIndices)

        cloud = pcl.PointCloud(np.random.rand(100, 3).astype('f4'))
        viewer = pcl.Visualizer()
        viewer.addPointCloud(cloud)
        viewer.registerKeyboardCallback(key_callback)
        viewer.registerMouseCallback(mouse_callback)
        viewer.registerPointPickingCallback(picking_callback)
        viewer.registerAreaPickingCallback(area_callback)

        viewer.spinOnce(time=2000)
        viewer.close()
