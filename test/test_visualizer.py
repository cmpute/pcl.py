import unittest
import numpy as np
import pcl

class TestVisualizer(unittest.TestCase):
    def test_add_pointcloud(self):
        cloud1 = pcl.PointCloud(np.random.rand(100, 4).astype('f4'))
        cloud2 = pcl.PointCloud(np.random.rand(100, 4).astype('f4'))
        
        viewer = pcl.Visualizer()
        viewer.addPointCloud(cloud1)
        viewer.addCoordinateSystem()
        viewer.removeCoordinateSystem()
        
        v1 = viewer.createViewPort(0, 0, 0.5, 1)
        v2 = viewer.createViewPort(0.5, 0, 1, 1)
        viewer.addPointCloud(cloud1, viewport=v1, id="cloud1")
        viewer.addPointCloud(cloud2, viewport=v2, id="cloud2")
        
        viewer.spinOnce(time=1000)
        viewer.close()

    def test_add_callback(self):
        def key_callback(event):
            print(event)
        def mouse_callback(event):
            print(event)
        def picking_callback(event):
            print(event)
        def area_callback(event):
            print(event)

        cloud = pcl.PointCloud(np.random.rand(100, 4).astype('f4'))
        viewer = pcl.Visualizer("Testing mouse and key events")
        viewer.addPointCloud(cloud)
        viewer.registerKeyboardCallback(key_callback)
        viewer.registerMouseCallback(mouse_callback)
        viewer.registerPointPickingCallback(picking_callback)
        viewer.registerAreaPickingCallback(area_callback)

        viewer.spinOnce(time=2000)
        viewer.close()

    def test_add_shape(self):
        cloud = pcl.PointCloud(np.random.rand(100, 4).astype('f4'))
        
        viewer = pcl.Visualizer()
        viewer.addPointCloud(cloud)
        
        viewer.addLine([-1,-1,-1], [1,1,1])
        viewer.addArrow([1,-1,-1], [-1,1,1], line_color=[0.7,0,0], text_color=[0,3,0,0])
        viewer.addSphere([0,0,0], 1)

        viewer.addText("Text", -1, 1, fontsize=100)
        viewer.addText3D("Text3D", [-1,1,-1], text_scale=2)
        
        viewer.spinOnce(time=500)
        viewer.updateSphere([0,0,0], 2)
        viewer.spinOnce(time=500)
        
        viewer.close()

    def test_color_handlers(self):
        viewer = pcl.Visualizer()

        cloud = pcl.PointCloud(np.random.rand(100, 4).astype('f4'))
        viewer.addPointCloud(cloud, field="y", id="cloud1")

        cloud = pcl.PointCloud(np.random.rand(100, 4).astype('f4'))
        viewer.addPointCloud(cloud, color=[0.8,0.2,0], id="cloud2")

        cloud = pcl.PointCloud(np.random.rand(100, 4).astype('f4'))
        viewer.addPointCloud(cloud, color_handler=lambda: np.random.rand(100, 4)*255, id="cloud3")

        cloud = np.random.rand(100, 4).astype('f4')
        cloud[:, 3] *= 20 # create label fields
        cloud = pcl.create_xyzl(cloud)
        viewer.addPointCloud(cloud, field="label", id="cloud4")

        cloud = np.random.rand(100, 6).astype('f4')
        cloud[:, 3:6] *= 256 # create rgb fields
        cloud = pcl.create_xyzrgb(cloud)
        viewer.addPointCloud(cloud, field="rgb", id="cloud5")

        cloud = np.random.rand(100, 7).astype('f4')
        cloud[:, 3:7] *= 256 # create rgb fields
        cloud = pcl.create_xyzrgba(cloud)
        viewer.addPointCloud(cloud, field="rgba", id="cloud6")

        viewer.spinOnce(time=2000)
        viewer.close()

    def test_add_normal(self):
        viewer = pcl.Visualizer()

        cloud_data = np.random.rand(100, 6)
        cloud_data[:, 3:] = np.clip(cloud_data[:, 3:] * 128 + 128, 0, 256)
        cloud = pcl.create_xyzrgb(cloud_data)
        normals = pcl.create_normal(np.random.rand(100, 4))
        viewer.addPointCloud(cloud)
        viewer.addPointCloudNormals(cloud, normals, level=2, scale=0.1, id="cloudn")

        viewer.spinOnce(time=2000)
        viewer.close()

if __name__ == "__main__":
    TestVisualizer().test_add_callback()
