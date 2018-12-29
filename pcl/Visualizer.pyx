from cython.operator cimport address as addr
from pcl._boost.smart_ptr cimport shared_ptr, make_shared
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2
from pcl.visualization.point_cloud_geometry_handlers cimport PointCloudGeometryHandler_PCLPointCloud2, PointCloudGeometryHandlerXYZ_PCLPointCloud2
from pcl.visualization.point_cloud_color_handlers cimport PointCloudColorHandler_PCLPointCloud2, PointCloudColorHandlerCustom_PCLPointCloud2

cdef class Visualizer:
    cpdef void setFullScreen(self, PyBool mode):
        self.base.setFullScreen(mode)
    cpdef void setWindowName(self, str name):
        self.base.setWindowName(name.decode('ascii'))
    cpdef void spin(self):
        self.base.spin()
    cpdef void spinOnce(self, PyInt time=1, PyBool force_redraw=False):
        self.base.spinOnce(time, force_redraw)
    cpdef void addPointCloud(self, PointCloud cloud, str id="cloud", PyInt viewpoint=0):
        cdef shared_ptr[PointCloudGeometryHandlerXYZ_PCLPointCloud2] geometry_handler = make_shared[PointCloudGeometryHandlerXYZ_PCLPointCloud2](<shared_ptr[const PCLPointCloud2]>cloud._ptr)
        cdef shared_ptr[PointCloudColorHandlerCustom_PCLPointCloud2] color_handler = make_shared[PointCloudColorHandlerCustom_PCLPointCloud2](<shared_ptr[const PCLPointCloud2]>cloud._ptr, 255, 255, 255)
        self.base.addPointCloud(<shared_ptr[const PCLPointCloud2]>cloud._ptr,
                                <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>geometry_handler,
                                <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>color_handler,
                                cloud._origin, cloud._orientation, id.decode('ascii'), viewpoint)
