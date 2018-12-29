from pcl._boost.smart_ptr cimport shared_ptr, make_shared
from pcl.common.PCLPointCloud2 cimport PCLPointCloud2, PCLPointCloud2ConstPtr
from pcl.visualization.point_cloud_geometry_handlers cimport PointCloudGeometryHandler_PCLPointCloud2, PointCloudGeometryHandlerXYZ_PCLPointCloud2
from pcl.visualization.point_cloud_color_handlers cimport PointCloudColorHandler_PCLPointCloud2, PointCloudColorHandlerCustom_PCLPointCloud2

cdef class Visualizer:
    cdef PCLVisualizer* ptr(self):
        return self._ptr.get()
    def __init__(self, str name="", bool create_interactor=True):
        self._ptr = make_shared[PCLVisualizer](<bytes>(name.encode('ascii')), create_interactor)

    cpdef void setFullScreen(self, bool mode):
        self.ptr().setFullScreen(mode)
    cpdef void setWindowName(self, str name):
        self.ptr().setWindowName(name.encode('ascii'))
    cpdef void setBackgroundColor(self, double r, double g, double b, int viewpoint=0):
        self.ptr().setBackgroundColor(r, g, b, viewpoint)
    cpdef void addCoordinateSystem(self, double scale=1, float x=0, float y=0, float z=0, int viewpoint=0):
        # TODO: support Affine3f
        self.ptr().addCoordinateSystem(scale, x, y, z, viewpoint)

    cpdef void spin(self):
        self.ptr().spin()
    cpdef void spinOnce(self, int time=1, bool force_redraw=False):
        self.ptr().spinOnce(time, force_redraw)
    cpdef bool wasStopped(self):
        return self.ptr().wasStopped()
    cpdef void resetStoppedFlag(self):
        self.ptr().resetStoppedFlag()
    cpdef void close(self):
        self.ptr().close()
    cpdef int createViewPort(self, double xmin, double ymin, double xmax, double ymax):
        cdef int retval = 0
        self.ptr().createViewPort(xmin, ymin, xmax, ymax, retval)
        return retval
    cpdef void createViewPortCamera(self, int viewpoint):
        self.ptr().createViewPortCamera(viewpoint)

    cpdef void addPointCloud(self, PointCloud cloud, str id="cloud", int viewpoint=0):
        cdef shared_ptr[PointCloudGeometryHandlerXYZ_PCLPointCloud2] geometry_handler = make_shared[PointCloudGeometryHandlerXYZ_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr)
        cdef shared_ptr[PointCloudColorHandlerCustom_PCLPointCloud2] color_handler = make_shared[PointCloudColorHandlerCustom_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr, 255, 255, 255)
        cdef bool retval = self.ptr().addPointCloud(<PCLPointCloud2ConstPtr>cloud._ptr,
            <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>geometry_handler,
            <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>color_handler,
            cloud._origin, cloud._orientation, id.encode('ascii'), viewpoint)
        if retval != True:
            raise RuntimeError("Function addPointCloud returned false, please check stderr output!")
