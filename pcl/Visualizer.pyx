from pcl._boost cimport shared_ptr, make_shared
from pcl._eigen cimport Affine3f, toAffine3f
from pcl.common cimport _ensure_true
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
    cpdef void setWindowBorders(self, bool mode):
        self.ptr().setWindowBorders(mode)
    cpdef void setBackgroundColor(self, double r, double g, double b, int viewpoint=0):
        self.ptr().setBackgroundColor(r, g, b, viewpoint)

    cpdef void spin(self):
        self.ptr().spin()
    cpdef void spinOnce(self, int time=1, bool force_redraw=False):
        self.ptr().spinOnce(time, force_redraw)

    cpdef void addCoordinateSystem(self, double scale=1, float x=0, float y=0, float z=0, np.ndarray t=None, int viewpoint=0):
        cdef Affine3f ct
        if t is not None:
            ct = toAffine3f(t)
            self.ptr().addCoordinateSystem(scale, ct, viewpoint)
        else:
            self.ptr().addCoordinateSystem(scale, x, y, z, viewpoint)
    cpdef void removeCoordinateSystem(self, int viewpoint=0):
        self.ptr().removeCoordinateSystem(viewpoint)

    cpdef void removePointCloud(self, str id="cloud", int viewpoint=0):
        self.ptr().removePointCloud(id.encode('ascii'), viewpoint)
    cpdef void removePolygonMesh(self, str id="polygon", int viewpoint=0):
        self.ptr().removePolygonMesh(id.encode('ascii'), viewpoint)
    cpdef void removeShape(self, str id="cloud", int viewpoint=0):
        self.ptr().removeShape(id.encode('ascii'), viewpoint)
    cpdef void removeText3D(self, str id="cloud", int viewpoint=0):
        self.ptr().removeText3D(id.encode('ascii'), viewpoint)
    cpdef void removeAllPointClouds(self, int viewpoint=0):
        self.ptr().removeAllPointClouds(viewpoint)
    cpdef void removeAllShapes(self, int viewpoint=0):
        self.ptr().removeAllShapes(viewpoint)

    cpdef void addText(self, str text, int xpos, int ypos, int fontsize=10, double r=1, double g=1, double b=1, str id="", int viewpoint=0):
        self.ptr().addText(text, xpos, ypos, fontsize, r, g, b, id.encode('ascii'), viewpoint)
    cpdef void updateText(self, str text, int xpos, int ypos, int fontsize=10, double r=1, double g=1, double b=1, str id=""):
        self.ptr().updateText(text, xpos, ypos, fontsize, r, g, b, id.encode('ascii'))
    
    cpdef void updateShapePose(self, str id, np.ndarray pose):
        self.ptr().updateShapePose(id, toAffine3f(pose))
    cpdef void updatePointCloudPose(self, str id, np.ndarray pose):
        self.ptr().updateShapePose(id, toAffine3f(pose))

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
        # TODO: specialize PointXYZ, PointXYZRGB, PointXYZRGBA
        cdef shared_ptr[PointCloudGeometryHandlerXYZ_PCLPointCloud2] geometry_handler = make_shared[PointCloudGeometryHandlerXYZ_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr)
        cdef shared_ptr[PointCloudColorHandlerCustom_PCLPointCloud2] color_handler = make_shared[PointCloudColorHandlerCustom_PCLPointCloud2](<PCLPointCloud2ConstPtr>cloud._ptr, 255, 255, 255)
        _ensure_true(self.ptr().addPointCloud(<PCLPointCloud2ConstPtr>cloud._ptr,
            <shared_ptr[const PointCloudGeometryHandler_PCLPointCloud2]>geometry_handler,
            <shared_ptr[const PointCloudColorHandler_PCLPointCloud2]>color_handler,
            cloud._origin, cloud._orientation, id.encode('ascii'), viewpoint),
            "addPointCloud")
