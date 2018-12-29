from libcpp cimport bool
from pcl._boost.smart_ptr cimport shared_ptr
from pcl.PointCloud cimport PointCloud
from pcl.visualization.pcl_visualizer cimport PCLVisualizer

cdef class Visualizer:
    cdef shared_ptr[PCLVisualizer] _ptr
    cdef object __weakref__
    cdef PCLVisualizer* ptr(self)

    cpdef void setFullScreen(self, bool mode)
    cpdef void setWindowName(self, str name)
    cpdef void setBackgroundColor(self, double r, double g, double b, int viewpoint=*)
    cpdef void addCoordinateSystem(self, double scale=*, float x=*, float y=*, float z=*, int viewpoint=*)

    cpdef void spin(self)
    cpdef void spinOnce(self, int time=*, bool force_redraw=*)
    cpdef bool wasStopped(self)
    cpdef void resetStoppedFlag(self)
    cpdef void close(self)
    cpdef int createViewPort(self, double xmin, double ymin, double xmax, double ymax)
    cpdef void createViewPortCamera(self, int viewport)

    cpdef void addPointCloud(self, PointCloud cloud, str id=*, int viewpoint=*)
