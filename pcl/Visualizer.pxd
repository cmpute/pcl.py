from cpython cimport bool as PyBool
from cpython cimport int as PyInt
from pcl.PointCloud cimport PointCloud
from pcl.visualization.pcl_visualizer cimport PCLVisualizer

cdef class Visualizer:
    cdef PCLVisualizer base
    cdef object __weakref__

    cpdef void setFullScreen(self, PyBool mode)
    cpdef void setWindowName(self, str name)
    cpdef void spin(self)
    cpdef void spinOnce(self, PyInt time=*, PyBool force_redraw=*)
    cpdef void addPointCloud(self, PointCloud cloud, str id=*, PyInt viewpoint=*)
