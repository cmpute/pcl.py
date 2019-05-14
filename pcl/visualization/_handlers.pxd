from libcpp cimport bool
from pcl.visualization.point_cloud_color_handlers cimport PointCloudColorHandler_PCLPointCloud2

cdef extern from "python_handlers.hpp" nogil:
    cdef cppclass PointCloudColorHandlerPython(PointCloudColorHandler_PCLPointCloud2):
        PointCloudColorHandlerPython(const PCLPointCloud2ConstPtr &cloud, unsigned char* (*func_handler)(void*), void* func_object)
        void setCapable(bool capable)