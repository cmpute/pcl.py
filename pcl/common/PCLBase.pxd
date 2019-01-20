from libcpp.vector cimport vector
from pcl._boost cimport shared_ptr, make_shared
from pcl.common.pcl_base cimport PCLBase_PCLPointCloud2 as cPCLBase
from pcl.PointCloud cimport PointCloud

cdef class PCLBase:
    cdef shared_ptr[cPCLBase] _ptr # use pointer for inheritance
    cdef PointCloud _input
