from libcpp.vector cimport vector
from libc.stdint cimport uint8_t, uint32_t
from pcl._boost cimport shared_ptr
from pcl.common.PCLHeader cimport PCLHeader
from pcl.common.PCLPointField cimport PCLPointField

cdef extern from "pcl/PCLPointCloud2.h" namespace "pcl":
    # Note: structs in C++ can be all considered as cppclass in cython
    cdef cppclass PCLPointCloud2:
        PCLPointCloud2 ()
        PCLHeader header
        uint32_t height
        uint32_t width
        vector[PCLPointField] fields
        uint8_t is_bigendian
        uint32_t point_step
        uint32_t row_step
        vector[uint8_t] data
        uint8_t is_dense

ctypedef shared_ptr[PCLPointCloud2] PCLPointCloud2Ptr
ctypedef shared_ptr[const PCLPointCloud2] PCLPointCloud2ConstPtr
