from libc.stdint cimport uint8_t, uint32_t
from libcpp.vector cimport vector
from pcl._boost cimport shared_ptr
from pcl.common.PCLHeader cimport PCLHeader

cdef extern from "pcl/PointIndices.h" namespace "pcl":
    cdef cppclass PointIndices:
        PointIndices ()
        PCLHeader header
        vector[int] indices

ctypedef shared_ptr[PointIndices] PointIndicesPtr
ctypedef shared_ptr[const PointIndices] PointIndicesConstPtr
