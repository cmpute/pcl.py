from libc.stdint cimport uint32_t, uint64_t
from libcpp.string cimport string

cdef extern from "pcl/PCLHeader.h" namespace "pcl":
    cdef cppclass PCLHeader:
        PCLHeader ()
        uint32_t seq
        uint64_t stamp
        string frame_id
