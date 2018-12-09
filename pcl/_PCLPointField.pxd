from libc.stdint cimport uint8_t, uint32_t
from libcpp.string cimport string

cdef extern from "pcl/PCLPointField.h" namespace "pcl":
    cdef struct PCLPointField:
        PCLPointField ()
        string name
        uint32_t offset
        uint8_t datatype
        uint32_t count
