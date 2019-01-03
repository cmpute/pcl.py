from libc.stdint cimport uint8_t, uint32_t
from libcpp.string cimport string
from pcl._boost cimport shared_ptr

cdef extern from "pcl/PCLPointField.h" namespace "pcl":
    cdef cppclass PCLPointField:
        PCLPointField ()
        string name
        uint32_t offset
        uint8_t datatype
        uint32_t count

ctypedef shared_ptr[PCLPointField] PCLPointFieldPtr
ctypedef shared_ptr[const PCLPointField] PCLPointFieldConstPtr
