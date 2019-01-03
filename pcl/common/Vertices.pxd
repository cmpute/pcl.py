from libcpp.vector cimport vector
from libc.stdint cimport uint32_t
from pcl._boost cimport shared_ptr

cdef extern from "pcl/Vertices.h" namespace "pcl":
    cdef cppclass Vertices:
        Vertices()
        vector[uint32_t] vertices

ctypedef shared_ptr[Vertices] VerticesPtr
ctypedef shared_ptr[const Vertices] VerticesConstPtr
        