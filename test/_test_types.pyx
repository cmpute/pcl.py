from pcl cimport *
from cpython cimport bool

cdef extern from "_test_types_src.h":
    cdef int test(PointCloud[PointXYZI]& cloud)

cpdef bool test_PointXYZI_size():
    cdef PointXYZI pt = PointXYZI()
    cdef PointCloud[PointXYZI] p = PointCloud[PointXYZI]()
    p.push_back(pt)
    p.push_back(pt)
    return p.size() + test(p) == 6
