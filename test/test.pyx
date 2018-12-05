from pcl cimport PointCloud, PointXYZI

cdef extern from "test.h":
    cdef int test(PointCloud[PointXYZI]& cloud)

cpdef int ptest():
    cdef PointXYZI pt = PointXYZI()
    cdef PointCloud[PointXYZI] p = PointCloud[PointXYZI]()
    p.push_back(pt)
    p.push_back(pt)
    return p.size() + test(p)
