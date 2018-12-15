from pcl.common.PCLPointCloud2 cimport PCLPointCloud2
from pcl.common.point_cloud cimport PointCloud

cdef extern from "pcl/conversions.h" namespace "pcl":
    void fromPCLPointCloud2[PointT](const PCLPointCloud2&, PointCloud[PointT]&)
    void toPCLPointCloud2[PointT](const PointCloud[PointT]&, PCLPointCloud2&)
