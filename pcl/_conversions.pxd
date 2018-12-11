from pcl._PCLPointCloud2 cimport PCLPointCloud2
from pcl._point_cloud cimport _point_cloud

cdef extern from "pcl/conversions.h" namespace "pcl":
    def void fromPCLPointCloud2[PointT](const PCLPointCloud2&, PointCloud[PointT]&)
    def void toPCLPointCloud2[Point](const PointCloud[PointT]&, PCLPointCloud2&)
